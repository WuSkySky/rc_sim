#include "MvCameraControl.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_r2_interfaces/msg/camera_frame.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{

using CameraFrame = robot_r2_interfaces::msg::CameraFrame;

constexpr char kImageTopic[] = "/r2/hik_camera/image_raw";
constexpr char kDebugTopic[] = "/r2/hik_camera/image_raw/debug";
constexpr char kCameraInfoTopic[] = "/r2/hik_camera/camera_info";
constexpr int64_t kMinimumFrameTimeoutMs = 1;
constexpr int64_t kMaximumFrameTimeoutMs = 10000;
constexpr int64_t kMinimumConsecutiveFailures = 1;
constexpr int64_t kMaximumConsecutiveFailures = 100;

std::string mvs_status(int status)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << std::uppercase
         << static_cast<uint32_t>(status);
  return stream.str();
}

void require_mvs_ok(int status, const std::string & operation)
{
  if (status != MV_OK) {
    throw std::runtime_error(operation + " failed with " + mvs_status(status));
  }
}

template<std::size_t Size>
std::string mvs_string(const unsigned char (&value)[Size])
{
  const auto * text = reinterpret_cast<const char *>(value);
  return std::string(text, strnlen(text, Size));
}

struct FloatRange
{
  double minimum{0.0};
  double maximum{0.0};
};

bool is_in_range(double value, const FloatRange & range)
{
  return std::isfinite(value) && value >= range.minimum && value <= range.maximum;
}

}  // namespace

class HikCamera final : public rclcpp::Node
{
public:
  HikCamera()
  : Node("hik_camera")
  {
    acquisition_frame_rate_hz_ =
      declare_parameter<double>("acquisition_frame_rate_hz", 30.0);
    exposure_time_us_ = declare_parameter<double>("exposure_time_us", 3000.0);
    gain_db_ = declare_parameter<double>("gain_db", 0.0);
    visualization_enabled_.store(
      declare_parameter<bool>("visualization_enabled", false));
    frame_timeout_ms_.store(
      declare_parameter<int64_t>("frame_timeout_ms", 1000));
    max_consecutive_failures_.store(
      declare_parameter<int64_t>("max_consecutive_failures", 5));

    validate_integer_parameters(
      frame_timeout_ms_.load(), max_consecutive_failures_.load());

    frame_id_ = frame_id_for_node(get_name());
    if (frame_id_.size() > CameraFrame::FRAME_ID_CAPACITY) {
      throw std::invalid_argument("camera frame_id exceeds CameraFrame capacity");
    }

    const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability_volatile();
    image_publisher_ = create_publisher<CameraFrame>(kImageTopic, image_qos);
    debug_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(kDebugTopic, image_qos);
    camera_info_publisher_ =
      create_publisher<sensor_msgs::msg::CameraInfo>(kCameraInfoTopic, 10);

    initialize_messages();

    try {
      initialize_camera();
      parameter_callback_handle_ = add_on_set_parameters_callback(
        std::bind(
          &HikCamera::on_parameters_changed, this, std::placeholders::_1));
      capture_thread_ = std::thread(&HikCamera::capture_loop, this);
    } catch (...) {
      close_camera();
      throw;
    }
  }

  ~HikCamera() override
  {
    stop_requested_.store(true);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    close_camera();
  }

private:
  static std::string frame_id_for_node(const std::string & node_name)
  {
    if (node_name == "front_hik_camera") {
      return "r2_front_camera_optical_frame";
    }
    return "r2_hik_camera_optical_frame";
  }

  static void validate_integer_parameters(
    int64_t frame_timeout_ms, int64_t max_consecutive_failures)
  {
    if (frame_timeout_ms < kMinimumFrameTimeoutMs ||
      frame_timeout_ms > kMaximumFrameTimeoutMs)
    {
      throw std::invalid_argument("frame_timeout_ms must be in [1, 10000]");
    }
    if (max_consecutive_failures < kMinimumConsecutiveFailures ||
      max_consecutive_failures > kMaximumConsecutiveFailures)
    {
      throw std::invalid_argument(
              "max_consecutive_failures must be in [1, 100]");
    }
  }

  void initialize_messages()
  {
    image_message_.frame_id_size = static_cast<uint8_t>(frame_id_.size());
    std::fill(image_message_.frame_id.begin(), image_message_.frame_id.end(), 0U);
    std::memcpy(
      image_message_.frame_id.data(), frame_id_.data(), frame_id_.size());
    image_message_.encoding = CameraFrame::ENCODING_BGR8;
    image_message_.is_bigendian = 0U;
    image_message_.layout_version = CameraFrame::LAYOUT_VERSION;
    image_message_.data.reserve(CameraFrame::DATA_CAPACITY);

    debug_message_.header.frame_id = frame_id_;
    debug_message_.encoding = "bgr8";
    debug_message_.is_bigendian = 0U;
    debug_message_.data.reserve(CameraFrame::DATA_CAPACITY);

    camera_info_message_.header.frame_id = frame_id_;
  }

  void initialize_camera()
  {
    MV_CC_DEVICE_INFO_LIST device_list{};
    require_mvs_ok(
      MV_CC_EnumDevices(MV_USB_DEVICE, &device_list),
      "MV_CC_EnumDevices");
    if (device_list.nDeviceNum == 0) {
      throw std::runtime_error("no HIKROBOT USB camera found");
    }
    if (device_list.nDeviceNum > 1) {
      RCLCPP_WARN(
        get_logger(), "Found %u HIKROBOT USB cameras; using the first one",
        device_list.nDeviceNum);
    }

    const MV_CC_DEVICE_INFO * device = device_list.pDeviceInfo[0];
    if (device == nullptr) {
      throw std::runtime_error("MVS returned an empty camera device entry");
    }
    const auto & usb = device->SpecialInfo.stUsb3VInfo;
    model_name_ = mvs_string(usb.chModelName);
    serial_number_ = mvs_string(usb.chSerialNumber);

    require_mvs_ok(
      MV_CC_CreateHandleWithoutLog(&camera_handle_, device),
      "MV_CC_CreateHandleWithoutLog");
    require_mvs_ok(MV_CC_OpenDevice(camera_handle_), "MV_CC_OpenDevice");
    camera_open_ = true;

    require_mvs_ok(
      MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0),
      "disable TriggerMode");
    require_mvs_ok(
      MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 0),
      "disable ExposureAuto");
    require_mvs_ok(
      MV_CC_SetEnumValue(camera_handle_, "GainAuto", 0),
      "disable GainAuto");
    require_mvs_ok(
      MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true),
      "enable AcquisitionFrameRate");

    frame_rate_range_ = query_float_range("AcquisitionFrameRate");
    exposure_range_ = query_float_range("ExposureTime");
    gain_range_ = query_float_range("Gain");
    validate_camera_parameters(
      acquisition_frame_rate_hz_, exposure_time_us_, gain_db_);

    require_mvs_ok(
      MV_CC_SetFloatValue(
        camera_handle_, "AcquisitionFrameRate",
        static_cast<float>(acquisition_frame_rate_hz_)),
      "set AcquisitionFrameRate");
    require_mvs_ok(
      MV_CC_SetFloatValue(
        camera_handle_, "ExposureTime", static_cast<float>(exposure_time_us_)),
      "set ExposureTime");
    require_mvs_ok(
      MV_CC_SetFloatValue(camera_handle_, "Gain", static_cast<float>(gain_db_)),
      "set Gain");

    MV_IMAGE_BASIC_INFO image_info{};
    require_mvs_ok(
      MV_CC_GetImageInfo(camera_handle_, &image_info), "MV_CC_GetImageInfo");
    const auto maximum_size =
      static_cast<std::size_t>(image_info.nWidthMax) *
      static_cast<std::size_t>(image_info.nHeightMax) * 3U;
    if (maximum_size == 0U || maximum_size > CameraFrame::DATA_CAPACITY) {
      throw std::runtime_error(
              "camera maximum BGR frame exceeds CameraFrame capacity");
    }
    conversion_buffer_.reserve(maximum_size);

    require_mvs_ok(MV_CC_StartGrabbing(camera_handle_), "MV_CC_StartGrabbing");
    camera_grabbing_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Opened HIKROBOT %s serial=%s at %.1f FPS, exposure %.1f us, "
      "gain %.1f dB; publishing %s",
      model_name_.c_str(), serial_number_.c_str(), acquisition_frame_rate_hz_,
      exposure_time_us_, gain_db_, kImageTopic);
  }

  FloatRange query_float_range(const char * name) const
  {
    MVCC_FLOATVALUE value{};
    require_mvs_ok(
      MV_CC_GetFloatValue(camera_handle_, name, &value),
      std::string("query ") + name);
    return FloatRange{value.fMin, value.fMax};
  }

  void validate_camera_parameters(
    double frame_rate, double exposure_time, double gain) const
  {
    if (!is_in_range(frame_rate, frame_rate_range_)) {
      throw std::invalid_argument("acquisition_frame_rate_hz is outside camera range");
    }
    if (!is_in_range(exposure_time, exposure_range_)) {
      throw std::invalid_argument("exposure_time_us is outside camera range");
    }
    if (!is_in_range(gain, gain_range_)) {
      std::ostringstream message;
      message << "gain_db " << gain << " is outside camera range ["
              << gain_range_.minimum << ", " << gain_range_.maximum << "]";
      throw std::invalid_argument(message.str());
    }
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    std::lock_guard<std::mutex> lock(camera_mutex_);

    double next_frame_rate = acquisition_frame_rate_hz_;
    double next_exposure = exposure_time_us_;
    double next_gain = gain_db_;
    bool next_visualization = visualization_enabled_.load();
    int64_t next_timeout = frame_timeout_ms_.load();
    int64_t next_max_failures = max_consecutive_failures_.load();

    try {
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name == "acquisition_frame_rate_hz") {
          next_frame_rate = parameter.as_double();
        } else if (name == "exposure_time_us") {
          next_exposure = parameter.as_double();
        } else if (name == "gain_db") {
          next_gain = parameter.as_double();
        } else if (name == "visualization_enabled") {
          next_visualization = parameter.as_bool();
        } else if (name == "frame_timeout_ms") {
          next_timeout = parameter.as_int();
        } else if (name == "max_consecutive_failures") {
          next_max_failures = parameter.as_int();
        }
      }
      validate_integer_parameters(next_timeout, next_max_failures);
      validate_camera_parameters(next_frame_rate, next_exposure, next_gain);
    } catch (const std::exception & error) {
      result.reason = error.what();
      return result;
    }

    if (camera_handle_ == nullptr || !camera_open_) {
      result.reason = "camera is not open";
      return result;
    }

    const double previous_frame_rate = acquisition_frame_rate_hz_;
    const double previous_exposure = exposure_time_us_;
    const double previous_gain = gain_db_;
    const int frame_rate_status = MV_CC_SetFloatValue(
      camera_handle_, "AcquisitionFrameRate", static_cast<float>(next_frame_rate));
    const int exposure_status = frame_rate_status == MV_OK ?
      MV_CC_SetFloatValue(
      camera_handle_, "ExposureTime", static_cast<float>(next_exposure)) :
      frame_rate_status;
    const int gain_status = exposure_status == MV_OK ?
      MV_CC_SetFloatValue(camera_handle_, "Gain", static_cast<float>(next_gain)) :
      exposure_status;

    if (frame_rate_status != MV_OK || exposure_status != MV_OK || gain_status != MV_OK) {
      const int failure = frame_rate_status != MV_OK ? frame_rate_status :
        (exposure_status != MV_OK ? exposure_status : gain_status);
      const int rollback_frame_rate = MV_CC_SetFloatValue(
        camera_handle_, "AcquisitionFrameRate",
        static_cast<float>(previous_frame_rate));
      const int rollback_exposure = MV_CC_SetFloatValue(
        camera_handle_, "ExposureTime", static_cast<float>(previous_exposure));
      const int rollback_gain = MV_CC_SetFloatValue(
        camera_handle_, "Gain", static_cast<float>(previous_gain));
      if (rollback_frame_rate != MV_OK || rollback_exposure != MV_OK ||
        rollback_gain != MV_OK)
      {
        RCLCPP_ERROR(get_logger(), "Failed to roll back camera parameters");
      }
      result.reason = "MVS rejected parameter update with " + mvs_status(failure);
      return result;
    }

    acquisition_frame_rate_hz_ = next_frame_rate;
    exposure_time_us_ = next_exposure;
    gain_db_ = next_gain;
    visualization_enabled_.store(next_visualization);
    frame_timeout_ms_.store(next_timeout);
    max_consecutive_failures_.store(next_max_failures);
    result.successful = true;
    return result;
  }

  void capture_loop()
  {
    int64_t consecutive_failures = 0;
    while (rclcpp::ok() && !stop_requested_.load()) {
      MV_FRAME_OUT frame{};
      int acquire_status = MV_E_CALLORDER;
      int convert_status = MV_E_CALLORDER;
      int free_status = MV_OK;
      uint32_t width = 0U;
      uint32_t height = 0U;
      std::size_t output_size = 0U;

      {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (camera_handle_ == nullptr || !camera_grabbing_) {
          break;
        }
        acquire_status = MV_CC_GetImageBuffer(
          camera_handle_, &frame,
          static_cast<unsigned int>(frame_timeout_ms_.load()));
        if (acquire_status == MV_OK) {
          width = frame.stFrameInfo.nWidth;
          height = frame.stFrameInfo.nHeight;
          output_size = static_cast<std::size_t>(width) *
            static_cast<std::size_t>(height) * 3U;
          if (width > std::numeric_limits<unsigned short>::max() ||
            height > std::numeric_limits<unsigned short>::max() ||
            output_size == 0U || output_size > CameraFrame::DATA_CAPACITY)
          {
            convert_status = MV_E_PARAMETER;
          } else {
            conversion_buffer_.resize(output_size);
            MV_CC_PIXEL_CONVERT_PARAM convert{};
            convert.nWidth = static_cast<unsigned short>(width);
            convert.nHeight = static_cast<unsigned short>(height);
            convert.enSrcPixelType = frame.stFrameInfo.enPixelType;
            convert.pSrcData = frame.pBufAddr;
            convert.nSrcDataLen = frame.stFrameInfo.nFrameLen;
            convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            convert.pDstBuffer = conversion_buffer_.data();
            convert.nDstBufferSize = static_cast<unsigned int>(output_size);
            convert_status = MV_CC_ConvertPixelType(camera_handle_, &convert);
            if (convert_status == MV_OK && convert.nDstLen != output_size) {
              convert_status = MV_E_PARAMETER;
            }
          }
          free_status = MV_CC_FreeImageBuffer(camera_handle_, &frame);
        }
      }

      if (acquire_status != MV_OK || convert_status != MV_OK || free_status != MV_OK) {
        ++consecutive_failures;
        const int failure = acquire_status != MV_OK ? acquire_status :
          (convert_status != MV_OK ? convert_status : free_status);
        RCLCPP_WARN(
          get_logger(), "Camera frame failed (%s), consecutive failures: %ld",
          mvs_status(failure).c_str(), consecutive_failures);
        if (consecutive_failures >= max_consecutive_failures_.load()) {
          RCLCPP_FATAL(get_logger(), "HIKROBOT camera stopped producing valid frames");
          rclcpp::shutdown();
          return;
        }
        continue;
      }

      consecutive_failures = 0;
      publish_frame(width, height, output_size);
    }
  }

  void publish_frame(uint32_t width, uint32_t height, std::size_t data_size)
  {
    const builtin_interfaces::msg::Time stamp = now();
    auto & image = image_message_;
    image.sequence = sequence_;
    image.stamp_sec = stamp.sec;
    image.stamp_nanosec = stamp.nanosec;
    image.width = width;
    image.height = height;
    image.step = width * 3U;
    image.data_size = static_cast<uint32_t>(data_size);
    image.data.resize(data_size);
    std::memcpy(&image.data[0], conversion_buffer_.data(), data_size);
    image_publisher_->publish(image);

    if (visualization_enabled_.load()) {
      debug_message_.header.stamp = stamp;
      debug_message_.width = width;
      debug_message_.height = height;
      debug_message_.step = width * 3U;
      debug_message_.data.resize(data_size);
      std::memcpy(debug_message_.data.data(), conversion_buffer_.data(), data_size);
      debug_publisher_->publish(debug_message_);
    }

    camera_info_message_.header.stamp = stamp;
    camera_info_message_.width = width;
    camera_info_message_.height = height;
    camera_info_publisher_->publish(camera_info_message_);
    ++sequence_;
  }

  void close_camera()
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (camera_handle_ == nullptr) {
      return;
    }
    if (camera_grabbing_) {
      const int status = MV_CC_StopGrabbing(camera_handle_);
      if (status != MV_OK) {
        RCLCPP_WARN(
          get_logger(), "MV_CC_StopGrabbing failed with %s",
          mvs_status(status).c_str());
      }
      camera_grabbing_ = false;
    }
    if (camera_open_) {
      const int status = MV_CC_CloseDevice(camera_handle_);
      if (status != MV_OK) {
        RCLCPP_WARN(
          get_logger(), "MV_CC_CloseDevice failed with %s",
          mvs_status(status).c_str());
      }
      camera_open_ = false;
    }
    const int status = MV_CC_DestroyHandle(camera_handle_);
    if (status != MV_OK) {
      RCLCPP_WARN(
        get_logger(), "MV_CC_DestroyHandle failed with %s",
        mvs_status(status).c_str());
    }
    camera_handle_ = nullptr;
  }

  std::string frame_id_;
  std::string model_name_;
  std::string serial_number_;

  double acquisition_frame_rate_hz_{30.0};
  double exposure_time_us_{3000.0};
  double gain_db_{0.0};
  FloatRange frame_rate_range_;
  FloatRange exposure_range_;
  FloatRange gain_range_;
  std::atomic_bool visualization_enabled_{false};
  std::atomic<int64_t> frame_timeout_ms_{1000};
  std::atomic<int64_t> max_consecutive_failures_{5};

  void * camera_handle_{nullptr};
  bool camera_open_{false};
  bool camera_grabbing_{false};
  std::mutex camera_mutex_;
  std::atomic_bool stop_requested_{false};
  std::thread capture_thread_;
  std::vector<uint8_t> conversion_buffer_;
  uint64_t sequence_{0U};

  rclcpp::Publisher<CameraFrame>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  CameraFrame image_message_;
  sensor_msgs::msg::Image debug_message_;
  sensor_msgs::msg::CameraInfo camera_info_message_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
  parameter_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<HikCamera>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("hik_camera"), "HIKROBOT camera failed: %s",
      error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
