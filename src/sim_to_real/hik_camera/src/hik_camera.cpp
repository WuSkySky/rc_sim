#include "MvCameraControl.h"

#include <algorithm>
#include <array>
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

namespace {

using CameraFrame = robot_r2_interfaces::msg::CameraFrame;

constexpr char kImageTopic[] = "/r2/hik_camera/image_raw";
constexpr char kDebugTopic[] = "/r2/hik_camera/image_raw/debug";
constexpr char kCameraInfoTopic[] = "/r2/hik_camera/camera_info";
constexpr int64_t kMinimumFrameTimeoutMs = 1;
constexpr int64_t kMaximumFrameTimeoutMs = 10000;
constexpr int64_t kMinimumConsecutiveFailures = 1;
constexpr int64_t kMaximumConsecutiveFailures = 100;

std::string mvs_status(int status) {
  std::ostringstream stream;
  stream << "0x" << std::hex << std::uppercase << static_cast<uint32_t>(status);
  return stream.str();
}

void require_mvs_ok(int status, const std::string &operation) {
  if (status != MV_OK) {
    throw std::runtime_error(operation + " failed with " + mvs_status(status));
  }
}

template <std::size_t Size>
std::string mvs_string(const unsigned char (&value)[Size]) {
  const auto *text = reinterpret_cast<const char *>(value);
  return std::string(text, strnlen(text, Size));
}

struct FloatRange {
  double minimum{0.0};
  double maximum{0.0};
};

struct IntegerRange {
  int64_t current{0};
  int64_t minimum{0};
  int64_t maximum{0};
};

bool is_in_range(double value, const FloatRange &range) {
  return std::isfinite(value) && value >= range.minimum &&
         value <= range.maximum;
}

} // namespace

class HikCamera final : public rclcpp::Node {
public:
  HikCamera() : Node("hik_camera") {
    acquisition_frame_rate_hz_ =
        declare_parameter<double>("acquisition_frame_rate_hz", 30.0);
    exposure_time_us_ = declare_parameter<double>("exposure_time_us", 3000.0);
    gain_db_ = declare_parameter<double>("gain_db", 0.0);
    binning_2x2_enabled_.store(
        declare_parameter<bool>("binning_2x2_enabled", true));
    visualization_enabled_.store(
        declare_parameter<bool>("visualization_enabled", false));
    frame_timeout_ms_.store(
        declare_parameter<int64_t>("frame_timeout_ms", 1000));
    max_consecutive_failures_.store(
        declare_parameter<int64_t>("max_consecutive_failures", 5));
    focal_length_mm_.store(declare_parameter<double>("focal_length_mm", 0.0));
    pixel_size_um_.store(declare_parameter<double>("pixel_size_um", 0.0));
    k1_.store(declare_parameter<double>("k1", 0.0));
    k2_.store(declare_parameter<double>("k2", 0.0));
    p1_.store(declare_parameter<double>("p1", 0.0));
    p2_.store(declare_parameter<double>("p2", 0.0));
    k3_.store(declare_parameter<double>("k3", 0.0));

    validate_integer_parameters(frame_timeout_ms_.load(),
                                max_consecutive_failures_.load());
    validate_intrinsic_parameters(focal_length_mm_.load(),
                                  pixel_size_um_.load(), k1_.load(),
                                  k2_.load(), p1_.load(), p2_.load(),
                                  k3_.load());

    frame_id_ = frame_id_for_node(get_name());
    if (frame_id_.size() > CameraFrame::FRAME_ID_CAPACITY) {
      throw std::invalid_argument(
          "camera frame_id exceeds CameraFrame capacity");
    }

    const auto image_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    image_publisher_ = create_publisher<CameraFrame>(kImageTopic, image_qos);
    debug_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(kDebugTopic, image_qos);
    camera_info_publisher_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(kCameraInfoTopic, 10);

    initialize_messages();

    try {
      initialize_camera();
      parameter_callback_handle_ = add_on_set_parameters_callback(std::bind(
          &HikCamera::on_parameters_changed, this, std::placeholders::_1));
      capture_thread_ = std::thread(&HikCamera::capture_loop, this);
    } catch (...) {
      close_camera();
      throw;
    }
  }

  ~HikCamera() override {
    stop_requested_.store(true);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    close_camera();
  }

private:
  static std::string frame_id_for_node(const std::string &node_name) {
    if (node_name == "front_hik_camera") {
      return "r2_front_camera_optical_frame";
    }
    return "r2_hik_camera_optical_frame";
  }

  static void validate_integer_parameters(int64_t frame_timeout_ms,
                                          int64_t max_consecutive_failures) {
    if (frame_timeout_ms < kMinimumFrameTimeoutMs ||
        frame_timeout_ms > kMaximumFrameTimeoutMs) {
      throw std::invalid_argument("frame_timeout_ms must be in [1, 10000]");
    }
    if (max_consecutive_failures < kMinimumConsecutiveFailures ||
        max_consecutive_failures > kMaximumConsecutiveFailures) {
      throw std::invalid_argument(
          "max_consecutive_failures must be in [1, 100]");
    }
  }

  static void validate_intrinsic_parameters(double focal_length_mm,
                                            double pixel_size_um, double k1,
                                            double k2, double p1, double p2,
                                            double k3) {
    if (!std::isfinite(focal_length_mm) || focal_length_mm < 0.0) {
      throw std::invalid_argument("focal_length_mm must be finite and >= 0");
    }
    if (!std::isfinite(pixel_size_um) || pixel_size_um < 0.0) {
      throw std::invalid_argument("pixel_size_um must be finite and >= 0");
    }
    if (focal_length_mm > 0.0 && pixel_size_um <= 0.0) {
      throw std::invalid_argument(
          "pixel_size_um must be positive when focal_length_mm is positive");
    }
    for (double coefficient : {k1, k2, p1, p2, k3}) {
      if (!std::isfinite(coefficient)) {
        throw std::invalid_argument(
            "distortion coefficients must be finite");
      }
    }
  }

  void initialize_messages() {
    image_message_.frame_id_size = static_cast<uint8_t>(frame_id_.size());
    std::fill(image_message_.frame_id.begin(), image_message_.frame_id.end(),
              0U);
    std::memcpy(image_message_.frame_id.data(), frame_id_.data(),
                frame_id_.size());
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

  void initialize_camera() {
    MV_CC_DEVICE_INFO_LIST device_list{};
    require_mvs_ok(MV_CC_EnumDevices(MV_USB_DEVICE, &device_list),
                   "MV_CC_EnumDevices");
    if (device_list.nDeviceNum == 0) {
      throw std::runtime_error("no HIKROBOT USB camera found");
    }
    if (device_list.nDeviceNum > 1) {
      RCLCPP_WARN(get_logger(),
                  "Found %u HIKROBOT USB cameras; using the first one",
                  device_list.nDeviceNum);
    }

    const MV_CC_DEVICE_INFO *device = device_list.pDeviceInfo[0];
    if (device == nullptr) {
      throw std::runtime_error("MVS returned an empty camera device entry");
    }
    const auto &usb = device->SpecialInfo.stUsb3VInfo;
    model_name_ = mvs_string(usb.chModelName);
    serial_number_ = mvs_string(usb.chSerialNumber);

    require_mvs_ok(MV_CC_CreateHandleWithoutLog(&camera_handle_, device),
                   "MV_CC_CreateHandleWithoutLog");
    require_mvs_ok(MV_CC_OpenDevice(camera_handle_), "MV_CC_OpenDevice");
    camera_open_ = true;

    require_mvs_ok(MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0),
                   "disable TriggerMode");
    require_mvs_ok(MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 0),
                   "disable ExposureAuto");
    require_mvs_ok(MV_CC_SetEnumValue(camera_handle_, "GainAuto", 0),
                   "disable GainAuto");
    require_mvs_ok(
        MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true),
        "enable AcquisitionFrameRate");

    apply_full_frame_binning(binning_2x2_enabled_.load());

    frame_rate_range_ = query_float_range("AcquisitionFrameRate");
    exposure_range_ = query_float_range("ExposureTime");
    gain_range_ = query_float_range("Gain");
    validate_camera_parameters(acquisition_frame_rate_hz_, exposure_time_us_,
                               gain_db_);

    require_mvs_ok(
        MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate",
                            static_cast<float>(acquisition_frame_rate_hz_)),
        "set AcquisitionFrameRate");
    require_mvs_ok(MV_CC_SetFloatValue(camera_handle_, "ExposureTime",
                                       static_cast<float>(exposure_time_us_)),
                   "set ExposureTime");
    require_mvs_ok(MV_CC_SetFloatValue(camera_handle_, "Gain",
                                       static_cast<float>(gain_db_)),
                   "set Gain");

    MV_IMAGE_BASIC_INFO image_info{};
    require_mvs_ok(MV_CC_GetImageInfo(camera_handle_, &image_info),
                   "MV_CC_GetImageInfo");
    const auto maximum_size = static_cast<std::size_t>(image_info.nWidthMax) *
                              static_cast<std::size_t>(image_info.nHeightMax) *
                              3U;
    if (maximum_size == 0U || maximum_size > CameraFrame::DATA_CAPACITY) {
      throw std::runtime_error(
          "camera maximum BGR frame exceeds CameraFrame capacity");
    }
    conversion_buffer_.reserve(maximum_size);

    require_mvs_ok(MV_CC_StartGrabbing(camera_handle_), "MV_CC_StartGrabbing");
    camera_grabbing_ = true;

    RCLCPP_INFO(get_logger(),
                "Opened HIKROBOT %s serial=%s at %.1f FPS, exposure %.1f us, "
                "gain %.1f dB, 2x2 binning %s, output %ux%u; publishing %s",
                model_name_.c_str(), serial_number_.c_str(),
                acquisition_frame_rate_hz_, exposure_time_us_, gain_db_,
                binning_2x2_enabled_.load() ? "enabled" : "disabled",
                static_cast<unsigned int>(image_info.nWidthValue),
                image_info.nHeightValue, kImageTopic);
  }

  FloatRange query_float_range(const char *name) const {
    MVCC_FLOATVALUE value{};
    require_mvs_ok(MV_CC_GetFloatValue(camera_handle_, name, &value),
                   std::string("query ") + name);
    return FloatRange{value.fMin, value.fMax};
  }

  IntegerRange query_integer_range(const char *name) const {
    MVCC_INTVALUE_EX value{};
    require_mvs_ok(MV_CC_GetIntValueEx(camera_handle_, name, &value),
                   std::string("query ") + name);
    return IntegerRange{value.nCurValue, value.nMin, value.nMax};
  }

  bool enum_value_is_supported(const char *name, unsigned int requested) const {
    MVCC_ENUMVALUE value{};
    require_mvs_ok(MV_CC_GetEnumValue(camera_handle_, name, &value),
                   std::string("query ") + name);
    for (unsigned int index = 0; index < value.nSupportedNum; ++index) {
      if (value.nSupportValue[index] == requested) {
        return true;
      }
    }
    return false;
  }

  void apply_full_frame_binning(bool enabled) {
    const unsigned int binning_value = enabled ? 2U : 1U;
    if (!enum_value_is_supported("BinningHorizontal", binning_value) ||
        !enum_value_is_supported("BinningVertical", binning_value)) {
      throw std::invalid_argument(enabled
                                      ? "camera does not support 2x2 binning"
                                      : "camera does not support 1x1 binning");
    }

    // Shrink the current ROI before changing binning so it remains valid when
    // the sensor dimensions change. Then expand to the new mode's maximum ROI
    // with zero offsets, which preserves the complete field of view.
    const auto current_offset_x_range = query_integer_range("OffsetX");
    const auto current_offset_y_range = query_integer_range("OffsetY");
    require_mvs_ok(MV_CC_SetIntValueEx(camera_handle_, "OffsetX",
                                       current_offset_x_range.minimum),
                   "reset OffsetX");
    require_mvs_ok(MV_CC_SetIntValueEx(camera_handle_, "OffsetY",
                                       current_offset_y_range.minimum),
                   "reset OffsetY");
    const auto current_width_range = query_integer_range("Width");
    const auto current_height_range = query_integer_range("Height");
    require_mvs_ok(MV_CC_SetIntValueEx(camera_handle_, "Width",
                                       current_width_range.minimum),
                   "minimize Width");
    require_mvs_ok(MV_CC_SetIntValueEx(camera_handle_, "Height",
                                       current_height_range.minimum),
                   "minimize Height");

    require_mvs_ok(
        MV_CC_SetEnumValue(camera_handle_, "BinningHorizontal", binning_value),
        "set BinningHorizontal");
    require_mvs_ok(
        MV_CC_SetEnumValue(camera_handle_, "BinningVertical", binning_value),
        "set BinningVertical");

    const auto next_width_range = query_integer_range("Width");
    const auto next_height_range = query_integer_range("Height");
    require_mvs_ok(
        MV_CC_SetIntValueEx(camera_handle_, "Width", next_width_range.maximum),
        "maximize Width");
    require_mvs_ok(MV_CC_SetIntValueEx(camera_handle_, "Height",
                                       next_height_range.maximum),
                   "maximize Height");

    const auto next_offset_x_range = query_integer_range("OffsetX");
    const auto next_offset_y_range = query_integer_range("OffsetY");
    require_mvs_ok(MV_CC_SetIntValueEx(camera_handle_, "OffsetX",
                                       next_offset_x_range.minimum),
                   "set OffsetX");
    require_mvs_ok(MV_CC_SetIntValueEx(camera_handle_, "OffsetY",
                                       next_offset_y_range.minimum),
                   "set OffsetY");
  }

  void validate_camera_parameters(double frame_rate, double exposure_time,
                                  double gain) const {
    if (!is_in_range(frame_rate, frame_rate_range_)) {
      throw std::invalid_argument(
          "acquisition_frame_rate_hz is outside camera range");
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

  void apply_float_parameters(double frame_rate, double exposure_time,
                              double gain) {
    require_mvs_ok(MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate",
                                       static_cast<float>(frame_rate)),
                   "set AcquisitionFrameRate");
    require_mvs_ok(MV_CC_SetFloatValue(camera_handle_, "ExposureTime",
                                       static_cast<float>(exposure_time)),
                   "set ExposureTime");
    require_mvs_ok(
        MV_CC_SetFloatValue(camera_handle_, "Gain", static_cast<float>(gain)),
        "set Gain");
  }

  void stop_grabbing() {
    if (!camera_grabbing_) {
      return;
    }
    require_mvs_ok(MV_CC_StopGrabbing(camera_handle_), "MV_CC_StopGrabbing");
    camera_grabbing_ = false;
  }

  void start_grabbing() {
    if (camera_grabbing_) {
      return;
    }
    require_mvs_ok(MV_CC_StartGrabbing(camera_handle_), "MV_CC_StartGrabbing");
    camera_grabbing_ = true;
  }

  rcl_interfaces::msg::SetParametersResult
  on_parameters_changed(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    std::lock_guard<std::mutex> lock(camera_mutex_);

    double next_frame_rate = acquisition_frame_rate_hz_;
    double next_exposure = exposure_time_us_;
    double next_gain = gain_db_;
    bool next_binning_2x2_enabled = binning_2x2_enabled_.load();
    bool next_visualization = visualization_enabled_.load();
    int64_t next_timeout = frame_timeout_ms_.load();
    int64_t next_max_failures = max_consecutive_failures_.load();
    double next_focal_length_mm = focal_length_mm_.load();
    double next_pixel_size_um = pixel_size_um_.load();
    double next_k1 = k1_.load();
    double next_k2 = k2_.load();
    double next_p1 = p1_.load();
    double next_p2 = p2_.load();
    double next_k3 = k3_.load();

    try {
      for (const auto &parameter : parameters) {
        const auto &name = parameter.get_name();
        if (name == "acquisition_frame_rate_hz") {
          next_frame_rate = parameter.as_double();
        } else if (name == "exposure_time_us") {
          next_exposure = parameter.as_double();
        } else if (name == "gain_db") {
          next_gain = parameter.as_double();
        } else if (name == "binning_2x2_enabled") {
          next_binning_2x2_enabled = parameter.as_bool();
        } else if (name == "visualization_enabled") {
          next_visualization = parameter.as_bool();
        } else if (name == "frame_timeout_ms") {
          next_timeout = parameter.as_int();
        } else if (name == "max_consecutive_failures") {
          next_max_failures = parameter.as_int();
        } else if (name == "focal_length_mm") {
          next_focal_length_mm = parameter.as_double();
        } else if (name == "pixel_size_um") {
          next_pixel_size_um = parameter.as_double();
        } else if (name == "k1") {
          next_k1 = parameter.as_double();
        } else if (name == "k2") {
          next_k2 = parameter.as_double();
        } else if (name == "p1") {
          next_p1 = parameter.as_double();
        } else if (name == "p2") {
          next_p2 = parameter.as_double();
        } else if (name == "k3") {
          next_k3 = parameter.as_double();
        }
      }
      validate_integer_parameters(next_timeout, next_max_failures);
      validate_intrinsic_parameters(next_focal_length_mm, next_pixel_size_um,
                                    next_k1, next_k2, next_p1, next_p2,
                                    next_k3);
    } catch (const std::exception &error) {
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
    const bool previous_binning_2x2_enabled = binning_2x2_enabled_.load();
    const bool binning_changed =
        next_binning_2x2_enabled != previous_binning_2x2_enabled;
    bool binning_reconfiguration_started = false;

    try {
      if (binning_changed) {
        stop_grabbing();
        binning_reconfiguration_started = true;
        apply_full_frame_binning(next_binning_2x2_enabled);
        frame_rate_range_ = query_float_range("AcquisitionFrameRate");
        exposure_range_ = query_float_range("ExposureTime");
        gain_range_ = query_float_range("Gain");
      }
      validate_camera_parameters(next_frame_rate, next_exposure, next_gain);
      apply_float_parameters(next_frame_rate, next_exposure, next_gain);
      if (binning_changed) {
        start_grabbing();
      }
    } catch (const std::exception &error) {
      const std::string update_error = error.what();
      try {
        if (binning_reconfiguration_started) {
          stop_grabbing();
          apply_full_frame_binning(previous_binning_2x2_enabled);
          frame_rate_range_ = query_float_range("AcquisitionFrameRate");
          exposure_range_ = query_float_range("ExposureTime");
          gain_range_ = query_float_range("Gain");
        }
        apply_float_parameters(previous_frame_rate, previous_exposure,
                               previous_gain);
        if (binning_reconfiguration_started) {
          start_grabbing();
        }
      } catch (const std::exception &rollback_error) {
        RCLCPP_ERROR(get_logger(), "Failed to roll back camera parameters: %s",
                     rollback_error.what());
      }
      result.reason = update_error;
      return result;
    }

    acquisition_frame_rate_hz_ = next_frame_rate;
    exposure_time_us_ = next_exposure;
    gain_db_ = next_gain;
    binning_2x2_enabled_.store(next_binning_2x2_enabled);
    focal_length_mm_.store(next_focal_length_mm);
    pixel_size_um_.store(next_pixel_size_um);
    k1_.store(next_k1);
    k2_.store(next_k2);
    p1_.store(next_p1);
    p2_.store(next_p2);
    k3_.store(next_k3);
    visualization_enabled_.store(next_visualization);
    frame_timeout_ms_.store(next_timeout);
    max_consecutive_failures_.store(next_max_failures);
    result.successful = true;
    if (binning_changed) {
      RCLCPP_INFO(get_logger(), "2x2 binning %s",
                  binning_2x2_enabled_.load() ? "enabled" : "disabled");
    }
    return result;
  }

  void capture_loop() {
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
              output_size == 0U || output_size > CameraFrame::DATA_CAPACITY) {
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

      if (acquire_status != MV_OK || convert_status != MV_OK ||
          free_status != MV_OK) {
        ++consecutive_failures;
        const int failure =
            acquire_status != MV_OK
                ? acquire_status
                : (convert_status != MV_OK ? convert_status : free_status);
        RCLCPP_WARN(get_logger(),
                    "Camera frame failed (%s), consecutive failures: %ld",
                    mvs_status(failure).c_str(), consecutive_failures);
        if (consecutive_failures >= max_consecutive_failures_.load()) {
          RCLCPP_FATAL(get_logger(),
                       "HIKROBOT camera stopped producing valid frames");
          rclcpp::shutdown();
          return;
        }
        continue;
      }

      consecutive_failures = 0;
      publish_frame(width, height, output_size);
    }
  }

  void fill_intrinsics(sensor_msgs::msg::CameraInfo &info, double width,
                       double height) const {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    const double focal_length_mm = focal_length_mm_.load();
    const double pixel_size_um = pixel_size_um_.load();
    const double binning_scale = binning_2x2_enabled_.load() ? 2.0 : 1.0;

    double focal_length_px = std::max(width, height);
    if (focal_length_mm > 0.0 && pixel_size_um > 0.0) {
      focal_length_px =
          focal_length_mm * 1000.0 / (pixel_size_um * binning_scale);
    }

    const double cx = width / 2.0;
    const double cy = height / 2.0;

    info.k = {focal_length_px, 0.0, cx, 0.0, focal_length_px, cy, 0.0, 0.0,
              1.0};
    info.p = {focal_length_px, 0.0, cx, 0.0, 0.0, focal_length_px, cy, 0.0,
              0.0, 0.0, 1.0, 0.0};
    info.d = {k1_.load(), k2_.load(), p1_.load(), p2_.load(), k3_.load()};
    info.distortion_model = "plumb_bob";
  }

  void publish_frame(uint32_t width, uint32_t height, std::size_t data_size) {
    const builtin_interfaces::msg::Time stamp = now();
    auto &image = image_message_;
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
      std::memcpy(debug_message_.data.data(), conversion_buffer_.data(),
                  data_size);
      debug_publisher_->publish(debug_message_);
    }

    camera_info_message_.header.stamp = stamp;
    camera_info_message_.width = width;
    camera_info_message_.height = height;
    fill_intrinsics(camera_info_message_, static_cast<double>(width),
                    static_cast<double>(height));
    camera_info_publisher_->publish(camera_info_message_);
    ++sequence_;
  }

  void close_camera() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (camera_handle_ == nullptr) {
      return;
    }
    if (camera_grabbing_) {
      const int status = MV_CC_StopGrabbing(camera_handle_);
      if (status != MV_OK) {
        RCLCPP_WARN(get_logger(), "MV_CC_StopGrabbing failed with %s",
                    mvs_status(status).c_str());
      }
      camera_grabbing_ = false;
    }
    if (camera_open_) {
      const int status = MV_CC_CloseDevice(camera_handle_);
      if (status != MV_OK) {
        RCLCPP_WARN(get_logger(), "MV_CC_CloseDevice failed with %s",
                    mvs_status(status).c_str());
      }
      camera_open_ = false;
    }
    const int status = MV_CC_DestroyHandle(camera_handle_);
    if (status != MV_OK) {
      RCLCPP_WARN(get_logger(), "MV_CC_DestroyHandle failed with %s",
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
  std::atomic_bool binning_2x2_enabled_{true};
  std::atomic<double> focal_length_mm_{0.0};
  std::atomic<double> pixel_size_um_{0.0};
  std::atomic<double> k1_{0.0};
  std::atomic<double> k2_{0.0};
  std::atomic<double> p1_{0.0};
  std::atomic<double> p2_{0.0};
  std::atomic<double> k3_{0.0};
  FloatRange frame_rate_range_;
  FloatRange exposure_range_;
  FloatRange gain_range_;
  std::atomic_bool visualization_enabled_{false};
  std::atomic<int64_t> frame_timeout_ms_{1000};
  std::atomic<int64_t> max_consecutive_failures_{5};

  void *camera_handle_{nullptr};
  bool camera_open_{false};
  bool camera_grabbing_{false};
  mutable std::mutex camera_mutex_;
  std::atomic_bool stop_requested_{false};
  std::thread capture_thread_;
  std::vector<uint8_t> conversion_buffer_;
  uint64_t sequence_{0U};

  rclcpp::Publisher<CameraFrame>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_publisher_;
  CameraFrame image_message_;
  sensor_msgs::msg::Image debug_message_;
  sensor_msgs::msg::CameraInfo camera_info_message_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<HikCamera>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("hik_camera"), "HIKROBOT camera failed: %s",
                 error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
