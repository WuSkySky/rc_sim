#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "robot_r2_interfaces/msg/camera_frame.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{

using CameraMode = std::array<int64_t, 3>;
using CameraFrame = robot_r2_interfaces::msg::CameraFrame;

constexpr std::array<CameraMode, 5> kSupportedModes{{
  {1920, 1080, 30},
  {1280, 720, 60},
  {1280, 720, 30},
  {640, 480 ,60},
  {640, 480, 30},
}};

std::string mode_to_string(const CameraMode & mode)
{
  std::ostringstream stream;
  stream << '[' << mode[0] << ", " << mode[1] << ", " << mode[2] << ']';
  return stream.str();
}

}  // namespace

class MipiCamera : public rclcpp::Node
{
public:
  MipiCamera()
  : Node("mipi_camera")
  {
    device_ = declare_parameter<std::string>("device", "/dev/mipi_right");
    const auto mode_values = declare_parameter<std::vector<int64_t>>(
      "mode", {1280, 720, 60});
    flip_method_ = declare_parameter<int64_t>("flip_method", 0);
    frame_id_ = frame_id_for_node(get_name());
    visualization_enabled_.store(
      declare_parameter<bool>("visualization_enabled", false));

    const auto mode = validate_mode(mode_values);
    width_ = static_cast<int>(mode[0]);
    height_ = static_cast<int>(mode[1]);
    framerate_ = static_cast<int>(mode[2]);

    if (flip_method_ < 0 || flip_method_ > 7) {
      throw std::invalid_argument("flip_method must be between 0 and 7");
    }
    if (frame_id_.empty()) {
      throw std::invalid_argument("frame_id must not be empty");
    }
    if (frame_id_.size() > CameraFrame::FRAME_ID_CAPACITY) {
      throw std::invalid_argument(
              "frame_id exceeds CameraFrame capacity of " +
              std::to_string(CameraFrame::FRAME_ID_CAPACITY) + " bytes");
    }
    if (!std::filesystem::exists(device_)) {
      throw std::runtime_error("camera device does not exist: " + device_);
    }

    resolved_device_ = std::filesystem::canonical(device_).string();
    sensor_id_ = sensor_id_from_device(resolved_device_);

    const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability_volatile();
    image_publisher_ = create_publisher<CameraFrame>(
      "/r2/mipi_camera/image_raw", image_qos);
    image_message_.frame_id_size = static_cast<uint8_t>(frame_id_.size());
    std::memcpy(
      image_message_.frame_id.data(), frame_id_.data(), frame_id_.size());
    image_message_.encoding = CameraFrame::ENCODING_BGR8;
    image_message_.is_bigendian = 0U;
    image_message_.layout_version = CameraFrame::LAYOUT_VERSION;
    image_message_.data.reserve(CameraFrame::DATA_CAPACITY);
    standard_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(
      "/r2/mipi_camera/image_raw/debug", image_qos);
    standard_image_message_.header.frame_id = frame_id_;
    standard_image_message_.encoding = "bgr8";
    standard_image_message_.is_bigendian = 0U;
    standard_image_message_.data.reserve(CameraFrame::DATA_CAPACITY);
    camera_info_publisher_ =
      create_publisher<sensor_msgs::msg::CameraInfo>(
      "/r2/mipi_camera/camera_info", rclcpp::QoS(10));
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(
        &MipiCamera::on_parameters_changed, this,
        std::placeholders::_1));

    start_pipeline();
    bus_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MipiCamera::check_bus, this));

    RCLCPP_INFO(
      get_logger(),
      "Capturing %s (%s) as Argus sensor-id=%d, %dx%d@%d FPS; "
      "publishing bounded CameraFrame samples on "
      "/r2/mipi_camera/image_raw",
      device_.c_str(), resolved_device_.c_str(), sensor_id_,
      width_, height_, framerate_);
  }

  ~MipiCamera() override
  {
    shutting_down_.store(true);
    if (sink_ != nullptr && new_sample_handler_ != 0) {
      g_signal_handler_disconnect(sink_, new_sample_handler_);
      new_sample_handler_ = 0;
    }
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (bus_ != nullptr) {
      gst_object_unref(bus_);
      bus_ = nullptr;
    }
    if (sink_ != nullptr) {
      gst_object_unref(sink_);
      sink_ = nullptr;
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

private:
  static std::string frame_id_for_node(const std::string & node_name)
  {
    if (node_name == "left_mipi_camera") {
      return "r2_left_camera_optical_frame";
    }
    if (node_name == "right_mipi_camera") {
      return "r2_right_camera_optical_frame";
    }
    if (node_name == "tip_mipi_camera") {
      return "r2_tip_camera_optical_frame";
    }
    return "r2_mipi_camera_optical_frame";
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() != "visualization_enabled") {
        continue;
      }
      if (parameter.get_type() !=
        rclcpp::ParameterType::PARAMETER_BOOL)
      {
        result.successful = false;
        result.reason = "visualization_enabled must be a boolean";
        return result;
      }
      visualization_enabled_.store(parameter.as_bool());
      RCLCPP_INFO(
        get_logger(), "Camera debug image publication %s",
        parameter.as_bool() ? "enabled" : "disabled");
    }
    return result;
  }

  static CameraMode validate_mode(const std::vector<int64_t> & values)
  {
    if (values.size() != 3) {
      throw std::invalid_argument(
              "mode must contain [width, height, framerate]");
    }

    const CameraMode mode{values[0], values[1], values[2]};
    for (const auto & supported_mode : kSupportedModes) {
      if (mode == supported_mode) {
        return mode;
      }
    }

    std::ostringstream supported;
    for (std::size_t index = 0; index < kSupportedModes.size(); ++index) {
      if (index > 0) {
        supported << ", ";
      }
      supported << mode_to_string(kSupportedModes[index]);
    }
    throw std::invalid_argument(
            "unsupported camera mode " + mode_to_string(mode) +
            "; supported modes: " + supported.str());
  }

  static int sensor_id_from_device(const std::string & device)
  {
    static const std::regex pattern(R"(video([0-9]+))");
    const auto basename = std::filesystem::path(device).filename().string();
    std::smatch match;
    if (!std::regex_match(basename, match, pattern)) {
      throw std::invalid_argument(
              "device must resolve to a video device such as /dev/video1");
    }
    return std::stoi(match[1].str());
  }

  std::string make_pipeline_description() const
  {
    std::ostringstream pipeline;
    pipeline
      << "nvarguscamerasrc sensor-id=" << sensor_id_ << " ! "
      << "video/x-raw(memory:NVMM),width=(int)" << width_
      << ",height=(int)" << height_
      << ",format=(string)NV12,framerate=(fraction)" << framerate_
      << "/1 ! "
      << "nvvidconv flip-method=" << flip_method_ << " ! "
      << "video/x-raw,width=(int)" << width_
      << ",height=(int)" << height_
      << ",format=(string)BGRx ! "
      << "videoconvert ! video/x-raw,format=(string)BGR ! "
      << "appsink name=camera_sink emit-signals=true "
      << "max-buffers=1 drop=true sync=false";
    return pipeline.str();
  }

  void start_pipeline()
  {
    gst_init(nullptr, nullptr);

    GError * parse_error = nullptr;
    pipeline_ = gst_parse_launch(
      make_pipeline_description().c_str(), &parse_error);
    if (parse_error != nullptr) {
      const std::string message = parse_error->message;
      g_error_free(parse_error);
      if (pipeline_ != nullptr) {
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
      }
      throw std::runtime_error(
              "failed to create GStreamer pipeline: " + message);
    }
    if (pipeline_ == nullptr) {
      throw std::runtime_error("failed to create GStreamer pipeline");
    }

    sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "camera_sink");
    if (sink_ == nullptr || !GST_IS_APP_SINK(sink_)) {
      throw std::runtime_error("failed to create the GStreamer appsink");
    }
    new_sample_handler_ = g_signal_connect(
      sink_, "new-sample", G_CALLBACK(on_new_sample), this);

    bus_ = gst_element_get_bus(pipeline_);
    if (bus_ == nullptr) {
      throw std::runtime_error("failed to get the GStreamer message bus");
    }

    const auto state_result = gst_element_set_state(
      pipeline_, GST_STATE_PLAYING);
    if (state_result == GST_STATE_CHANGE_FAILURE) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      throw std::runtime_error(
              "failed to start the GStreamer camera pipeline");
    }
  }

  static GstFlowReturn on_new_sample(
    GstAppSink * sink, gpointer user_data)
  {
    return static_cast<MipiCamera *>(user_data)->publish_sample(sink);
  }

  GstFlowReturn publish_sample(GstAppSink * sink)
  {
    if (shutting_down_.load() || pipeline_failed_.load()) {
      return GST_FLOW_FLUSHING;
    }

    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (shutting_down_.load() || pipeline_failed_.load()) {
      return GST_FLOW_FLUSHING;
    }

    GstSample * sample = gst_app_sink_pull_sample(sink);
    if (sample == nullptr) {
      RCLCPP_ERROR(get_logger(), "Failed to pull camera sample");
      return GST_FLOW_ERROR;
    }

    GstCaps * caps = gst_sample_get_caps(sample);
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    if (caps == nullptr || buffer == nullptr) {
      gst_sample_unref(sample);
      RCLCPP_ERROR(get_logger(), "Camera sample has no caps or buffer");
      return GST_FLOW_ERROR;
    }

    const GstStructure * structure = gst_caps_get_structure(caps, 0);
    int width = 0;
    int height = 0;
    if (!gst_structure_get_int(structure, "width", &width) ||
      !gst_structure_get_int(structure, "height", &height))
    {
      gst_sample_unref(sample);
      RCLCPP_ERROR(get_logger(), "Camera sample has invalid dimensions");
      return GST_FLOW_ERROR;
    }

    GstMapInfo map_info = GST_MAP_INFO_INIT;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
      gst_sample_unref(sample);
      RCLCPP_ERROR(get_logger(), "Failed to map camera frame");
      return GST_FLOW_ERROR;
    }

    if (width <= 0 || height <= 0 || width != width_ || height != height_) {
      RCLCPP_ERROR(
        get_logger(),
        "Camera frame dimensions %dx%d do not match configured mode %dx%d; "
        "stopping publication",
        width, height, width_, height_);
      pipeline_failed_.store(true);
      gst_buffer_unmap(buffer, &map_info);
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    const char * format = gst_structure_get_string(structure, "format");
    if (format == nullptr || std::string(format) != "BGR") {
      RCLCPP_ERROR(
        get_logger(),
        "Camera frame encoding is '%s', expected BGR; stopping publication",
        format != nullptr ? format : "<missing>");
      pipeline_failed_.store(true);
      gst_buffer_unmap(buffer, &map_info);
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    const auto minimum_step = static_cast<std::size_t>(width) * 3U;
    const auto minimum_size =
      minimum_step * static_cast<std::size_t>(height);
    if (map_info.size < minimum_size) {
      RCLCPP_ERROR(
        get_logger(),
        "Camera frame is too small: got %zu bytes, expected at least %zu",
        map_info.size, minimum_size);
      pipeline_failed_.store(true);
      gst_buffer_unmap(buffer, &map_info);
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    std::size_t step = minimum_step;
    std::size_t data_size = minimum_size;
    if (map_info.size % static_cast<std::size_t>(height) == 0U) {
      step = map_info.size / static_cast<std::size_t>(height);
      data_size = map_info.size;
    }

    if (step < minimum_step ||
      step > std::numeric_limits<uint32_t>::max() ||
      data_size != step * static_cast<std::size_t>(height) ||
      data_size > CameraFrame::DATA_CAPACITY)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Camera frame layout is invalid or exceeds CameraFrame capacity: "
        "width=%d height=%d step=%zu data_size=%zu capacity=%u; "
        "stopping publication",
        width, height, step, data_size, CameraFrame::DATA_CAPACITY);
      pipeline_failed_.store(true);
      gst_buffer_unmap(buffer, &map_info);
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    const builtin_interfaces::msg::Time stamp = now();
    bool sample_released = false;
    try {
      auto & image = image_message_;
      image.sequence = sequence_;
      image.stamp_sec = stamp.sec;
      image.stamp_nanosec = stamp.nanosec;
      image.width = static_cast<uint32_t>(width);
      image.height = static_cast<uint32_t>(height);
      image.step = static_cast<uint32_t>(step);
      image.data_size = static_cast<uint32_t>(data_size);
      image.data.resize(data_size);
      std::memcpy(&image.data[0], map_info.data, data_size);

      gst_buffer_unmap(buffer, &map_info);
      gst_sample_unref(sample);
      sample_released = true;

      image_publisher_->publish(image);
      if (visualization_enabled_.load()) {
        auto & standard = standard_image_message_;
        standard.header.stamp = stamp;
        standard.height = image.height;
        standard.width = image.width;
        standard.step = image.step;
        standard.data.resize(data_size);
        std::memcpy(standard.data.data(), &image.data[0], data_size);
        standard_image_publisher_->publish(standard);
      }
      ++sequence_;
    } catch (const std::exception & error) {
      if (!sample_released) {
        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
      }
      pipeline_failed_.store(true);
      RCLCPP_ERROR(
        get_logger(), "Failed to publish a bounded camera frame: %s",
        error.what());
      return GST_FLOW_ERROR;
    }

    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header.stamp = stamp;
    camera_info.header.frame_id = frame_id_;
    camera_info.height = static_cast<uint32_t>(height);
    camera_info.width = static_cast<uint32_t>(width);

    camera_info_publisher_->publish(std::move(camera_info));
    return GST_FLOW_OK;
  }

  void check_bus()
  {
    if (bus_ == nullptr) {
      return;
    }

    constexpr auto message_types = static_cast<GstMessageType>(
      GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_WARNING);
    while (GstMessage * message = gst_bus_pop_filtered(bus_, message_types)) {
      if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_WARNING) {
        GError * error = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_warning(message, &error, &debug);
        RCLCPP_WARN(
          get_logger(), "GStreamer warning: %s; %s",
          error != nullptr ? error->message : "unknown warning",
          debug != nullptr ? debug : "no details");
        if (error != nullptr) {
          g_error_free(error);
        }
        g_free(debug);
        gst_message_unref(message);
        continue;
      }

      pipeline_failed_.store(true);
      if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
        GError * error = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_error(message, &error, &debug);
        RCLCPP_ERROR(
          get_logger(), "GStreamer camera error: %s; %s",
          error != nullptr ? error->message : "unknown error",
          debug != nullptr ? debug : "no details");
        if (error != nullptr) {
          g_error_free(error);
        }
        g_free(debug);
      } else {
        RCLCPP_ERROR(get_logger(), "GStreamer camera stream ended");
      }
      gst_message_unref(message);
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      return;
    }
  }

  std::string device_;
  std::string resolved_device_;
  std::string frame_id_;
  int width_{0};
  int height_{0};
  int framerate_{0};
  int sensor_id_{0};
  int64_t flip_method_{0};

  rclcpp::Publisher<CameraFrame>::SharedPtr image_publisher_;
  CameraFrame image_message_;
  std::atomic_bool visualization_enabled_{false};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
  standard_image_publisher_;
  sensor_msgs::msg::Image standard_image_message_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
  camera_info_publisher_;
  rclcpp::TimerBase::SharedPtr bus_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
  parameter_callback_handle_;

  GstElement * pipeline_{nullptr};
  GstElement * sink_{nullptr};
  GstBus * bus_{nullptr};
  gulong new_sample_handler_{0};
  std::atomic_bool shutting_down_{false};
  std::atomic_bool pipeline_failed_{false};
  uint64_t sequence_{0};
  std::mutex callback_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<MipiCamera>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("mipi_camera"),
      "MIPI camera failed: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
