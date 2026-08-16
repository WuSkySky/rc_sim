#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cctype>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
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

constexpr char kImageTopic[] = "/r2/yahboom_camera/image_raw";
constexpr char kDebugTopic[] = "/r2/yahboom_camera/image_raw/debug";
constexpr char kCameraInfoTopic[] = "/r2/yahboom_camera/camera_info";

std::string mode_to_string(const CameraMode & mode)
{
  std::ostringstream stream;
  stream << '[' << mode[0] << ", " << mode[1] << ", " << mode[2] << ']';
  return stream.str();
}

}  // namespace

class YahboomCamera : public rclcpp::Node
{
public:
  YahboomCamera()
  : Node("yahboom_camera")
  {
    device_ = declare_parameter<std::string>("device", "/dev/video2");
    pixel_format_ = normalize_pixel_format(
      declare_parameter<std::string>("pixel_format", "MJPG"));
    const auto mode_values = declare_parameter<std::vector<int64_t>>(
      "mode", {1920, 1080, 30});
    frame_id_ = frame_id_for_node(get_name());
    visualization_enabled_.store(
      declare_parameter<bool>("visualization_enabled", false));
    focal_length_mm_.store(declare_parameter<double>("focal_length_mm", 0.0));
    pixel_size_um_.store(declare_parameter<double>("pixel_size_um", 0.0));
    k1_.store(declare_parameter<double>("k1", 0.0));
    k2_.store(declare_parameter<double>("k2", 0.0));
    p1_.store(declare_parameter<double>("p1", 0.0));
    p2_.store(declare_parameter<double>("p2", 0.0));
    k3_.store(declare_parameter<double>("k3", 0.0));
    validate_intrinsic_parameters(focal_length_mm_.load(),
                                  pixel_size_um_.load(), k1_.load(),
                                  k2_.load(), p1_.load(), p2_.load(),
                                  k3_.load());

    const auto mode = validate_mode(mode_values);
    width_ = static_cast<int>(mode[0]);
    height_ = static_cast<int>(mode[1]);
    framerate_ = static_cast<int>(mode[2]);

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

    const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability_volatile();
    image_publisher_ = create_publisher<CameraFrame>(
      kImageTopic, image_qos);
    image_message_.frame_id_size = static_cast<uint8_t>(frame_id_.size());
    std::fill(
      image_message_.frame_id.begin(), image_message_.frame_id.end(), 0U);
    std::memcpy(
      image_message_.frame_id.data(), frame_id_.data(), frame_id_.size());
    image_message_.encoding = CameraFrame::ENCODING_BGR8;
    image_message_.is_bigendian = 0U;
    image_message_.layout_version = CameraFrame::LAYOUT_VERSION;
    image_message_.data.reserve(CameraFrame::DATA_CAPACITY);
    debug_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      kDebugTopic, image_qos);
    debug_message_.header.frame_id = frame_id_;
    debug_message_.encoding = "bgr8";
    debug_message_.is_bigendian = 0U;
    debug_message_.data.reserve(CameraFrame::DATA_CAPACITY);
    camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      kCameraInfoTopic, rclcpp::QoS(10));
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(
        &YahboomCamera::on_parameters_changed, this,
        std::placeholders::_1));

    start_pipeline();
    bus_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&YahboomCamera::check_bus, this));

    RCLCPP_INFO(
      get_logger(),
      "Capturing %s (%s) at %dx%d@%d FPS; publishing bounded CameraFrame "
      "samples on %s",
      device_.c_str(), pixel_format_.c_str(), width_, height_, framerate_,
      kImageTopic);
  }

  ~YahboomCamera() override
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
    if (node_name == "front_yahboom_camera") {
      return "r2_front_camera_optical_frame";
    }
    return "r2_yahboom_camera_optical_frame";
  }

  static std::string normalize_pixel_format(std::string value)
  {
    std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char character) {
        return static_cast<char>(std::toupper(character));
      });
    if (value != "MJPG" && value != "YUYV") {
      throw std::invalid_argument(
              "pixel_format must be MJPG or YUYV, got '" + value + "'");
    }
    return value;
  }

  static CameraMode validate_mode(const std::vector<int64_t> & values)
  {
    if (values.size() != 3) {
      throw std::invalid_argument(
              "mode must contain [width, height, framerate]");
    }

    const CameraMode mode{values[0], values[1], values[2]};
    if (mode[0] <= 0 || mode[1] <= 0 || mode[2] <= 0) {
      throw std::invalid_argument(
              "mode values must be positive: " + mode_to_string(mode));
    }
    const auto frame_bytes =
      static_cast<int64_t>(mode[0]) * static_cast<int64_t>(mode[1]) * 3;
    if (frame_bytes > static_cast<int64_t>(CameraFrame::DATA_CAPACITY)) {
      throw std::invalid_argument(
              "mode " + mode_to_string(mode) +
              " exceeds CameraFrame capacity of " +
              std::to_string(CameraFrame::DATA_CAPACITY) + " bytes");
    }
    return mode;
  }

  static void validate_intrinsic_parameters(double focal_length_mm,
                                            double pixel_size_um, double k1,
                                            double k2, double p1, double p2,
                                            double k3)
  {
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

  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    const bool previous_visualization = visualization_enabled_.load();
    bool next_visualization = previous_visualization;
    double next_focal_length_mm = focal_length_mm_.load();
    double next_pixel_size_um = pixel_size_um_.load();
    double next_k1 = k1_.load();
    double next_k2 = k2_.load();
    double next_p1 = p1_.load();
    double next_p2 = p2_.load();
    double next_k3 = k3_.load();

    try {
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name == "visualization_enabled") {
          next_visualization = parameter.as_bool();
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
      validate_intrinsic_parameters(next_focal_length_mm, next_pixel_size_um,
                                    next_k1, next_k2, next_p1, next_p2,
                                    next_k3);
    } catch (const std::exception & error) {
      result.reason = error.what();
      return result;
    }

    visualization_enabled_.store(next_visualization);
    focal_length_mm_.store(next_focal_length_mm);
    pixel_size_um_.store(next_pixel_size_um);
    k1_.store(next_k1);
    k2_.store(next_k2);
    p1_.store(next_p1);
    p2_.store(next_p2);
    k3_.store(next_k3);
    result.successful = true;
    if (next_visualization != previous_visualization) {
      RCLCPP_INFO(
        get_logger(), "Camera debug image publication %s",
        next_visualization ? "enabled" : "disabled");
    }
    return result;
  }

  std::string make_pipeline_description() const
  {
    std::ostringstream pipeline;
    pipeline << "v4l2src device=" << device_;
    if (pixel_format_ == "MJPG") {
      pipeline << " ! image/jpeg,width=(int)" << width_
               << ",height=(int)" << height_
               << ",framerate=(fraction)" << framerate_ << "/1"
               << " ! jpegdec";
    } else {
      pipeline << " ! video/x-raw,format=(string)YUY2,width=(int)" << width_
               << ",height=(int)" << height_
               << ",framerate=(fraction)" << framerate_ << "/1";
    }
    pipeline << " ! videoconvert ! video/x-raw,format=(string)BGR"
             << " ! appsink name=camera_sink emit-signals=true "
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
    return static_cast<YahboomCamera *>(user_data)->publish_sample(sink);
  }

  void fill_intrinsics(sensor_msgs::msg::CameraInfo & info, double width,
                       double height) const
  {
    const double focal_length_mm = focal_length_mm_.load();
    const double pixel_size_um = pixel_size_um_.load();

    double focal_length_px = std::max(width, height);
    if (focal_length_mm > 0.0 && pixel_size_um > 0.0) {
      focal_length_px = focal_length_mm * 1000.0 / pixel_size_um;
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
        auto & debug = debug_message_;
        debug.header.stamp = stamp;
        debug.height = image.height;
        debug.width = image.width;
        debug.step = image.step;
        debug.data.resize(data_size);
        std::memcpy(debug.data.data(), &image.data[0], data_size);
        debug_publisher_->publish(debug);
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
    fill_intrinsics(camera_info, static_cast<double>(width),
                    static_cast<double>(height));

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
  std::string pixel_format_;
  std::string frame_id_;
  int width_{0};
  int height_{0};
  int framerate_{0};
  std::atomic<double> focal_length_mm_{0.0};
  std::atomic<double> pixel_size_um_{0.0};
  std::atomic<double> k1_{0.0};
  std::atomic<double> k2_{0.0};
  std::atomic<double> p1_{0.0};
  std::atomic<double> p2_{0.0};
  std::atomic<double> k3_{0.0};

  rclcpp::Publisher<CameraFrame>::SharedPtr image_publisher_;
  CameraFrame image_message_;
  std::atomic_bool visualization_enabled_{false};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
  sensor_msgs::msg::Image debug_message_;
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
    rclcpp::spin(std::make_shared<YahboomCamera>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("yahboom_camera"),
      "Yahboom camera failed: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
