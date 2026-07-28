#include <gazebo/common/Plugin.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo_ros/node.hpp>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_r2_interfaces/msg/camera_frame.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace robot_r2_description
{

using CameraFrame = robot_r2_interfaces::msg::CameraFrame;

class RobotR2Camera : public gazebo::CameraPlugin
{
public:
  void Load(
    gazebo::sensors::SensorPtr sensor,
    sdf::ElementPtr sdf) override
  {
    sensor_ = std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(sensor);
    if (!sensor_) {
      throw std::runtime_error(
              "RobotR2Camera must be attached to a camera sensor");
    }

    node_ = gazebo_ros::Node::Get(sdf);
    image_topic_ = sdf->Get<std::string>(
      "image_topic", "front_camera/image_raw").first;
    debug_topic_ = sdf->Get<std::string>(
      "debug_image_topic", "front_camera/image_raw/debug").first;
    camera_info_topic_ = sdf->Get<std::string>(
      "camera_info_topic", "front_camera/camera_info").first;
    frame_id_ = sdf->Get<std::string>("frame_name", "base_link").first;
    const bool visualization_default = sdf->Get<bool>(
      "visualization_enabled", false).first;
    visualization_enabled_.store(
      node_->declare_parameter<bool>(
        "visualization_enabled", visualization_default));

    if (image_topic_.empty() || debug_topic_.empty() ||
      camera_info_topic_.empty())
    {
      throw std::invalid_argument(
              "image_topic, debug_image_topic, and camera_info_topic "
              "must not be empty");
    }
    if (frame_id_.size() > CameraFrame::FRAME_ID_CAPACITY) {
      throw std::invalid_argument(
              "frame_name exceeds CameraFrame frame_id capacity");
    }

    const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability_volatile();
    image_publisher_ = node_->create_publisher<CameraFrame>(
      image_topic_, image_qos);
    camera_info_publisher_ =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::QoS(10));
    standard_image_publisher_ =
      node_->create_publisher<sensor_msgs::msg::Image>(
      debug_topic_, image_qos);
    standard_image_.header.frame_id = frame_id_;
    standard_image_.data.reserve(CameraFrame::DATA_CAPACITY);
    parameter_callback_handle_ =
      node_->add_on_set_parameters_callback(
      std::bind(
        &RobotR2Camera::on_parameters_changed, this,
        std::placeholders::_1));

    frame_.frame_id_size = static_cast<uint8_t>(frame_id_.size());
    std::memcpy(
      frame_.frame_id.data(), frame_id_.data(), frame_id_.size());
    frame_.layout_version = CameraFrame::LAYOUT_VERSION;
    frame_.is_bigendian = 0U;
    frame_.data.reserve(CameraFrame::DATA_CAPACITY);
    gazebo::CameraPlugin::Load(sensor, sdf);
    sensor_->SetActive(true);

    RCLCPP_INFO(
      node_->get_logger(),
      "Gazebo camera publishing CameraFrame on %s; debug images on %s "
      "are initially %s",
      image_topic_.c_str(), debug_topic_.c_str(),
      visualization_enabled_.load() ? "enabled" : "disabled");
  }

  void OnNewFrame(
    const unsigned char * image,
    unsigned int width,
    unsigned int height,
    unsigned int depth,
    const std::string & format) override
  {
    uint8_t encoding = 0U;
    unsigned int channels = 0U;
    std::string standard_encoding;
    if (format == "R8G8B8" && depth == 3U) {
      encoding = CameraFrame::ENCODING_RGB8;
      channels = 3U;
      standard_encoding = "rgb8";
    } else if (format == "B8G8R8" && depth == 3U) {
      encoding = CameraFrame::ENCODING_BGR8;
      channels = 3U;
      standard_encoding = "bgr8";
    } else if (format == "L8" && depth == 1U) {
      encoding = CameraFrame::ENCODING_MONO8;
      channels = 1U;
      standard_encoding = "mono8";
    } else {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Unsupported Gazebo camera format '%s' with depth %u",
        format.c_str(), depth);
      return;
    }

    const std::size_t step =
      static_cast<std::size_t>(width) * channels;
    const std::size_t data_size =
      step * static_cast<std::size_t>(height);
    if (width == 0U || height == 0U ||
      data_size > CameraFrame::DATA_CAPACITY)
    {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Invalid Gazebo camera frame %ux%u (%zu bytes)",
        width, height, data_size);
      return;
    }

    const auto stamp = node_->now();
    const auto stamp_ns = stamp.nanoseconds();
    frame_.sequence = sequence_;
    frame_.stamp_sec =
      static_cast<int32_t>(stamp_ns / 1000000000LL);
    frame_.stamp_nanosec =
      static_cast<uint32_t>(stamp_ns % 1000000000LL);
    frame_.width = width;
    frame_.height = height;
    frame_.step = static_cast<uint32_t>(step);
    frame_.data_size = static_cast<uint32_t>(data_size);
    frame_.encoding = encoding;
    frame_.data.resize(data_size);
    std::memcpy(&frame_.data[0], image, data_size);
    image_publisher_->publish(frame_);

    if (visualization_enabled_.load()) {
      standard_image_.header.stamp = stamp;
      standard_image_.height = height;
      standard_image_.width = width;
      standard_image_.encoding = standard_encoding;
      standard_image_.is_bigendian = 0U;
      standard_image_.step = static_cast<uint32_t>(step);
      standard_image_.data.resize(data_size);
      std::memcpy(standard_image_.data.data(), image, data_size);
      standard_image_publisher_->publish(standard_image_);
    }

    publish_camera_info(stamp, width, height);
    ++sequence_;
  }

private:
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
        node_->get_logger(),
        "Gazebo camera debug image publication %s",
        parameter.as_bool() ? "enabled" : "disabled");
    }
    return result;
  }

  void publish_camera_info(
    const rclcpp::Time & stamp,
    uint32_t width,
    uint32_t height)
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id_;
    info.width = width;
    info.height = height;
    info.distortion_model = "plumb_bob";
    info.d.assign(5U, 0.0);
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0};

    const double horizontal_fov =
      sensor_->Camera()->HFOV().Radian();
    const double focal =
      static_cast<double>(width) /
      (2.0 * std::tan(horizontal_fov / 2.0));
    const double center_x =
      (static_cast<double>(width) - 1.0) / 2.0;
    const double center_y =
      (static_cast<double>(height) - 1.0) / 2.0;
    info.k = {
      focal, 0.0, center_x,
      0.0, focal, center_y,
      0.0, 0.0, 1.0};
    info.p = {
      focal, 0.0, center_x, 0.0,
      0.0, focal, center_y, 0.0,
      0.0, 0.0, 1.0, 0.0};
    camera_info_publisher_->publish(info);
  }

  gazebo::sensors::CameraSensorPtr sensor_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<CameraFrame>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
  standard_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
  camera_info_publisher_;
  CameraFrame frame_;
  sensor_msgs::msg::Image standard_image_;
  std::string image_topic_;
  std::string debug_topic_;
  std::string camera_info_topic_;
  std::string frame_id_;
  std::atomic_bool visualization_enabled_{false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
  parameter_callback_handle_;
  uint64_t sequence_{0U};
};

GZ_REGISTER_SENSOR_PLUGIN(RobotR2Camera)

}  // namespace robot_r2_description
