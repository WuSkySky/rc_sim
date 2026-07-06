#include <functional>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ignition/math/Pose3.hh>
#include <rclcpp/rclcpp.hpp>

namespace robot_r2_description
{

class RobotR2Localizer : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);

    pose_topic_ = sdf->Get<std::string>(
      "pose_topic", "/r2/pose_feedback").first;
    link_name_ = sdf->Get<std::string>("link_name", "base_link").first;
    pose_offset_ = sdf->Get<ignition::math::Pose3d>(
      "pose_offset", ignition::math::Pose3d::Zero).first;
    frame_id_ = sdf->Get<std::string>("frame_id", "world").first;
    publish_rate_ = sdf->Get<double>("publish_rate", 50.0).first;

    link_ = model_->GetLink(link_name_);
    if (!link_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Robot R2 localizer cannot find link [%s]",
        link_name_.c_str());
      return;
    }

    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      pose_topic_,
      rclcpp::QoS(10));

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RobotR2Localizer::OnUpdate, this));

    RCLCPP_INFO(
      node_->get_logger(),
      "Robot R2 localizer publishing [%s]",
      pose_topic_.c_str());
  }

private:
  void OnUpdate()
  {
    const auto now = model_->GetWorld()->SimTime();
    if (publish_rate_ > 0.0 && last_publish_time_.Double() > 0.0) {
      const double interval = 1.0 / publish_rate_;
      if ((now - last_publish_time_).Double() < interval) {
        return;
      }
    }
    last_publish_time_ = now;

    const auto pose = link_->WorldPose() * pose_offset_;

    geometry_msgs::msg::PoseStamped msg;
    const auto stamp_ns = node_->get_clock()->now().nanoseconds();
    msg.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    msg.header.frame_id = frame_id_;
    msg.pose.position.x = pose.Pos().X();
    msg.pose.position.y = pose.Pos().Y();
    msg.pose.position.z = pose.Pos().Z();
    msg.pose.orientation.x = pose.Rot().X();
    msg.pose.orientation.y = pose.Rot().Y();
    msg.pose.orientation.z = pose.Rot().Z();
    msg.pose.orientation.w = pose.Rot().W();
    pose_pub_->publish(msg);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_connection_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  gazebo::common::Time last_publish_time_{0};
  std::string pose_topic_{"/r2/pose_feedback"};
  std::string link_name_{"base_link"};
  ignition::math::Pose3d pose_offset_{ignition::math::Pose3d::Zero};
  std::string frame_id_{"world"};
  double publish_rate_{50.0};
};

GZ_REGISTER_MODEL_PLUGIN(RobotR2Localizer)

}  // namespace robot_r2_description
