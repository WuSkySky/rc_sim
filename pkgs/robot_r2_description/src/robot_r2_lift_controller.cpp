#include <algorithm>
#include <functional>
#include <mutex>
#include <string>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_r2_interfaces/msg/lift_command.hpp>
#include <robot_r2_interfaces/msg/lift_feedback.hpp>

namespace robot_r2_description
{

class RobotR2LiftController : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);

    command_topic_ = sdf->Get<std::string>(
      "command_topic", "/simulation/r2/lift/cmd_lift").first;
    position_feedback_topic_ = sdf->Get<std::string>(
      "position_feedback_topic", "/simulation/r2/lift/position_feedback").first;
    front_joint_name_ = sdf->Get<std::string>(
      "front_joint_name", "front_lift_joint").first;
    rear_joint_name_ = sdf->Get<std::string>(
      "rear_joint_name", "rear_lift_joint").first;
    min_lift_ = sdf->Get<double>("min_lift", -0.3).first;
    max_lift_ = sdf->Get<double>("max_lift", 0.3).first;
    position_p_gain_ = sdf->Get<double>("position_p_gain", 300.0).first;
    position_i_gain_ = sdf->Get<double>("position_i_gain", 0.0).first;
    position_d_gain_ = sdf->Get<double>("position_d_gain", 80.0).first;
    position_i_max_ = sdf->Get<double>("position_i_max", 0.0).first;
    position_i_min_ = sdf->Get<double>("position_i_min", 0.0).first;
    max_actuation_force_ = sdf->Get<double>("max_actuation_force", 400.0).first;

    if (min_lift_ > max_lift_) {
      std::swap(min_lift_, max_lift_);
    }

    front_joint_ = model_->GetJoint(front_joint_name_);
    rear_joint_ = model_->GetJoint(rear_joint_name_);
    if (!front_joint_ || !rear_joint_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Cannot find lift joints: front=%s rear=%s",
        front_joint_name_.c_str(),
        rear_joint_name_.c_str());
      return;
    }

    joint_controller_ = model_->GetJointController();
    if (!joint_controller_) {
      RCLCPP_ERROR(node_->get_logger(), "Cannot get joint controller");
      return;
    }

    const std::string front_scoped_name = front_joint_->GetScopedName();
    const std::string rear_scoped_name = rear_joint_->GetScopedName();
    gazebo::common::PID position_pid(
      position_p_gain_,
      position_i_gain_,
      position_d_gain_,
      position_i_max_,
      position_i_min_,
      max_actuation_force_,
      -max_actuation_force_);

    joint_controller_->SetPositionPID(front_scoped_name, position_pid);
    joint_controller_->SetPositionPID(rear_scoped_name, position_pid);

    front_target_ = 0.0;
    rear_target_ = 0.0;
    joint_controller_->SetJointPosition(front_scoped_name, front_target_);
    joint_controller_->SetJointPosition(rear_scoped_name, rear_target_);
    joint_controller_->SetPositionTarget(front_scoped_name, front_target_);
    joint_controller_->SetPositionTarget(rear_scoped_name, rear_target_);

    command_sub_ =
      node_->create_subscription<robot_r2_interfaces::msg::LiftCommand>(
      command_topic_,
      rclcpp::QoS(10),
      [this](robot_r2_interfaces::msg::LiftCommand::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        front_target_ = Clamp(msg->front_lift, min_lift_, max_lift_);
        rear_target_ = Clamp(msg->rear_lift, min_lift_, max_lift_);
      });

    position_feedback_pub_ =
      node_->create_publisher<robot_r2_interfaces::msg::LiftFeedback>(
      position_feedback_topic_,
      rclcpp::QoS(10));

    update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RobotR2LiftController::OnUpdate, this));

    RCLCPP_INFO(
      node_->get_logger(),
      "Robot R2 lift controller started on %s",
      command_topic_.c_str());
  }

private:
  void OnUpdate()
  {
    double front_target = 0.0;
    double rear_target = 0.0;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      front_target = front_target_;
      rear_target = rear_target_;
    }

    joint_controller_->SetPositionTarget(
      front_joint_->GetScopedName(), Clamp(front_target, min_lift_, max_lift_));
    joint_controller_->SetPositionTarget(
      rear_joint_->GetScopedName(), Clamp(rear_target, min_lift_, max_lift_));
    joint_controller_->Update();

    robot_r2_interfaces::msg::LiftFeedback feedback;
    feedback.front_lift = front_joint_->Position(0);
    feedback.rear_lift = rear_joint_->Position(0);
    position_feedback_pub_->publish(feedback);
  }

  static double Clamp(double value, double minimum, double maximum)
  {
    return std::max(minimum, std::min(value, maximum));
  }

private:
  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr front_joint_;
  gazebo::physics::JointPtr rear_joint_;
  gazebo::physics::JointControllerPtr joint_controller_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_connection_;
  rclcpp::Subscription<robot_r2_interfaces::msg::LiftCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<robot_r2_interfaces::msg::LiftFeedback>::SharedPtr position_feedback_pub_;

  std::mutex mutex_;
  std::string command_topic_{"/simulation/r2/lift/cmd_lift"};
  std::string position_feedback_topic_{"/simulation/r2/lift/position_feedback"};
  std::string front_joint_name_{"front_lift_joint"};
  std::string rear_joint_name_{"rear_lift_joint"};
  double min_lift_{-0.3};
  double max_lift_{0.3};
  double position_p_gain_{300.0};
  double position_i_gain_{0.0};
  double position_d_gain_{80.0};
  double position_i_max_{0.0};
  double position_i_min_{0.0};
  double max_actuation_force_{400.0};
  double front_target_{0.0};
  double rear_target_{0.0};
};

GZ_REGISTER_MODEL_PLUGIN(RobotR2LiftController)

}  // namespace robot_r2_description
