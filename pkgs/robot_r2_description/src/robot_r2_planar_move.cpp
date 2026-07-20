#include <algorithm>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>

namespace robot_r2_description
{

class RobotR2PlanarMove : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);

    command_topic_ = sdf->Get<std::string>(
      "command_topic", "/r2/cmd_vel").first;
    velocity_feedback_topic_ = sdf->Get<std::string>(
      "velocity_feedback_topic", "/r2/velocity_feedback").first;
    cmd_vel_timeout_ = sdf->Get<double>("cmd_vel_timeout", 0.25).first;

    x_velocity_p_gain_ = sdf->Get<double>("x_velocity_p_gain", 15.0).first;
    y_velocity_p_gain_ = sdf->Get<double>("y_velocity_p_gain", 15.0).first;
    x_velocity_i_gain_ = sdf->Get<double>("x_velocity_i_gain", 0.0).first;
    y_velocity_i_gain_ = sdf->Get<double>("y_velocity_i_gain", 0.0).first;
    max_integral_force_ = sdf->Get<double>("max_integral_force", 0.0).first;
    yaw_velocity_p_gain_ = sdf->Get<double>("yaw_velocity_p_gain", 1.0).first;
    max_x_velocity_ = sdf->Get<double>("max_x_velocity", 0.6).first;
    max_y_velocity_ = sdf->Get<double>("max_y_velocity", 0.6).first;
    max_yaw_velocity_ = sdf->Get<double>("max_yaw_velocity", 0.5).first;

    base_link_ = model_->GetLink("base_link");
    if (!base_link_) {
      return;
    }

    model_links_ = model_->GetLinks();

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      command_topic_,
      rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        target_cmd_ = *msg;
        last_cmd_time_ = model_->GetWorld()->SimTime();
        has_cmd_ = true;
      });

    velocity_feedback_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      velocity_feedback_topic_,
      rclcpp::QoS(10));

    update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&RobotR2PlanarMove::OnUpdate, this));

  }

private:
  void OnUpdate()
  {
    geometry_msgs::msg::Twist cmd;
    auto now = model_->GetWorld()->SimTime();
    const double dt = last_update_time_.Double() > 0.0 ?
      (now - last_update_time_).Double() : 0.0;
    last_update_time_ = now;
    bool command_active = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_cmd_ && (now - last_cmd_time_).Double() <= cmd_vel_timeout_) {
        cmd = target_cmd_;
        command_active = true;
      }
    }

    double tx = Clamp(cmd.linear.x, -max_x_velocity_, max_x_velocity_);
    double ty = Clamp(cmd.linear.y, -max_y_velocity_, max_y_velocity_);
    double tyaw = Clamp(cmd.angular.z, -max_yaw_velocity_, max_yaw_velocity_);

    auto v = model_->RelativeLinearVel();
    auto w = model_->WorldAngularVel();
    const double error_x = tx - v.X();
    const double error_y = ty - v.Y();

    const bool moving_command =
      std::abs(tx) > 1e-4 || std::abs(ty) > 1e-4 || std::abs(tyaw) > 1e-4;

    if (command_active && moving_command && dt > 0.0) {
      integral_x_ += error_x * dt;
      integral_y_ += error_y * dt;
    } else {
      integral_x_ = 0.0;
      integral_y_ = 0.0;
    }

    const double integral_force_x = Clamp(
      integral_x_ * x_velocity_i_gain_,
      -max_integral_force_,
      max_integral_force_);
    const double integral_force_y = Clamp(
      integral_y_ * y_velocity_i_gain_,
      -max_integral_force_,
      max_integral_force_);

    ignition::math::Vector3d force(
      error_x * x_velocity_p_gain_ + integral_force_x,
      error_y * y_velocity_p_gain_ + integral_force_y,
      0.0
    );

    ignition::math::Vector3d torque(
      0.0,
      0.0,
      (tyaw - w.Z()) * yaw_velocity_p_gain_
    );

    const auto world_force =
      base_link_->WorldPose().Rot().RotateVector(force);
    base_link_->AddForceAtWorldPosition(
      world_force, CalculateModelCenterOfMass());
    base_link_->AddTorque(torque);

    geometry_msgs::msg::Twist feedback;
    feedback.linear.x = v.X();
    feedback.linear.y = v.Y();
    feedback.linear.z = v.Z();
    feedback.angular.z = w.Z();
    velocity_feedback_pub_->publish(feedback);
  }

  static double Clamp(double v, double mn, double mx)
  {
    return std::max(mn, std::min(v, mx));
  }

  ignition::math::Vector3d CalculateModelCenterOfMass() const
  {
    ignition::math::Vector3d weighted_position =
      ignition::math::Vector3d::Zero;
    double total_mass = 0.0;

    for (const auto & link : model_links_) {
      if (!link || !link->GetInertial()) {
        continue;
      }

      const double mass = link->GetInertial()->Mass();
      if (mass <= 0.0) {
        continue;
      }

      weighted_position += link->WorldCoGPose().Pos() * mass;
      total_mass += mass;
    }

    if (total_mass <= 0.0) {
      return base_link_->WorldCoGPose().Pos();
    }
    return weighted_position / total_mass;
  }

private:
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_link_;
  std::vector<gazebo::physics::LinkPtr> model_links_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_connection_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_feedback_pub_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist target_cmd_;
  gazebo::common::Time last_cmd_time_{0};
  gazebo::common::Time last_update_time_{0};
  bool has_cmd_{false};

  std::string command_topic_{"/r2/cmd_vel"};
  std::string velocity_feedback_topic_{"/r2/velocity_feedback"};
  double cmd_vel_timeout_{0.25};

  double max_x_velocity_{0.6};
  double max_y_velocity_{0.6};
  double max_yaw_velocity_{0.5};

  double x_velocity_p_gain_{15.0};
  double y_velocity_p_gain_{15.0};
  double x_velocity_i_gain_{0.0};
  double y_velocity_i_gain_{0.0};
  double max_integral_force_{0.0};
  double yaw_velocity_p_gain_{1.0};
  double integral_x_{0.0};
  double integral_y_{0.0};
};

GZ_REGISTER_MODEL_PLUGIN(RobotR2PlanarMove)

} // namespace robot_r2_description
