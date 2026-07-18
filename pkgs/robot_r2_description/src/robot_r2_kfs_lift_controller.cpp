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
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace robot_r2_description
{

class RobotR2KfsLiftController : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);

    command_topic_ = sdf->Get<std::string>(
      "command_topic", "/r2/kfs_lift/cmd").first;
    feedback_topic_ = sdf->Get<std::string>(
      "feedback_topic", "/r2/kfs_lift/feedback").first;
    joint_name_ = sdf->Get<std::string>(
      "joint_name", "kfs_lift_joint").first;

    min_position_ = sdf->Get<double>("min_position", 0.0).first;
    max_position_ = sdf->Get<double>("max_position", 0.42).first;
    if (min_position_ > max_position_) {
      std::swap(min_position_, max_position_);
    }

    const double sdf_p_gain =
      sdf->Get<double>("position_p_gain", 6000.0).first;
    const double sdf_i_gain =
      sdf->Get<double>("position_i_gain", 1000.0).first;
    const double sdf_d_gain =
      sdf->Get<double>("position_d_gain", 20.0).first;
    const double sdf_i_max =
      sdf->Get<double>("position_i_max", 1000.0).first;
    const double sdf_i_min =
      sdf->Get<double>("position_i_min", -1000.0).first;
    const double sdf_force_limit =
      sdf->Get<double>("max_actuation_force", 1000.0).first;

    node_->declare_parameter("kfs_lift.position_p_gain", sdf_p_gain);
    node_->declare_parameter("kfs_lift.position_i_gain", sdf_i_gain);
    node_->declare_parameter("kfs_lift.position_d_gain", sdf_d_gain);
    node_->declare_parameter("kfs_lift.position_i_max", sdf_i_max);
    node_->declare_parameter("kfs_lift.position_i_min", sdf_i_min);
    node_->declare_parameter(
      "kfs_lift.max_actuation_force", sdf_force_limit);
    LoadParameters();

    joint_ = model_->GetJoint(joint_name_);
    if (!joint_) {
      RCLCPP_ERROR(
        node_->get_logger(), "KFS lift joint '%s' was not found",
        joint_name_.c_str());
      return;
    }

    target_ = Clamp(joint_->Position(0), min_position_, max_position_);

    command_subscription_ =
      node_->create_subscription<std_msgs::msg::Float64>(
      command_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        if (!std::isfinite(msg->data)) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        target_ = Clamp(msg->data, min_position_, max_position_);
      });

    feedback_publisher_ =
      node_->create_publisher<std_msgs::msg::Float64>(
      feedback_topic_, rclcpp::QoS(10));

    parameter_callback_ = node_->add_on_set_parameters_callback(
      std::bind(
        &RobotR2KfsLiftController::OnParametersChanged, this,
        std::placeholders::_1));

    last_update_time_ = model_->GetWorld()->SimTime();
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RobotR2KfsLiftController::OnUpdate, this));
  }

private:
  void LoadParameters()
  {
    p_gain_ = node_->get_parameter("kfs_lift.position_p_gain").as_double();
    i_gain_ = node_->get_parameter("kfs_lift.position_i_gain").as_double();
    d_gain_ = node_->get_parameter("kfs_lift.position_d_gain").as_double();
    i_max_ = node_->get_parameter("kfs_lift.position_i_max").as_double();
    i_min_ = node_->get_parameter("kfs_lift.position_i_min").as_double();
    force_limit_ =
      node_->get_parameter("kfs_lift.max_actuation_force").as_double();
  }

  rcl_interfaces::msg::SetParametersResult OnParametersChanged(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "kfs_lift.position_p_gain") {
        p_gain_ = parameter.as_double();
      } else if (name == "kfs_lift.position_i_gain") {
        i_gain_ = parameter.as_double();
        if (i_gain_ <= 1e-9) {
          integral_ = 0.0;
        } else {
          integral_ = Clamp(integral_, i_min_, i_max_);
        }
      } else if (name == "kfs_lift.position_d_gain") {
        d_gain_ = parameter.as_double();
        reset_derivative_ = true;
      } else if (name == "kfs_lift.position_i_max") {
        i_max_ = parameter.as_double();
      } else if (name == "kfs_lift.position_i_min") {
        i_min_ = parameter.as_double();
      } else if (name == "kfs_lift.max_actuation_force") {
        force_limit_ = parameter.as_double();
      }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void OnUpdate()
  {
    double target;
    double p_gain;
    double i_gain;
    double d_gain;
    double i_max;
    double i_min;
    double force_limit;
    bool reset_derivative;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target = target_;
      p_gain = p_gain_;
      i_gain = i_gain_;
      d_gain = d_gain_;
      i_max = i_max_;
      i_min = i_min_;
      force_limit = force_limit_;
      reset_derivative = reset_derivative_;
      reset_derivative_ = false;
    }

    const auto now = model_->GetWorld()->SimTime();
    double dt = (now - last_update_time_).Double();
    if (dt <= 0.0 || dt > 1.0) {
      dt = 0.001;
    }
    last_update_time_ = now;

    const double position = joint_->Position(0);
    const double error = target - position;
    if (reset_derivative) {
      previous_error_ = error;
    }

    const double derivative =
      dt > 1e-6 ? (error - previous_error_) / dt : 0.0;
    const double p_term = p_gain * error;
    const double d_term = d_gain * derivative;

    double candidate_integral = integral_;
    if (i_gain > 1e-9) {
      candidate_integral = Clamp(
        integral_ + i_gain * error * dt, i_min, i_max);
    } else {
      candidate_integral = 0.0;
      integral_ = 0.0;
    }

    const double raw_force = p_term + candidate_integral + d_term;
    double force = Clamp(raw_force, -force_limit, force_limit);
    const bool saturated_high = raw_force > force_limit;
    const bool saturated_low = raw_force < -force_limit;
    const bool drives_further_into_saturation =
      (saturated_high && error > 0.0) ||
      (saturated_low && error < 0.0);
    if (i_gain > 1e-9 && !drives_further_into_saturation) {
      integral_ = candidate_integral;
      force = Clamp(
        p_term + integral_ + d_term, -force_limit, force_limit);
    }

    previous_error_ = error;
    joint_->SetForce(0, force);

    std_msgs::msg::Float64 feedback;
    feedback.data = position;
    feedback_publisher_->publish(feedback);
  }

  static double Clamp(double value, double lower, double upper)
  {
    return std::max(lower, std::min(value, upper));
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr joint_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_connection_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
    command_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr feedback_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_;

  std::mutex mutex_;
  std::string command_topic_{"/r2/kfs_lift/cmd"};
  std::string feedback_topic_{"/r2/kfs_lift/feedback"};
  std::string joint_name_{"kfs_lift_joint"};
  double min_position_{0.0};
  double max_position_{0.42};
  double p_gain_{6000.0};
  double i_gain_{1000.0};
  double d_gain_{20.0};
  double i_max_{1000.0};
  double i_min_{-1000.0};
  double force_limit_{1000.0};
  double target_{0.0};
  double integral_{0.0};
  double previous_error_{0.0};
  bool reset_derivative_{false};
  gazebo::common::Time last_update_time_{0};
};

GZ_REGISTER_MODEL_PLUGIN(RobotR2KfsLiftController)

}  // namespace robot_r2_description
