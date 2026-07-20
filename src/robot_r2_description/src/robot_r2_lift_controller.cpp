#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
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

    // --- topics and joint names: read from SDF only (structural) ---

    command_topic_ = sdf->Get<std::string>(
      "command_topic", "/r2/lift/cmd_lift").first;
    position_feedback_topic_ = sdf->Get<std::string>(
      "position_feedback_topic", "/r2/lift/position_feedback").first;

    joint_names_[0] = sdf->Get<std::string>(
      "drive_fl_joint_name", "drive_lift_fl_joint").first;
    joint_names_[1] = sdf->Get<std::string>(
      "drive_fr_joint_name", "drive_lift_fr_joint").first;
    joint_names_[2] = sdf->Get<std::string>(
      "drive_rl_joint_name", "drive_lift_rl_joint").first;
    joint_names_[3] = sdf->Get<std::string>(
      "drive_rr_joint_name", "drive_lift_rr_joint").first;

    // --- hardware limits: SDF only ---

    min_lift_ = sdf->Get<double>("min_lift", 0.0).first;
    max_lift_ = sdf->Get<double>("max_lift", 0.376).first;
    if (min_lift_ > max_lift_) {
      std::swap(min_lift_, max_lift_);
    }

    // --- PID parameters: SDF default, overridable via ROS params ---

    const double sdf_p_gain = sdf->Get<double>("position_p_gain", 6000.0).first;
    const double sdf_i_gain = sdf->Get<double>("position_i_gain", 1000.0).first;
    const double sdf_d_gain = sdf->Get<double>("position_d_gain", 20.0).first;
    const double sdf_i_max  = sdf->Get<double>("position_i_max", 1000.0).first;
    const double sdf_i_min  = sdf->Get<double>("position_i_min", -1000.0).first;
    const double sdf_force  = sdf->Get<double>("max_actuation_force", 10000.0).first;

    node_->declare_parameter("lift.position_p_gain", sdf_p_gain);
    node_->declare_parameter("lift.position_i_gain", sdf_i_gain);
    node_->declare_parameter("lift.position_d_gain", sdf_d_gain);
    node_->declare_parameter("lift.position_i_max",  sdf_i_max);
    node_->declare_parameter("lift.position_i_min",  sdf_i_min);
    node_->declare_parameter("lift.max_actuation_force", sdf_force);

    LoadPidParamsFromNode();

    // --- lookup joints ---

    for (int i = 0; i < 4; ++i) {
      joints_[i] = model_->GetJoint(joint_names_[i]);
      if (!joints_[i]) {
        return;
      }
    }

    for (int i = 0; i < 4; ++i) {
      integral_term_[i] = 0.0;
      prev_error_[i] = 0.0;
    }

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

    parameter_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&RobotR2LiftController::OnParametersChanged, this, std::placeholders::_1));

    last_update_time_ = model_->GetWorld()->SimTime();

    update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RobotR2LiftController::OnUpdate, this));

  }

private:
  void LoadPidParamsFromNode()
  {
    position_p_gain_     = node_->get_parameter("lift.position_p_gain").as_double();
    position_i_gain_     = node_->get_parameter("lift.position_i_gain").as_double();
    position_d_gain_     = node_->get_parameter("lift.position_d_gain").as_double();
    position_i_max_      = node_->get_parameter("lift.position_i_max").as_double();
    position_i_min_      = node_->get_parameter("lift.position_i_min").as_double();
    max_actuation_force_ = node_->get_parameter("lift.max_actuation_force").as_double();
  }

  rcl_interfaces::msg::SetParametersResult OnParametersChanged(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    double next_p_gain = position_p_gain_;
    double next_i_gain = position_i_gain_;
    double next_d_gain = position_d_gain_;
    double next_i_max = position_i_max_;
    double next_i_min = position_i_min_;
    double next_force = max_actuation_force_;
    bool pid_changed = false;
    bool i_gain_changed = false;

    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();

      auto read_double = [&parameter, &name](double & output) -> bool
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          return false;
        }
        output = parameter.as_double();
        return std::isfinite(output);
      };

      if (name == "lift.position_p_gain") {
        pid_changed = true;
        if (!read_double(next_p_gain) || next_p_gain < 0.0) {
          return FailureResult("lift.position_p_gain must be a finite non-negative double");
        }
      } else if (name == "lift.position_i_gain") {
        pid_changed = true;
        i_gain_changed = true;
        if (!read_double(next_i_gain) || next_i_gain < 0.0) {
          return FailureResult("lift.position_i_gain must be a finite non-negative double");
        }
      } else if (name == "lift.position_d_gain") {
        pid_changed = true;
        if (!read_double(next_d_gain) || next_d_gain < 0.0) {
          return FailureResult("lift.position_d_gain must be a finite non-negative double");
        }
      } else if (name == "lift.position_i_max") {
        pid_changed = true;
        if (!read_double(next_i_max)) {
          return FailureResult("lift.position_i_max must be a finite double");
        }
      } else if (name == "lift.position_i_min") {
        pid_changed = true;
        if (!read_double(next_i_min)) {
          return FailureResult("lift.position_i_min must be a finite double");
        }
      } else if (name == "lift.max_actuation_force") {
        pid_changed = true;
        if (!read_double(next_force) || next_force <= 0.0) {
          return FailureResult("lift.max_actuation_force must be a finite positive double");
        }
      }
    }

    if (next_i_min > next_i_max) {
      return FailureResult("lift.position_i_min must be <= lift.position_i_max");
    }

    position_p_gain_ = next_p_gain;
    position_i_gain_ = next_i_gain;
    position_d_gain_ = next_d_gain;
    position_i_max_ = next_i_max;
    position_i_min_ = next_i_min;
    max_actuation_force_ = next_force;

    if (pid_changed) {
      for (int i = 0; i < 4; ++i) {
        if (i_gain_changed && next_i_gain <= 1e-9) {
          integral_term_[i] = 0.0;
        } else {
          integral_term_[i] = Clamp(integral_term_[i], position_i_min_, position_i_max_);
        }
      }
      derivative_state_reset_requested_ = true;
    }

    return SuccessResult();
  }

  void OnUpdate()
  {
    double front_target = 0.0;
    double rear_target = 0.0;
    double p_gain = 0.0;
    double i_gain = 0.0;
    double d_gain = 0.0;
    double i_max = 0.0;
    double i_min = 0.0;
    double force_limit = 0.0;
    bool reset_derivative_state = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      front_target = front_target_;
      rear_target = rear_target_;
      p_gain = position_p_gain_;
      i_gain = position_i_gain_;
      d_gain = position_d_gain_;
      i_max = position_i_max_;
      i_min = position_i_min_;
      force_limit = max_actuation_force_;
      reset_derivative_state = derivative_state_reset_requested_;
      derivative_state_reset_requested_ = false;
    }

    auto now = model_->GetWorld()->SimTime();
    double dt = (now - last_update_time_).Double();
    if (dt <= 0.0 || dt > 1.0) {
      dt = 0.001;
    }
    last_update_time_ = now;

    const std::array<double, 4> targets = {
      front_target, front_target, rear_target, rear_target};

    if (reset_derivative_state) {
      for (int i = 0; i < 4; ++i) {
        prev_error_[i] = targets[i] - joints_[i]->Position(0);
      }
    }

    for (int i = 0; i < 4; ++i) {
      const double position = joints_[i]->Position(0);
      const double error = targets[i] - position;

      double derivative = 0.0;
      if (dt > 1e-6) {
        derivative = (error - prev_error_[i]) / dt;
      }
      const double p_term = error * p_gain;
      const double d_term = derivative * d_gain;

      double candidate_integral_term = integral_term_[i];
      if (i_gain > 1e-9) {
        candidate_integral_term = Clamp(
          integral_term_[i] + i_gain * error * dt,
          i_min,
          i_max);
      } else {
        candidate_integral_term = 0.0;
        integral_term_[i] = 0.0;
      }

      double unclamped_force = p_term + candidate_integral_term + d_term;
      double force = Clamp(unclamped_force, -force_limit, force_limit);

      const bool saturated_high = unclamped_force > force_limit;
      const bool saturated_low = unclamped_force < -force_limit;
      const bool drives_further_into_saturation =
        (saturated_high && error > 0.0) || (saturated_low && error < 0.0);

      if (i_gain > 1e-9) {
        if (!drives_further_into_saturation) {
          integral_term_[i] = candidate_integral_term;
          force = Clamp(p_term + integral_term_[i] + d_term, -force_limit, force_limit);
        }
      }

      prev_error_[i] = error;
      joints_[i]->SetForce(0, force);
    }

    // --- publish feedback ---
    robot_r2_interfaces::msg::LiftFeedback feedback;
    feedback.front_left_lift  = joints_[0]->Position(0);
    feedback.front_right_lift = joints_[1]->Position(0);
    feedback.rear_left_lift   = joints_[2]->Position(0);
    feedback.rear_right_lift  = joints_[3]->Position(0);
    position_feedback_pub_->publish(feedback);
  }

  static double Clamp(double value, double minimum, double maximum)
  {
    return std::max(minimum, std::min(value, maximum));
  }

  static rcl_interfaces::msg::SetParametersResult SuccessResult()
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  static rcl_interfaces::msg::SetParametersResult FailureResult(const std::string & reason)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = reason;
    return result;
  }

private:
  gazebo::physics::ModelPtr model_;
  std::array<gazebo::physics::JointPtr, 4> joints_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_connection_;
  rclcpp::Subscription<robot_r2_interfaces::msg::LiftCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<robot_r2_interfaces::msg::LiftFeedback>::SharedPtr position_feedback_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::mutex mutex_;
  std::string command_topic_{"/r2/lift/cmd_lift"};
  std::string position_feedback_topic_{"/r2/lift/position_feedback"};
  std::array<std::string, 4> joint_names_{
    "drive_lift_fl_joint", "drive_lift_fr_joint",
    "drive_lift_rl_joint", "drive_lift_rr_joint"};
  double min_lift_{0.0};
  double max_lift_{0.376};

  // PID gains — updated via ROS parameter callback
  double position_p_gain_{6000.0};
  double position_i_gain_{1000.0};
  double position_d_gain_{20.0};
  double position_i_max_{1000.0};
  double position_i_min_{-1000.0};
  double max_actuation_force_{10000.0};

  // PID state — integral stored directly in output(force) units for live tuning
  std::array<double, 4> integral_term_{};
  std::array<double, 4> prev_error_{};
  bool derivative_state_reset_requested_{false};

  double front_target_{0.0};
  double rear_target_{0.0};
  gazebo::common::Time last_update_time_{0};
};

GZ_REGISTER_MODEL_PLUGIN(RobotR2LiftController)

}  // namespace robot_r2_description
