#include <algorithm>
#include <array>
#include <mutex>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace robot_r2_description
{

class RobotR2GripperController : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);

    command_topic_ = sdf->Get<std::string>("command_topic", "/r2/gripper/grip_cmd").first;
    feedback_topic_ = sdf->Get<std::string>("feedback_topic", "/r2/gripper/grip_feedback").first;
    joint_names_[0] = sdf->Get<std::string>("left_joint_name",  "gripper_left_joint").first;
    joint_names_[1] = sdf->Get<std::string>("right_joint_name", "gripper_right_joint").first;

    // Left:  lower=-0.209 (closed), upper=0 (open)
    // Right: lower=0 (open), upper=0.209 (closed)
    // Single command 0.0=open, 0.209=closed → map to each joint
    stroke_ = sdf->Get<double>("stroke", 0.209).first;

    const double sdf_p = sdf->Get<double>("position_p_gain", 1000.0).first;
    const double sdf_i = sdf->Get<double>("position_i_gain", 200.0).first;
    const double sdf_d = sdf->Get<double>("position_d_gain", 3.0).first;
    const double sdf_imax = sdf->Get<double>("position_i_max", 2000.0).first;
    const double sdf_imin = sdf->Get<double>("position_i_min", -2000.0).first;
    const double sdf_force = sdf->Get<double>("max_actuation_force", 10000.0).first;

    node_->declare_parameter("gripper.position_p_gain", sdf_p);
    node_->declare_parameter("gripper.position_i_gain", sdf_i);
    node_->declare_parameter("gripper.position_d_gain", sdf_d);
    node_->declare_parameter("gripper.position_i_max",  sdf_imax);
    node_->declare_parameter("gripper.position_i_min",  sdf_imin);
    node_->declare_parameter("gripper.max_actuation_force", sdf_force);
    LoadParams();

    for (int i = 0; i < 2; ++i) {
      joints_[i] = model_->GetJoint(joint_names_[i]);
      if (!joints_[i]) {
        return;
      }
    }

    // Command: 0.0 = open (grippers at ends), stroke = closed (grippers at center)
    command_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      command_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = Clamp(msg->data, 0.0, stroke_);
      });

    feedback_pub_ = node_->create_publisher<std_msgs::msg::Float64>(feedback_topic_, rclcpp::QoS(10));

    param_cb_ = node_->add_on_set_parameters_callback(
      std::bind(&RobotR2GripperController::OnParamsChanged, this, std::placeholders::_1));

    last_time_ = model_->GetWorld()->SimTime();
    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RobotR2GripperController::OnUpdate, this));

  }

private:
  void LoadParams()
  {
    p_gain_  = node_->get_parameter("gripper.position_p_gain").as_double();
    i_gain_  = node_->get_parameter("gripper.position_i_gain").as_double();
    d_gain_  = node_->get_parameter("gripper.position_d_gain").as_double();
    i_max_   = node_->get_parameter("gripper.position_i_max").as_double();
    i_min_   = node_->get_parameter("gripper.position_i_min").as_double();
    force_limit_ = node_->get_parameter("gripper.max_actuation_force").as_double();
  }

  rcl_interfaces::msg::SetParametersResult OnParamsChanged(
    const std::vector<rclcpp::Parameter> & params)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & p : params) {
      if (p.get_name() == "gripper.position_p_gain") { p_gain_ = p.as_double(); }
      else if (p.get_name() == "gripper.position_i_gain") {
        i_gain_ = p.as_double();
        if (i_gain_ <= 1e-9) for (int i=0;i<2;++i) integral_[i] = 0.0;
      }
      else if (p.get_name() == "gripper.position_d_gain") { d_gain_ = p.as_double(); deriv_reset_ = true; }
      else if (p.get_name() == "gripper.position_i_max") { i_max_ = p.as_double(); }
      else if (p.get_name() == "gripper.position_i_min") { i_min_ = p.as_double(); }
      else if (p.get_name() == "gripper.max_actuation_force") { force_limit_ = p.as_double(); }
    }
    rcl_interfaces::msg::SetParametersResult r; r.successful = true; return r;
  }

  void OnUpdate()
  {
    double cmd, pg, ig, dg, imax, imin, flim;
    bool reset_d;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = command_; pg = p_gain_; ig = i_gain_; dg = d_gain_;
      imax = i_max_; imin = i_min_; flim = force_limit_; reset_d = deriv_reset_; deriv_reset_ = false;
    }

    // Map single command to individual joint targets
    // Left:  open=0 → target=0,   closed=stroke → target=-stroke
    // Right: open=0 → target=0,   closed=stroke → target=+stroke
    double targets[2] = { -cmd, cmd };

    auto now = model_->GetWorld()->SimTime();
    double dt = (now - last_time_).Double();
    if (dt <= 0.0 || dt > 1.0) dt = 0.001;
    last_time_ = now;

    for (int i = 0; i < 2; ++i) {
      double pos = joints_[i]->Position(0);
      double err = targets[i] - pos;
      if (reset_d) prev_err_[i] = err;

      double deriv = (dt > 1e-6) ? (err - prev_err_[i]) / dt : 0.0;
      double p_term = err * pg;
      double d_term = deriv * dg;

      double cand_i = integral_[i];
      if (ig > 1e-9) cand_i = Clamp(integral_[i] + ig * err * dt, imin, imax);
      else { cand_i = 0.0; integral_[i] = 0.0; }

      double raw = p_term + cand_i + d_term;
      double force = Clamp(raw, -flim, flim);

      bool sat_hi = raw > flim, sat_lo = raw < -flim;
      bool deeper = (sat_hi && err > 0.0) || (sat_lo && err < 0.0);
      if (ig > 1e-9 && !deeper) { integral_[i] = cand_i; force = Clamp(p_term + integral_[i] + d_term, -flim, flim); }

      prev_err_[i] = err;
      joints_[i]->SetForce(0, force);
    }

    auto fb = std_msgs::msg::Float64();
    fb.data = joints_[0]->Position(0);  // report left gripper position
    feedback_pub_->publish(fb);
  }

  static double Clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }

  gazebo::physics::ModelPtr model_;
  std::array<gazebo::physics::JointPtr, 2> joints_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_conn_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr feedback_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  std::mutex mutex_;
  std::string command_topic_{"/r2/gripper/grip_cmd"};
  std::string feedback_topic_{"/r2/gripper/grip_feedback"};
  std::array<std::string, 2> joint_names_{"gripper_left_joint", "gripper_right_joint"};
  double stroke_{0.209};
  double p_gain_{1000.0}, i_gain_{200.0}, d_gain_{3.0};
  double i_max_{2000.0}, i_min_{-2000.0}, force_limit_{10000.0};
  double command_{0.0};
  std::array<double, 2> integral_{};
  std::array<double, 2> prev_err_{};
  bool deriv_reset_{false};
  gazebo::common::Time last_time_{0};
};

GZ_REGISTER_MODEL_PLUGIN(RobotR2GripperController)
}  // namespace robot_r2_description
