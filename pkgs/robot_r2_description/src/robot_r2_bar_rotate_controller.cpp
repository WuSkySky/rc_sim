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

class RobotR2BarRotateController : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);

    command_topic_ = sdf->Get<std::string>("command_topic", "/r2/gripper/rotate_cmd").first;
    feedback_topic_ = sdf->Get<std::string>("feedback_topic", "/r2/gripper/rotate_feedback").first;
    joint_name_ = sdf->Get<std::string>("joint_name", "long_bar_revolute_joint").first;

    min_pos_ = sdf->Get<double>("min_position", -3.14159).first;
    max_pos_ = sdf->Get<double>("max_position",  0.0).first;
    if (min_pos_ > max_pos_) std::swap(min_pos_, max_pos_);

    const double sdf_p = sdf->Get<double>("position_p_gain", 60.0).first;
    const double sdf_i = sdf->Get<double>("position_i_gain", 40.0).first;
    const double sdf_d = sdf->Get<double>("position_d_gain", 0.0).first;
    const double sdf_imax = sdf->Get<double>("position_i_max", 500.0).first;
    const double sdf_imin = sdf->Get<double>("position_i_min", -500.0).first;
    const double sdf_force = sdf->Get<double>("max_actuation_force", 80.0).first;

    node_->declare_parameter("bar_rotate.position_p_gain", sdf_p);
    node_->declare_parameter("bar_rotate.position_i_gain", sdf_i);
    node_->declare_parameter("bar_rotate.position_d_gain", sdf_d);
    node_->declare_parameter("bar_rotate.position_i_max",  sdf_imax);
    node_->declare_parameter("bar_rotate.position_i_min",  sdf_imin);
    node_->declare_parameter("bar_rotate.max_actuation_force", sdf_force);
    LoadParams();

    joint_ = model_->GetJoint(joint_name_);
    if (!joint_) {
      return;
    }

    command_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      command_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        target_ = Clamp(msg->data, min_pos_, max_pos_);
      });

    feedback_pub_ = node_->create_publisher<std_msgs::msg::Float64>(feedback_topic_, rclcpp::QoS(10));

    param_cb_ = node_->add_on_set_parameters_callback(
      std::bind(&RobotR2BarRotateController::OnParamsChanged, this, std::placeholders::_1));

    last_time_ = model_->GetWorld()->SimTime();
    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RobotR2BarRotateController::OnUpdate, this));

  }

private:
  void LoadParams()
  {
    p_gain_  = node_->get_parameter("bar_rotate.position_p_gain").as_double();
    i_gain_  = node_->get_parameter("bar_rotate.position_i_gain").as_double();
    d_gain_  = node_->get_parameter("bar_rotate.position_d_gain").as_double();
    i_max_   = node_->get_parameter("bar_rotate.position_i_max").as_double();
    i_min_   = node_->get_parameter("bar_rotate.position_i_min").as_double();
    force_limit_ = node_->get_parameter("bar_rotate.max_actuation_force").as_double();
  }

  rcl_interfaces::msg::SetParametersResult OnParamsChanged(
    const std::vector<rclcpp::Parameter> & params)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & p : params) {
      if (p.get_name() == "bar_rotate.position_p_gain") { p_gain_ = p.as_double(); }
      else if (p.get_name() == "bar_rotate.position_i_gain") {
        i_gain_ = p.as_double();
        if (i_gain_ <= 1e-9) integral_ = 0.0;
        else integral_ = Clamp(integral_, i_min_, i_max_);
      }
      else if (p.get_name() == "bar_rotate.position_d_gain") { d_gain_ = p.as_double(); deriv_reset_ = true; }
      else if (p.get_name() == "bar_rotate.position_i_max") { i_max_ = p.as_double(); }
      else if (p.get_name() == "bar_rotate.position_i_min") { i_min_ = p.as_double(); }
      else if (p.get_name() == "bar_rotate.max_actuation_force") { force_limit_ = p.as_double(); }
    }
    rcl_interfaces::msg::SetParametersResult r; r.successful = true; return r;
  }

  void OnUpdate()
  {
    double target, pg, ig, dg, imax, imin, flim;
    bool reset_d;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target = target_; pg = p_gain_; ig = i_gain_; dg = d_gain_;
      imax = i_max_; imin = i_min_; flim = force_limit_; reset_d = deriv_reset_; deriv_reset_ = false;
    }

    auto now = model_->GetWorld()->SimTime();
    double dt = (now - last_time_).Double();
    if (dt <= 0.0 || dt > 1.0) dt = 0.001;
    last_time_ = now;

    double pos = joint_->Position(0);
    double err = target - pos;
    if (reset_d) prev_err_ = err;

    double deriv = (dt > 1e-6) ? (err - prev_err_) / dt : 0.0;
    double p_term = err * pg;
    double d_term = deriv * dg;

    double cand_i = integral_;
    if (ig > 1e-9) cand_i = Clamp(integral_ + ig * err * dt, imin, imax);
    else { cand_i = 0.0; integral_ = 0.0; }

    double raw = p_term + cand_i + d_term;
    double force = Clamp(raw, -flim, flim);

    bool sat_hi = raw > flim, sat_lo = raw < -flim;
    bool deeper = (sat_hi && err > 0.0) || (sat_lo && err < 0.0);
    if (ig > 1e-9 && !deeper) { integral_ = cand_i; force = Clamp(p_term + integral_ + d_term, -flim, flim); }

    prev_err_ = err;
    joint_->SetForce(0, force);

    auto fb = std_msgs::msg::Float64();
    fb.data = pos;
    feedback_pub_->publish(fb);
  }

  static double Clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr joint_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_conn_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr feedback_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  std::mutex mutex_;
  std::string command_topic_{"/r2/gripper/rotate_cmd"};
  std::string feedback_topic_{"/r2/gripper/rotate_feedback"};
  std::string joint_name_{"long_bar_revolute_joint"};
  double min_pos_{-3.14159}, max_pos_{0.0};
  double p_gain_{60.0}, i_gain_{40.0}, d_gain_{0.0};
  double i_max_{500.0}, i_min_{-500.0}, force_limit_{80.0};
  double target_{0.0}, integral_{0.0}, prev_err_{0.0};
  bool deriv_reset_{false};
  gazebo::common::Time last_time_{0};
};

GZ_REGISTER_MODEL_PLUGIN(RobotR2BarRotateController)
}  // namespace robot_r2_description
