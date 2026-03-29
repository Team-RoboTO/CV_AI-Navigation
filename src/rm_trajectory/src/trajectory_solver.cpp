/**
 * @file trajectory_solver.cpp
 * @brief Auto-aim trajectory solver node.
 *
 * This node receives tracked 3D targets and decides where the gimbal should aim
 * and whether the system is allowed to fire.
 *
 * Educational overview:
 *   1. Ballistics:
 *      The bullet does not travel instantly and does not move in a perfectly
 *      straight line.  Gravity pulls it downward, and drag can slow it down.
 *      We therefore solve for the pitch angle and flight time instead of aiming
 *      with a naive straight line.
 *
 *   2. Valid vs reachable:
 *      A ballistic equation may have a mathematically valid solution, but the
 *      required pitch could still exceed the mechanical limits of the gimbal.
 *      We keep these concepts separate:
 *        - valid     = the math produced a real physical trajectory
 *        - reachable = the robot can mechanically point there
 *
 *   3. Measurement age and transport delay:
 *      The target message already has some age when this node receives it.
 *      We explicitly model that delay so prediction is done from "where the
 *      target is now" rather than "where it was when the camera saw it".
 *
 *   4. Direct vs indirect mode:
 *      When the enemy spins slowly, we can aim at the face that is visible now.
 *      When it spins quickly, it can be better to predict when a face will rotate
 *      into a good firing position and fire at that future alignment instead.
 *
 *   5. Hysteresis:
 *      Switching between direct and indirect modes with a single threshold can
 *      cause rapid mode toggling near the boundary.  Hysteresis uses different
 *      enter/exit thresholds so the mode choice stays stable.
 */
#include "rm_trajectory/trajectory_solver.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

#include "rclcpp_components/register_node_macro.hpp"

namespace rm_auto_aim
{

TrajectorySolverNode::TrajectorySolverNode(
  const rclcpp::NodeOptions & options)
: Node("trajectory_solver", options),
  ballistics_solver_(ballistics_params_),
  predictor_(ballistics_solver_),
  selector_(predictor_, cost_weights_)
{
  bullet_speed_ = declare_parameter("bullet_speed", 25.0);
  gravity_ = declare_parameter("gravity", 9.8);
  linear_drag_coeff_ = declare_parameter("linear_drag_coeff", 0.01);
  quadratic_drag_coeff_ = declare_parameter("quadratic_drag_coeff", 0.01);
  use_quadratic_drag_ = declare_parameter("use_quadratic_drag", false);
  time_bias_ = declare_parameter("time_bias", 0.08);
  time_bias_alpha_ = declare_parameter("time_bias_alpha", 0.35);
  gimbal_response_delay_ = declare_parameter("gimbal_response_delay", 0.0);
  gimbal_height_ = declare_parameter("gimbal.height", 0.5);
  min_fire_dist_ = declare_parameter("min_fire_dist", 0.5);
  max_fire_dist_ = declare_parameter("max_fire_dist", 10.0);
  angular_window_ = declare_parameter("angular_window", 0.09);
  angular_window_ref_dist_ = declare_parameter("angular_window_ref_dist", 3.0);
  max_measurement_age_ = declare_parameter("max_measurement_age", 0.10);
  max_gimbal_yaw_rate_ = declare_parameter("max_gimbal_yaw_rate", 6.0);
  max_gimbal_pitch_rate_ = declare_parameter("max_gimbal_pitch_rate", 4.0);
  indirect_vyaw_threshold_ = declare_parameter("indirect_vyaw_threshold", 3.0);
  indirect_timing_tolerance_ = declare_parameter("indirect_timing_tolerance", 0.02);
  indirect_max_candidates_ = declare_parameter("indirect_max_candidates", 8);
  oblique_exponent_ = declare_parameter("oblique_exponent", 2.0);
  gimbal_pitch_max_ = declare_parameter("gimbal_pitch_max", 0.524);
  gimbal_pitch_min_ = declare_parameter("gimbal_pitch_min", -0.524);
  cmd_smooth_alpha_ = declare_parameter("cmd_smooth_alpha", 0.4);
  max_cmd_angle_ = declare_parameter("max_cmd_angle", 15.0);
  latency_gate_sigma_ = declare_parameter("latency_gate_sigma", 2.5);
  pose_source_ = declare_parameter("pose_source", "none");
  pose_timeout_ = declare_parameter("micro_pose_timeout", 0.15);
  yaw_sign_ = declare_parameter("gimbal.yaw_sign", 1.0);
  pitch_sign_ = declare_parameter("gimbal.pitch_sign", 1.0);
  latency_warmup_samples_ = declare_parameter("latency_warmup_samples", 5);

  // Engagement cost weights.
  // These weights control HOW the solver chooses the best shot among multiple
  // candidates.  Lower total score is better.
  cost_weights_.range = declare_parameter("cost.range", cost_weights_.range);
  cost_weights_.flight_time = declare_parameter("cost.flight_time", cost_weights_.flight_time);
  cost_weights_.uncertainty = declare_parameter("cost.uncertainty", cost_weights_.uncertainty);
  cost_weights_.slew = declare_parameter("cost.slew", cost_weights_.slew);
  cost_weights_.switch_target = declare_parameter("cost.switch_target", cost_weights_.switch_target);
  cost_weights_.staleness = declare_parameter("cost.staleness", cost_weights_.staleness);
  cost_weights_.temp_lost = declare_parameter("cost.temp_lost", cost_weights_.temp_lost);
  cost_weights_.low_visibility = declare_parameter("cost.low_visibility", cost_weights_.low_visibility);
  cost_weights_.negative_margin = declare_parameter(
    "cost.negative_margin", cost_weights_.negative_margin);

  ballistics_params_.gravity = gravity_;
  ballistics_params_.linear_drag_coeff = linear_drag_coeff_;
  ballistics_params_.quadratic_drag_coeff = quadratic_drag_coeff_;
  ballistics_params_.use_quadratic_drag = use_quadratic_drag_;
  ballistics_params_.pitch_min = gimbal_pitch_min_;
  ballistics_params_.pitch_max = gimbal_pitch_max_;

  ballistics_solver_ = BallisticsSolver(ballistics_params_);

  auto target_qos = rclcpp::SensorDataQoS()
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::Volatile)
    .keep_last(30);

  targets_sub_ = create_subscription<auto_aim_interfaces::msg::Targets>(
    "/tracker/targets", target_qos,
    std::bind(&TrajectorySolverNode::targetsCallback, this, std::placeholders::_1));

  cmd_pub_ = create_publisher<auto_aim_interfaces::msg::GimbalCmd>(
    "/tracker/cmd_gimbal", rclcpp::SensorDataQoS());
  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10));
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
    "/trajectory/marker", rclcpp::QoS(10));

  if (pose_source_ == "micro_pose") {
    micro_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/micro_pose", rclcpp::SensorDataQoS(),
      std::bind(&TrajectorySolverNode::microPoseCallback, this, std::placeholders::_1));
  } else if (pose_source_ == "camera_imu") {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&TrajectorySolverNode::cameraImuCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_WARN(
      get_logger(),
      "pose_source='none': use only for software tests. Selection stays valid, but "
      "hardware-relative commands are not guaranteed without real pose feedback.");
  }
}

SolverContext TrajectorySolverNode::buildContext(double transport_delay) const
{
  SolverContext ctx;
  ctx.time_bias = time_bias_;
  ctx.gimbal_response_delay = gimbal_response_delay_;
  ctx.gimbal_height = gimbal_height_;
  ctx.bullet_speed = bullet_speed_;
  ctx.angular_window = angular_window_;
  ctx.angular_window_ref_dist = angular_window_ref_dist_;
  ctx.min_fire_dist = min_fire_dist_;
  ctx.max_fire_dist = max_fire_dist_;
  ctx.max_measurement_age = max_measurement_age_;
  ctx.current_yaw = current_yaw_;
  ctx.current_pitch = current_pitch_;
  ctx.max_gimbal_yaw_rate = max_gimbal_yaw_rate_;
  ctx.max_gimbal_pitch_rate = max_gimbal_pitch_rate_;
  ctx.indirect_vyaw_threshold = indirect_vyaw_threshold_;
  ctx.indirect_timing_tolerance = indirect_timing_tolerance_;
  ctx.indirect_max_candidates = indirect_max_candidates_;
  ctx.visibility_exponent = oblique_exponent_;
  ctx.previous_tracker_id = previous_tracker_id_;
  ctx.previous_indirect_mode = indirect_mode_active_;
  ctx.transport_delay = transport_delay;
  return ctx;
}

bool TrajectorySolverNode::poseIsFresh() const
{
  if (pose_source_ == "none") {
    return true;
  }
  if (last_pose_time_.nanoseconds() == 0) {
    return false;
  }
  return (this->now() - last_pose_time_).seconds() <= pose_timeout_;
}

void TrajectorySolverNode::microPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  const double pitch = pitch_sign_ * msg->pose.position.x;
  const double yaw = yaw_sign_ * msg->pose.position.y;
  
  // We intentionally do NOT clamp yaw here.  Yaw is an angle on a circle and
  // a free-spinning gimbal may legitimately cross ±90° or even ±180°.
  // We keep only a loose pitch sanity check because absurd pitch values usually
  // indicate a malformed upstream packet, not a real robot state.
  if (std::abs(pitch) < 2.0) {
    current_pitch_ = pitch;
    current_yaw_ = yaw;
    last_pose_time_ = this->now();
  }
}

void TrajectorySolverNode::cameraImuCallback(
  const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  tf2::Quaternion q;
  tf2::fromMsg(msg->orientation, q);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  current_pitch_ = pitch_sign_ * pitch;
  current_yaw_ = yaw_sign_ * yaw;
  last_pose_time_ = this->now();
}

void TrajectorySolverNode::publishSafeCommand(const std_msgs::msg::Header & header)
{
  auto_aim_interfaces::msg::GimbalCmd cmd;
  cmd.header = header;
  cmd.pitch = 0.0;
  cmd.yaw = 0.0;
  cmd.distance = 0.0;
  cmd.fire_cmd = false;
  cmd_pub_->publish(cmd);

  geometry_msgs::msg::Twist twist;
  // NOTE: Using Twist to publish boolean flags and absolute/relative angles (in degrees).
  // Semantically incorrect but kept for backward compatibility with the consumer node.
  twist_pub_->publish(twist);

  visualization_msgs::msg::Marker del;
  del.header = header;
  del.ns = "impact_point";
  del.action = visualization_msgs::msg::Marker::DELETE;
  marker_pub_->publish(del);

  cmd_smooth_initialized_ = false;
}

void TrajectorySolverNode::publishShotPlan(
  const std_msgs::msg::Header & header,
  const ShotPlan & plan,
  bool fire)
{
  const double max_rad = max_cmd_angle_ * M_PI / 180.0;
  double rel_yaw = std::clamp(plan.relative_yaw, -max_rad, max_rad);
  double rel_pitch = std::clamp(plan.relative_pitch, -max_rad, max_rad);

  if (!cmd_smooth_initialized_) {
    smoothed_rel_yaw_ = rel_yaw;
    smoothed_rel_pitch_ = rel_pitch;
    cmd_smooth_initialized_ = true;
  } else {
    smoothed_rel_yaw_ =
      cmd_smooth_alpha_ * rel_yaw + (1.0 - cmd_smooth_alpha_) * smoothed_rel_yaw_;
    smoothed_rel_pitch_ =
      cmd_smooth_alpha_ * rel_pitch + (1.0 - cmd_smooth_alpha_) * smoothed_rel_pitch_;
  }

  auto_aim_interfaces::msg::GimbalCmd cmd;
  cmd.header = header;
  cmd.pitch = smoothed_rel_pitch_ * 180.0 / M_PI;
  cmd.yaw = smoothed_rel_yaw_ * 180.0 / M_PI;
  cmd.distance = plan.range;
  cmd.fire_cmd = fire;
  cmd_pub_->publish(cmd);

  geometry_msgs::msg::Twist twist;
  // NOTE: Using Twist to publish boolean flags and absolute/relative angles (in degrees).
  // Semantically incorrect but kept for backward compatibility with the consumer node.
  twist.angular.x = fire ? 1.0 : 0.0;
  twist.angular.y = cmd.pitch;
  twist.angular.z = cmd.yaw;
  twist_pub_->publish(twist);

  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "impact_point";
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = plan.x;
  marker.pose.position.y = plan.y;
  marker.pose.position.z = plan.z;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.g = fire ? 1.0 : 0.2;
  marker.color.r = fire ? 0.2 : 1.0;
  marker_pub_->publish(marker);
}

void TrajectorySolverNode::targetsCallback(
  auto_aim_interfaces::msg::Targets::UniquePtr msg)
{
  if (msg->targets.empty()) {
    publishSafeCommand(msg->header);
    previous_tracker_id_ = -1;
    return;
  }

  const double measured_latency = (this->now() - msg->header.stamp).seconds();
  if (measured_latency > 0.001 && measured_latency < 0.5) {
    const double residual = measured_latency - time_bias_;
    const double sigma = std::sqrt(std::max(time_bias_var_, 1e-8));

    const bool accept =
      (latency_warmup_count_ < latency_warmup_samples_) ||
      (std::abs(residual) < latency_gate_sigma_ * sigma);

    if (accept) {
      time_bias_ = time_bias_alpha_ * measured_latency +
        (1.0 - time_bias_alpha_) * time_bias_;

      const double new_residual = measured_latency - time_bias_;
      time_bias_var_ = time_bias_alpha_ * new_residual * new_residual +
        (1.0 - time_bias_alpha_) * time_bias_var_;

      ++latency_warmup_count_;
    }
  }

  const double transport_delay = std::max(0.0, (this->now() - msg->header.stamp).seconds());
  const SolverContext ctx = buildContext(transport_delay);
  const auto best = selector_.selectBestTarget(*msg, ctx);
  if (!best.has_value()) {
    publishSafeCommand(msg->header);
    previous_tracker_id_ = -1;
    return;
  }

  const ShotPlan & plan = *best;
  const bool pose_fresh = poseIsFresh();
  const bool can_slew =
    std::abs(plan.relative_yaw) <= max_gimbal_yaw_rate_ * std::max(plan.predict_time, 0.03) &&
    std::abs(plan.relative_pitch) <= max_gimbal_pitch_rate_ * std::max(plan.predict_time, 0.03);
  const bool fire =
    pose_fresh &&
    !plan.measurement_stale &&
    !plan.temp_lost &&
    plan.ballistic_valid &&
    plan.reachable &&
    plan.range >= min_fire_dist_ &&
    plan.range <= max_fire_dist_ &&
    plan.fire_window_margin >= 0.0 &&
    can_slew;

  if (plan.tracker_id != previous_tracker_id_) {
    cmd_smooth_initialized_ = false;
  }

  publishShotPlan(msg->header, plan, fire);
  previous_tracker_id_ = plan.tracker_id;
  indirect_mode_active_ = plan.indirect;
}

}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::TrajectorySolverNode)
