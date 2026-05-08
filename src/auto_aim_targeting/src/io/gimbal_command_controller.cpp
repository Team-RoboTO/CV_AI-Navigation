// ============================================================================
// gimbal_command_controller.cpp — Formats gimbal commands from shot plans.
//
// Optional EMA smoothing is applied to absolute yaw/pitch destinations before
// publishing. While the smoothed command is still lagging the requested shot
// plan, fire is suppressed so command and fire do not disagree.
//
// Commands are ABSOLUTE angles (yaw/pitch in odom frame), not relative.
// The downstream controller (lower computer) knows its current angles and
// computes the delta internally.
//
// This avoids the fundamental problem where relative angles require accurate
// gimbal feedback (pose_source), which may be unavailable or stale.
// ============================================================================
#include "auto_aim_targeting/io/gimbal_command_controller.hpp"

#include <algorithm>
#include <cmath>

#include <angles/angles.h>

namespace rm_auto_aim
{

CommandOutput GimbalCommandController::makeHoldOutput(const std_msgs::msg::Header & header)
{
  CommandOutput output;
  output.cmd.header = header;
  output.cmd.pitch = 0.0;
  output.cmd.yaw = 0.0;
  output.cmd.distance = 0.0;
  output.cmd.fire_cmd = false;
  output.twist.angular.x = 0.0;
  output.twist.angular.y = 0.0;
  output.twist.angular.z = 0.0;
  this->smooth_initialized_ = false;
  this->last_smoothed_tracker_id_ = -1;

  output.marker.header = header;
  output.marker.ns = "impact_point";
  output.marker.action = visualization_msgs::msg::Marker::DELETE;
  return output;
}

CommandOutput GimbalCommandController::makeShotOutput(
  const std_msgs::msg::Header & header,
  const ShotPlan & plan,
  bool fire)
{
  const double limited_yaw =
    std::clamp(plan.absolute_yaw, -this->max_cmd_yaw_angle_rad_, this->max_cmd_yaw_angle_rad_);
  const double limited_pitch =
    std::clamp(plan.absolute_pitch, -this->max_cmd_pitch_angle_rad_, this->max_cmd_pitch_angle_rad_);
  const bool command_clamped =
    std::abs(angles::shortest_angular_distance(plan.absolute_yaw, limited_yaw)) > 1e-6 ||
    std::abs(plan.absolute_pitch - limited_pitch) > 1e-6;

  double cmd_yaw = limited_yaw;
  double cmd_pitch = limited_pitch;
  if (this->smooth_initialized_ && plan.tracker_id != this->last_smoothed_tracker_id_) {
    this->smooth_initialized_ = false;
  }
  const double alpha = std::clamp(this->cmd_smooth_alpha_, 0.0, 1.0);
  if (alpha < 1.0) {
    if (!this->smooth_initialized_) {
      this->smoothed_yaw_ = cmd_yaw;
      this->smoothed_pitch_ = cmd_pitch;
      this->smooth_initialized_ = true;
    } else {
      this->smoothed_yaw_ = angles::normalize_angle(
        this->smoothed_yaw_ +
        alpha * angles::shortest_angular_distance(this->smoothed_yaw_, cmd_yaw));
      this->smoothed_pitch_ =
        alpha * cmd_pitch + (1.0 - alpha) * this->smoothed_pitch_;
      cmd_yaw = this->smoothed_yaw_;
      cmd_pitch = this->smoothed_pitch_;
    }
  } else {
    this->smooth_initialized_ = false;
  }
  this->last_smoothed_tracker_id_ = plan.tracker_id;

  const bool smoothing_lagged =
    std::abs(angles::shortest_angular_distance(cmd_yaw, limited_yaw)) >
      this->fire_yaw_tolerance_ ||
    std::abs(cmd_pitch - limited_pitch) > this->fire_pitch_tolerance_;
  const bool final_fire = fire && !command_clamped && !smoothing_lagged;

  CommandOutput output;
  output.cmd.header = header;
  output.cmd.yaw = cmd_yaw * 180.0 / M_PI;
  output.cmd.pitch = cmd_pitch * 180.0 / M_PI;
  output.cmd.distance = plan.range;
  output.cmd.fire_cmd = final_fire;

  output.twist.angular.x = final_fire ? 1.0 : 0.0;
  output.twist.angular.y = cmd_pitch;
  output.twist.angular.z = cmd_yaw;

  output.marker.header = header;
  output.marker.ns = "impact_point";
  output.marker.type = visualization_msgs::msg::Marker::SPHERE;
  output.marker.action = visualization_msgs::msg::Marker::ADD;
  output.marker.pose.position.x = plan.x;
  output.marker.pose.position.y = plan.y;
  output.marker.pose.position.z = plan.z;
  output.marker.pose.orientation.w = 1.0;
  output.marker.scale.x = output.marker.scale.y = output.marker.scale.z = 0.1;
  output.marker.color.a = 1.0;
  output.marker.color.g = final_fire ? 1.0 : 0.2;
  output.marker.color.r = final_fire ? 0.2 : 1.0;

  return output;
}

}  // namespace rm_auto_aim
