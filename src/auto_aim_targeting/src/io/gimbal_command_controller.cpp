// ============================================================================
// gimbal_command_controller.cpp — Smooths and formats gimbal commands.
//
// ENTRYPOINTS:
//   makeHoldOutput → build zero-command when no target is tracked
//   makeShotOutput → build GimbalCmd + Twist + impact marker from a ShotPlan
//
// HELPER (stepdown):
//   reset → clear EMA smoothing state (called by both entrypoints)
// ============================================================================
#include "auto_aim_targeting/io/gimbal_command_controller.hpp"

#include <algorithm>
#include <cmath>

namespace rm_auto_aim
{

CommandOutput GimbalCommandController::makeHoldOutput(const std_msgs::msg::Header & header)
{
  this->reset();

  CommandOutput output;
  output.cmd.header = header;
  output.cmd.pitch = 0.0;
  output.cmd.yaw = 0.0;
  output.cmd.distance = 0.0;
  output.cmd.fire_cmd = false;

  output.marker.header = header;
  output.marker.ns = "impact_point";
  output.marker.action = visualization_msgs::msg::Marker::DELETE;
  return output;
}

CommandOutput GimbalCommandController::makeShotOutput(
  const std_msgs::msg::Header & header,
  const ShotPlan & plan,
  bool fire,
  bool reset_smoothing)
{
  if (reset_smoothing) {
    this->reset();
  }

  const double rel_yaw =
    std::max(-this->max_cmd_yaw_angle_rad_, std::min(plan.relative_yaw, this->max_cmd_yaw_angle_rad_));
  const double rel_pitch =
    std::max(-this->max_cmd_pitch_angle_rad_, std::min(plan.relative_pitch, this->max_cmd_pitch_angle_rad_));

  if (!this->initialized_) {
    this->smoothed_rel_yaw_ = rel_yaw;
    this->smoothed_rel_pitch_ = rel_pitch;
    this->initialized_ = true;
  } else {
    this->smoothed_rel_yaw_ =
      this->cmd_smooth_alpha_ * rel_yaw +
      (1.0 - this->cmd_smooth_alpha_) * this->smoothed_rel_yaw_;
    this->smoothed_rel_pitch_ =
      this->cmd_smooth_alpha_ * rel_pitch +
      (1.0 - this->cmd_smooth_alpha_) * this->smoothed_rel_pitch_;
  }

  CommandOutput output;
  output.cmd.header = header;
  output.cmd.pitch = this->smoothed_rel_pitch_ * 180.0 / M_PI;
  output.cmd.yaw = this->smoothed_rel_yaw_ * 180.0 / M_PI;
  output.cmd.distance = plan.range;
  output.cmd.fire_cmd = fire;

  output.twist.angular.x = 0.0;
  output.twist.angular.y = this->smoothed_rel_pitch_;
  output.twist.angular.z = this->smoothed_rel_yaw_;

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
  output.marker.color.g = fire ? 1.0 : 0.2;
  output.marker.color.r = fire ? 0.2 : 1.0;

  return output;
}

void GimbalCommandController::reset()
{
  this->initialized_ = false;
  this->smoothed_rel_yaw_ = 0.0;
  this->smoothed_rel_pitch_ = 0.0;
}

}  // namespace rm_auto_aim
