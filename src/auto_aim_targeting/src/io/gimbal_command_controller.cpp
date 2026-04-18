// ============================================================================
// gimbal_command_controller.cpp — Formats gimbal commands from shot plans.
//
// NO EMA smoothing. The gimbal PID controller downstream handles convergence.
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
  double cmd_yaw = std::clamp(plan.absolute_yaw, -this->max_cmd_yaw_angle_rad_, this->max_cmd_yaw_angle_rad_);
  double cmd_pitch = std::clamp(plan.absolute_pitch, -this->max_cmd_pitch_angle_rad_, this->max_cmd_pitch_angle_rad_);

  CommandOutput output;
  output.cmd.header = header;
  output.cmd.yaw = cmd_yaw * 180.0 / M_PI;
  output.cmd.pitch = cmd_pitch * 180.0 / M_PI;
  output.cmd.distance = plan.range;
  output.cmd.fire_cmd = fire;

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
  output.marker.color.g = fire ? 1.0 : 0.2;
  output.marker.color.r = fire ? 0.2 : 1.0;

  return output;
}

}  // namespace rm_auto_aim
