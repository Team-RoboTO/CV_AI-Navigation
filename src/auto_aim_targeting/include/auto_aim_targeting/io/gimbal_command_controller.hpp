#ifndef AUTO_AIM_TARGETING__IO__GIMBAL_COMMAND_CONTROLLER_HPP_
#define AUTO_AIM_TARGETING__IO__GIMBAL_COMMAND_CONTROLLER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>

#include "auto_aim_interfaces/msg/gimbal_cmd.hpp"
#include "auto_aim_targeting/planning_types.hpp"
#include "auto_aim_targeting/types.hpp"

namespace rm_auto_aim
{

class GimbalCommandController
{
public:
  GimbalCommandController(
    double max_cmd_pitch_angle_deg,
    double max_cmd_yaw_angle_deg,
    double cmd_smooth_alpha = 1.0,
    double fire_yaw_tolerance = 0.03,
    double fire_pitch_tolerance = 0.03)
  : max_cmd_pitch_angle_rad_(max_cmd_pitch_angle_deg * M_PI / 180.0),
    max_cmd_yaw_angle_rad_(max_cmd_yaw_angle_deg * M_PI / 180.0),
    cmd_smooth_alpha_(cmd_smooth_alpha),
    fire_yaw_tolerance_(fire_yaw_tolerance),
    fire_pitch_tolerance_(fire_pitch_tolerance) {}

  CommandOutput makeHoldOutput(const std_msgs::msg::Header & header);
  CommandOutput makeShotOutput(
    const std_msgs::msg::Header & header,
    const ShotPlan & plan,
    bool fire);

private:
  double max_cmd_pitch_angle_rad_;
  double max_cmd_yaw_angle_rad_;
  double cmd_smooth_alpha_;
  double fire_yaw_tolerance_;
  double fire_pitch_tolerance_;
  bool smooth_initialized_ = false;
  int last_smoothed_tracker_id_ = -1;
  double smoothed_yaw_ = 0.0;
  double smoothed_pitch_ = 0.0;
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__IO__GIMBAL_COMMAND_CONTROLLER_HPP_
