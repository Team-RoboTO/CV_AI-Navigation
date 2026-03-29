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
    double cmd_smooth_alpha,
    double max_cmd_pitch_angle_deg,
    double max_cmd_yaw_angle_deg)
  : cmd_smooth_alpha_(cmd_smooth_alpha),
    max_cmd_pitch_angle_rad_(max_cmd_pitch_angle_deg * M_PI / 180.0),
    max_cmd_yaw_angle_rad_(max_cmd_yaw_angle_deg * M_PI / 180.0) {}

  void reset();
  CommandOutput makeHoldOutput(const std_msgs::msg::Header & header);
  CommandOutput makeShotOutput(
    const std_msgs::msg::Header & header,
    const ShotPlan & plan,
    bool fire,
    bool reset_smoothing);

private:
  double cmd_smooth_alpha_;
  double max_cmd_pitch_angle_rad_;
  double max_cmd_yaw_angle_rad_;
  double smoothed_rel_yaw_ = 0.0;
  double smoothed_rel_pitch_ = 0.0;
  bool initialized_ = false;
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__IO__GIMBAL_COMMAND_CONTROLLER_HPP_
