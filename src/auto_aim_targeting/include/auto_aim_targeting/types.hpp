#ifndef AUTO_AIM_TARGETING__TYPES_HPP_
#define AUTO_AIM_TARGETING__TYPES_HPP_

#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/time.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/gimbal_cmd.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/targets.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"
#include "auto_aim_targeting/config.hpp"
#include "auto_aim_targeting/planning_types.hpp"

namespace rm_auto_aim
{

struct GimbalPoseState
{
  double yaw = 0.0;
  double pitch = 0.0;
  geometry_msgs::msg::Quaternion orientation{};
  rclcpp::Time last_update{0, 0, RCL_ROS_TIME};
  bool fresh = true;
};

struct Observation
{
  vision_msgs::msg::Detection2DArray::ConstSharedPtr detection_msg;
  auto_aim_interfaces::msg::Armors::SharedPtr armors;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  GimbalPoseState pose_state{};
  std::vector<size_t> detection_indices{};
};

struct FireGateResult
{
  bool fire = false;
  std::string blocker{};
};

struct CommandOutput
{
  auto_aim_interfaces::msg::GimbalCmd cmd{};
  geometry_msgs::msg::Twist twist{};
  visualization_msgs::msg::Marker marker{};
};

struct AimResult
{
  auto_aim_interfaces::msg::Targets targets_msg{};
  auto_aim_interfaces::msg::Target target_msg{};
  auto_aim_interfaces::msg::TrackerInfo tracker_info{};
  bool has_tracker_info = false;
  vision_msgs::msg::Detection2D optimal_bbox{};
  visualization_msgs::msg::MarkerArray marker_array{};
  CommandOutput command_output{};
  int best_tracker_id = -1;
};

inline PlanningContext makePlanningContext(
  const TargetingConfig & config,
  const GimbalPoseState & pose_state,
  double transport_delay,
  int previous_tracker_id,
  bool previous_indirect_mode,
  double time_bias)
{
  PlanningContext ctx;
  ctx.time_bias = time_bias;
  ctx.gimbal_response_delay = config.engagement.gimbal_response_delay;
  ctx.gimbal_height = config.pose.gimbal_height;
  ctx.bullet_speed = config.ballistics.bullet_speed;
  ctx.angular_window = config.engagement.angular_window;
  ctx.angular_window_ref_dist = config.engagement.angular_window_ref_dist;
  ctx.min_fire_dist = config.engagement.min_fire_dist;
  ctx.max_fire_dist = config.engagement.max_fire_dist;
  ctx.max_measurement_age = config.engagement.max_measurement_age;
  ctx.current_yaw = pose_state.yaw;
  ctx.current_pitch = pose_state.pitch;
  ctx.max_gimbal_yaw_rate = config.engagement.max_gimbal_yaw_rate;
  ctx.max_gimbal_pitch_rate = config.engagement.max_gimbal_pitch_rate;
  ctx.indirect_vyaw_threshold = config.engagement.indirect_vyaw_threshold;
  ctx.indirect_timing_tolerance = config.engagement.indirect_timing_tolerance;
  ctx.indirect_max_candidates = config.engagement.indirect_max_candidates;
  ctx.visibility_exponent = config.engagement.oblique_exponent;
  ctx.previous_tracker_id = previous_tracker_id;
  ctx.previous_indirect_mode = previous_indirect_mode;
  ctx.transport_delay = transport_delay;
  return ctx;
}

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__TYPES_HPP_
