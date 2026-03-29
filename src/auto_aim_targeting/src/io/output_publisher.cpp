// ============================================================================
// output_publisher.cpp — Aggregates pipeline results into ROS messages.
//
// ENTRYPOINTS:
//   buildAimResult → assemble targets, gimbal commands, markers, debug info
//   publish          → send all messages to their respective ROS topics
// ============================================================================
#include "auto_aim_targeting/io/output_publisher.hpp"

#include <utility>

namespace rm_auto_aim
{

TargetingOutputPublisher::TargetingOutputPublisher(
  const TargetingConfig & config,
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub,
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub,
  rclcpp::Publisher<auto_aim_interfaces::msg::Targets>::SharedPtr targets_pub,
  rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr optimal_bbox_pub,
  rclcpp::Publisher<auto_aim_interfaces::msg::GimbalCmd>::SharedPtr cmd_pub,
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub,
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub,
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub)
: targeting_debug_publisher_(config.measurement.target_frame),
  gimbal_command_controller_(
    config.visualization.cmd_smooth_alpha,
    config.visualization.max_cmd_pitch_angle_deg,
    config.visualization.max_cmd_yaw_angle_deg),
  info_pub_(std::move(info_pub)),
  target_pub_(std::move(target_pub)),
  targets_pub_(std::move(targets_pub)),
  optimal_bbox_pub_(std::move(optimal_bbox_pub)),
  cmd_pub_(std::move(cmd_pub)),
  twist_pub_(std::move(twist_pub)),
  trajectory_marker_pub_(std::move(trajectory_marker_pub)),
  marker_pub_(std::move(marker_pub)),
  target_frame_(config.measurement.target_frame)
{
  this->position_marker_.ns = "position";
  this->position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  this->position_marker_.pose.orientation.w = 1.0;
  this->position_marker_.scale.x = this->position_marker_.scale.y =
    this->position_marker_.scale.z = 0.1;
  this->position_marker_.color.a = 1.0;
  this->position_marker_.color.g = 1.0;

  this->linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  this->linear_v_marker_.ns = "linear_v";
  this->linear_v_marker_.scale.x = 0.03;
  this->linear_v_marker_.scale.y = 0.05;
  this->linear_v_marker_.color.a = 1.0;
  this->linear_v_marker_.color.r = 1.0;
  this->linear_v_marker_.color.g = 1.0;

  this->angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  this->angular_v_marker_.ns = "angular_v";
  this->angular_v_marker_.scale.x = 0.03;
  this->angular_v_marker_.scale.y = 0.05;
  this->angular_v_marker_.color.a = 1.0;
  this->angular_v_marker_.color.b = 1.0;
  this->angular_v_marker_.color.g = 1.0;

  this->armor_marker_.ns = "armors";
  this->armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  this->armor_marker_.scale.x = 0.03;
  this->armor_marker_.scale.z = 0.125;
  this->armor_marker_.color.a = 1.0;
  this->armor_marker_.color.r = 1.0;
}

AimResult TargetingOutputPublisher::buildAimResult(
  const Observation & input,
  const std::map<int, std::unique_ptr<Tracker>> & trackers,
  int best_tracker_id,
  const std::optional<AimDecision> & decision,
  const FireGateResult & fire_result,
  int previous_tracker_id)
{
  AimResult output;
  output.best_tracker_id = best_tracker_id;
  output.targets_msg.header.stamp = input.stamp;
  output.targets_msg.header.frame_id = this->target_frame_;
  output.targets_msg.best_target_idx = -1;

  for (const auto & kv : trackers) {
    const auto & tracker = *kv.second;
    if (tracker.tracker_state == Tracker::LOST) {
      continue;
    }

    auto_aim_interfaces::msg::Target target_msg;
    this->targeting_debug_publisher_.fillTargetMsg(target_msg, tracker, input.stamp);
    if (tracker.tracker_id == best_tracker_id) {
      output.targets_msg.best_target_idx = static_cast<int>(output.targets_msg.targets.size());
    }
    output.targets_msg.targets.push_back(target_msg);
  }

  output.target_msg.header.stamp = input.stamp;
  output.target_msg.header.frame_id = this->target_frame_;
  output.target_msg.tracking = false;
  output.target_msg.id = "";
  output.target_msg.armors_num = 0;

  const Tracker * best_tracker = this->findTracker(trackers, best_tracker_id);
  if (best_tracker != nullptr) {
    this->targeting_debug_publisher_.fillTargetMsg(output.target_msg, *best_tracker, input.stamp);
    output.tracker_info = this->targeting_debug_publisher_.makeTrackerInfo(*best_tracker);
    output.has_tracker_info = true;
  }

  output.optimal_bbox = this->targeting_debug_publisher_.selectOptimalBBox(
    input.armors,
    input.detection_msg,
    input.detection_indices,
    best_tracker);
  output.marker_array = this->targeting_debug_publisher_.buildMarkers(
    output.target_msg,
    best_tracker,
    this->position_marker_,
    this->linear_v_marker_,
    this->angular_v_marker_,
    this->armor_marker_);

  if (!decision.has_value()) {
    output.command_output = this->gimbal_command_controller_.makeHoldOutput(output.targets_msg.header);
  } else {
    const bool reset_smoothing = decision->plan.tracker_id != previous_tracker_id;
    output.command_output = this->gimbal_command_controller_.makeShotOutput(
      output.targets_msg.header,
      decision->plan,
      fire_result.fire,
      reset_smoothing);
  }

  return output;
}

const Tracker * TargetingOutputPublisher::findTracker(
  const std::map<int, std::unique_ptr<Tracker>> & trackers,
  int tracker_id) const
{
  const auto it = trackers.find(tracker_id);
  if (it == trackers.end()) {
    return nullptr;
  }
  return it->second.get();
}

void TargetingOutputPublisher::publish(const AimResult & output)
{
  this->targets_pub_->publish(output.targets_msg);
  this->target_pub_->publish(output.target_msg);
  if (output.has_tracker_info) {
    this->info_pub_->publish(output.tracker_info);
  }
  this->optimal_bbox_pub_->publish(output.optimal_bbox);
  this->marker_pub_->publish(output.marker_array);
  this->cmd_pub_->publish(output.command_output.cmd);
  this->twist_pub_->publish(output.command_output.twist);
  this->trajectory_marker_pub_->publish(output.command_output.marker);
}

}  // namespace rm_auto_aim
