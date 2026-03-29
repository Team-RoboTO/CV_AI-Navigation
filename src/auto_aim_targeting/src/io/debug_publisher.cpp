// ============================================================================
// debug_publisher.cpp — Builds debug/visualization messages from tracker state.
//
// ENTRYPOINTS:
//   fillTargetMsg     → populate a Target message from a Tracker
//   makeTrackerInfo   → build TrackerInfo diagnostic message
//   selectOptimalBBox → find the YOLO bbox matching the best tracker
//   buildMarkers      → create RViz markers for position, velocity, armors
// ============================================================================
#include "auto_aim_targeting/io/debug_publisher.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace rm_auto_aim
{

void TargetingDebugPublisher::fillTargetMsg(
  auto_aim_interfaces::msg::Target & msg,
  const Tracker & tracker,
  const rclcpp::Time & time) const
{
  msg.header.stamp = time;
  msg.header.frame_id = this->target_frame_;
  msg.tracker_id = tracker.tracker_id;
  msg.tracker_state = static_cast<uint8_t>(tracker.tracker_state);

  if (tracker.tracker_state == Tracker::DETECTING) {
    msg.tracking = false;
    msg.id = "";
    msg.armors_num = 0;
    return;
  }

  if (tracker.tracker_state != Tracker::TRACKING &&
      tracker.tracker_state != Tracker::TEMP_LOST)
  {
    msg.tracking = false;
    msg.id = "";
    msg.armors_num = 0;
    return;
  }

  msg.tracking = true;
  const auto & state = tracker.target_state;
  msg.id = tracker.tracked_id;
  msg.armors_num = static_cast<int>(tracker.tracked_armors_num);
  msg.position.x = state(0);
  msg.velocity.x = state(1);
  msg.position.y = state(2);
  msg.velocity.y = state(3);
  msg.position.z = state(4);
  msg.velocity.z = state(5);
  msg.yaw = state(6);
  msg.v_yaw = state(7);
  msg.radius_1 = state(8);
  msg.radius_2 = tracker.another_r;
  msg.dz = tracker.dz;
  msg.v_yaw_variance = tracker.ekf.getVariance(7);
  msg.position_variance_x = tracker.ekf.getVariance(0);
  msg.position_variance_y = tracker.ekf.getVariance(2);
  msg.position_variance_z = tracker.ekf.getVariance(4);

  const auto accel = tracker.smoothedAcceleration();
  msg.acceleration.x = accel(0);
  msg.acceleration.y = accel(1);
  msg.acceleration.z = accel(2);
  msg.last_measurement_stamp = tracker.lastMeasurementTime();
}

auto_aim_interfaces::msg::TrackerInfo TargetingDebugPublisher::makeTrackerInfo(const Tracker & tracker) const
{
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  info_msg.position_diff = tracker.position_error_norm;
  info_msg.yaw_diff = tracker.yaw_error_abs;
  info_msg.face_angle = tracker.face_obliquity_angle;
  if (tracker.measurement_valid) {
    info_msg.position.x = tracker.measurement(0);
    info_msg.position.y = tracker.measurement(1);
    info_msg.position.z = tracker.measurement(2);
    info_msg.yaw = tracker.measurement(3);
  }
  info_msg.xyz_damping_alpha = tracker.position_velocity_decay;
  info_msg.yaw_damping_alpha = tracker.yaw_velocity_decay;
  return info_msg;
}

vision_msgs::msg::Detection2D TargetingDebugPublisher::selectOptimalBBox(
  const auto_aim_interfaces::msg::Armors::SharedPtr & armors,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg,
  const std::vector<size_t> & detection_indices,
  const Tracker * best_tracker) const
{
  vision_msgs::msg::Detection2D optimal_bbox;
  optimal_bbox.header = detection_msg->header;
  if (best_tracker == nullptr) {
    return optimal_bbox;
  }

  size_t matched_idx = static_cast<size_t>(-1);
  double min_dist = 0.2;
  for (size_t i = 0; i < armors->armors.size(); i++) {
    const auto & armor = armors->armors[i];
    double dx = armor.pose.position.x - best_tracker->tracked_armor.pose.position.x;
    double dy = armor.pose.position.y - best_tracker->tracked_armor.pose.position.y;
    double dz = armor.pose.position.z - best_tracker->tracked_armor.pose.position.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < min_dist) {
      min_dist = dist;
      matched_idx = i;
    }
  }

  if (matched_idx != static_cast<size_t>(-1) && matched_idx < detection_indices.size()) {
    size_t original_detection_idx = detection_indices[matched_idx];
    if (original_detection_idx < detection_msg->detections.size()) {
      optimal_bbox = detection_msg->detections[original_detection_idx];
    }
  }

  return optimal_bbox;
}

visualization_msgs::msg::MarkerArray TargetingDebugPublisher::buildMarkers(
  const auto_aim_interfaces::msg::Target & target_msg,
  const Tracker * best_tracker,
  const visualization_msgs::msg::Marker & position_template,
  const visualization_msgs::msg::Marker & linear_v_template,
  const visualization_msgs::msg::Marker & angular_v_template,
  const visualization_msgs::msg::Marker & armor_template) const
{
  auto position_marker = position_template;
  auto linear_v_marker = linear_v_template;
  auto angular_v_marker = angular_v_template;
  auto armor_marker = armor_template;

  position_marker.header = target_msg.header;
  linear_v_marker.header = target_msg.header;
  angular_v_marker.header = target_msg.header;
  armor_marker.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;
  if (target_msg.tracking) {
    this->buildTrackingMarkers(
      target_msg, best_tracker,
      position_marker, linear_v_marker, angular_v_marker, armor_marker,
      marker_array);
  } else {
    this->buildDeleteMarkers(
      position_marker, linear_v_marker, angular_v_marker, armor_marker,
      marker_array);
  }

  marker_array.markers.emplace_back(position_marker);
  marker_array.markers.emplace_back(linear_v_marker);
  marker_array.markers.emplace_back(angular_v_marker);
  return marker_array;
}

void TargetingDebugPublisher::buildTrackingMarkers(
  const auto_aim_interfaces::msg::Target & target_msg,
  const Tracker * best_tracker,
  visualization_msgs::msg::Marker & position_marker,
  visualization_msgs::msg::Marker & linear_v_marker,
  visualization_msgs::msg::Marker & angular_v_marker,
  visualization_msgs::msg::Marker & armor_marker,
  visualization_msgs::msg::MarkerArray & marker_array) const
{
  double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
  double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
  double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
  double dz = target_msg.dz;

  position_marker.action = visualization_msgs::msg::Marker::ADD;
  position_marker.pose.position.x = xc;
  position_marker.pose.position.y = yc;
  position_marker.pose.position.z = za + dz / 2;

  linear_v_marker.action = visualization_msgs::msg::Marker::ADD;
  linear_v_marker.points.clear();
  linear_v_marker.points.emplace_back(position_marker.pose.position);
  geometry_msgs::msg::Point arrow_end = position_marker.pose.position;
  arrow_end.x += vx;
  arrow_end.y += vy;
  arrow_end.z += vz;
  linear_v_marker.points.emplace_back(arrow_end);

  angular_v_marker.action = visualization_msgs::msg::Marker::ADD;
  angular_v_marker.points.clear();
  angular_v_marker.points.emplace_back(position_marker.pose.position);
  arrow_end = position_marker.pose.position;
  arrow_end.z += target_msg.v_yaw / M_PI;
  angular_v_marker.points.emplace_back(arrow_end);

  armor_marker.action = visualization_msgs::msg::Marker::ADD;
  const bool is_small = best_tracker != nullptr && best_tracker->tracked_armor.type == "small";
  armor_marker.scale.y = is_small ? 0.140 : 0.235;
  armor_marker.scale.z = is_small ? 0.125 : 0.127;
  bool is_current_pair = true;
  size_t a_n = target_msg.armors_num;
  geometry_msgs::msg::Point p_a;
  double r = 0.0;
  for (size_t i = 0; i < a_n; i++) {
    double tmp_yaw = yaw + i * (2 * M_PI / a_n);
    if (a_n == 4) {
      r = is_current_pair ? r1 : r2;
      p_a.z = za + (is_current_pair ? 0 : dz);
      is_current_pair = !is_current_pair;
    } else {
      r = r1;
      p_a.z = za;
    }
    p_a.x = xc - r * std::cos(tmp_yaw);
    p_a.y = yc - r * std::sin(tmp_yaw);

    armor_marker.id = static_cast<int>(i);
    armor_marker.pose.position = p_a;
    tf2::Quaternion q;
    q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
    armor_marker.pose.orientation = tf2::toMsg(q);
    marker_array.markers.emplace_back(armor_marker);
  }
}

void TargetingDebugPublisher::buildDeleteMarkers(
  visualization_msgs::msg::Marker & position_marker,
  visualization_msgs::msg::Marker & linear_v_marker,
  visualization_msgs::msg::Marker & angular_v_marker,
  visualization_msgs::msg::Marker & armor_marker,
  visualization_msgs::msg::MarkerArray & marker_array) const
{
  position_marker.action = visualization_msgs::msg::Marker::DELETE;
  linear_v_marker.action = visualization_msgs::msg::Marker::DELETE;
  angular_v_marker.action = visualization_msgs::msg::Marker::DELETE;

  armor_marker.action = visualization_msgs::msg::Marker::DELETE;
  for (int i = 0; i < 4; i++) {
    armor_marker.id = i;
    marker_array.markers.emplace_back(armor_marker);
  }
}

}  // namespace rm_auto_aim
