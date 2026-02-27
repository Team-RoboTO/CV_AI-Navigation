#include "rm_trajectory/trajectory_solver.hpp"

#include <angles/angles.h>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace rm_auto_aim {
TrajectorySolverNode::TrajectorySolverNode(const rclcpp::NodeOptions &options)
    : Node("trajectory_solver", options) {
  RCLCPP_INFO(this->get_logger(), "Starting TrajectorySolverNode!");

  bullet_speed_    = this->declare_parameter("bullet_speed",    25.0);
  gravity_         = this->declare_parameter("gravity",          9.8);
  k_               = this->declare_parameter("k",                0.01);
  time_bias_       = this->declare_parameter("time_bias",        0.08);
  time_bias_alpha_ = this->declare_parameter("time_bias_alpha",  0.35);
  min_fire_dist_   = this->declare_parameter("min_fire_dist",    0.5);
  max_fire_dist_   = this->declare_parameter("max_fire_dist",   10.0);
  max_spin_rate_               = this->declare_parameter("max_spin_rate",                4.0);
  min_spin_rate_for_predictor_ = this->declare_parameter("min_spin_rate_for_predictor", 1.5);
  angular_window_              = this->declare_parameter("angular_window",               0.09);

  // Match RELIABLE QoS used by armor_tracker's target publisher
  auto target_qos = rclcpp::SensorDataQoS()
      .reliability(rclcpp::ReliabilityPolicy::Reliable);
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
      "/tracker/target", target_qos,
      std::bind(&TrajectorySolverNode::targetCallback, this,
                std::placeholders::_1));

  cmd_pub_ = this->create_publisher<auto_aim_interfaces::msg::GimbalCmd>(
      "/tracker/cmd_gimbal", rclcpp::SensorDataQoS());

  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS());

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/trajectory/marker", 10);
}

std::pair<double, double> TrajectorySolverNode::solveTrajectory(
    const double ground_dist, const double target_z, const double v)
{
  // Input validation
  if (ground_dist < 1e-3 || v < 1e-3) {
    return {0.0, 0.0};
  }

  double t = ground_dist / v;
  double pitch = 0.0;
  const int max_iter = 10;

  for (int i = 0; i < max_iter; i++) {
    // Gravity drop the bullet must overcome
    double dz = 0.5 * gravity_ * t * t;
    pitch = std::atan2(target_z + dz, ground_dist);

    // Clamp pitch to prevent division-by-zero in cos(pitch) (~80 degrees)
    if (pitch > 1.4) pitch = 1.4;
    else if (pitch < -1.4) pitch = -1.4;

    // Path length along the barrel direction
    double cos_pitch = std::cos(pitch);
    double path_len = ground_dist / cos_pitch;

    // Effective bullet speed accounting for linear drag (a = -k*v)
    // v_eff = v0 * (1 - exp(-k*t)) / (k*t)  [mean speed over flight]
    double v_eff;
    double kt = k_ * t;
    if (kt > 1e-6) {
      v_eff = v * (1.0 - std::exp(-kt)) / kt;
    } else {
      v_eff = v;
    }

    double new_t = path_len / v_eff;
    if (std::abs(new_t - t) < 1e-4) {
      t = new_t;
      break;
    }
    t = new_t;
  }

  // Final validity check
  if (!std::isfinite(pitch) || !std::isfinite(t)) {
    RCLCPP_WARN(this->get_logger(), "Trajectory solver produced non-finite result");
    return {0.0, 0.0};
  }

  return {pitch, t};
}

void TrajectorySolverNode::targetCallback(
    const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  if (!msg->tracking) return;

  // Suppress fire during TEMP_LOST — EKF is coasting without measurements
  bool temp_lost = (msg->tracker_state == auto_aim_interfaces::msg::Target::TEMP_LOST);

  // H4: Adaptive latency compensation — EMA over measured pipeline delay
  double measured_latency = (this->now() - msg->header.stamp).seconds();
  if (measured_latency > 0.0 && measured_latency < 0.5) {
    time_bias_ = time_bias_alpha_ * measured_latency
                 + (1.0 - time_bias_alpha_) * time_bias_;
  }

  double xc = msg->position.x, yc = msg->position.y, za = msg->position.z;
  double vx = msg->velocity.x, vy = msg->velocity.y, vz = msg->velocity.z;

  // --- Pass 1: estimate flight time from center position ---
  double dist_center = std::sqrt(xc * xc + yc * yc);
  double t0 = dist_center / bullet_speed_;
  double pt = t0 + time_bias_;

  // Predict center position at impact
  double cx = xc + vx * pt;
  double cy = yc + vy * pt;
  double cz = za + vz * pt;

  // Predict yaw at impact
  double yaw_at_impact = msg->yaw + msg->v_yaw * pt;

  // --- Pass 2: find the best-facing armor face and aim at it ---
  int n_faces = std::max(msg->armors_num, 1);
  double face_spacing = 2.0 * M_PI / n_faces;
  double bearing = std::atan2(cy, cx);

  double best_diff = M_PI;
  int best_face = 0;
  for (int i = 0; i < n_faces; i++) {
    double face_yaw = yaw_at_impact + i * face_spacing;
    double diff = std::abs(angles::shortest_angular_distance(bearing, face_yaw));
    if (diff < best_diff) {
      best_diff = diff;
      best_face = i;
    }
  }

  // Compute face position: xa = xc - r * cos(yaw), ya = yc - r * sin(yaw)
  double face_yaw = yaw_at_impact + best_face * face_spacing;
  bool is_current_pair = (best_face % 2 == 0);
  double r = is_current_pair ? msg->radius_1 : msg->radius_2;
  double dz_offset = (n_faces == 4 && !is_current_pair) ? msg->dz : 0.0;

  double tx = cx - r * std::cos(face_yaw);
  double ty = cy - r * std::sin(face_yaw);
  double tz = cz + dz_offset;

  double ground_dist = std::sqrt(tx * tx + ty * ty);
  double yaw = std::atan2(ty, tx);

  auto result = solveTrajectory(ground_dist, tz, bullet_speed_);
  double pitch = result.first;

  double range = std::sqrt(tx * tx + ty * ty + tz * tz);
  bool dist_ok = (range >= min_fire_dist_) && (range <= max_fire_dist_);

  // Fire gate: check if armor face is within angular window
  bool in_range = false;
  if (dist_ok && !temp_lost) {
    double abs_v_yaw = std::abs(msg->v_yaw);
    if (abs_v_yaw < min_spin_rate_for_predictor_) {
      // Slow / stopped target: original hard gate
      in_range = (abs_v_yaw < max_spin_rate_);
    } else {
      // Fast-spinning target: use best_diff from face selection above
      // Re-compute with refined flight time from face-aimed trajectory
      double t_flight = result.second;
      double refined_yaw_at_impact = msg->yaw + msg->v_yaw * t_flight;
      double min_diff = M_PI;
      for (int i = 0; i < n_faces; i++) {
        double fy = refined_yaw_at_impact + i * face_spacing;
        double diff = std::abs(angles::shortest_angular_distance(yaw, fy));
        if (diff < min_diff) min_diff = diff;
      }

      // Scale angular window by v_yaw confidence (SNR gate)
      double sigma_vyaw = std::sqrt(std::max(msg->v_yaw_variance, 1e-6));
      double yaw_uncertainty = sigma_vyaw * t_flight;
      double effective_window = angular_window_ + yaw_uncertainty;
      in_range = (min_diff < effective_window);
    }
  }

  auto_aim_interfaces::msg::GimbalCmd cmd;
  cmd.header = msg->header;
  cmd.pitch    = pitch;
  cmd.yaw      = yaw;
  cmd.distance = range;
  cmd.fire_cmd = in_range;
  cmd_pub_->publish(cmd);

  geometry_msgs::msg::Twist twist;
  twist.angular.x = in_range ? 1.0 : 0.0;
  twist.angular.y = pitch * 180.0 / M_PI;
  twist.angular.z = yaw   * 180.0 / M_PI;
  twist_pub_->publish(twist);

  // Visualise predicted impact point
  visualization_msgs::msg::Marker marker;
  marker.header = msg->header;
  marker.ns = "impact_point";
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = tx;
  marker.pose.position.y = ty;
  marker.pose.position.z = tz;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
  marker.color.a = 1.0; marker.color.g = 1.0;
  marker_pub_->publish(marker);
}

} // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::TrajectorySolverNode);
