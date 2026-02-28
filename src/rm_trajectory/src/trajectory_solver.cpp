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
  angular_window_              = this->declare_parameter("angular_window",               0.09);
  accel_ema_alpha_ = this->declare_parameter("accel_ema_alpha", 0.3);
  max_accel_       = this->declare_parameter("max_accel",       6.0);
  latency_gate_sigma_ = this->declare_parameter("latency_gate_sigma", 2.5);

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
      "/cmd_vel", rclcpp::SystemDefaultsQoS());

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
  if (!msg->tracking) {
    // Publish safe command: fire off, zero angles
    auto_aim_interfaces::msg::GimbalCmd cmd;
    cmd.header = msg->header;
    cmd.pitch    = 0.0;
    cmd.yaw      = 0.0;
    cmd.distance = 0.0;
    cmd.fire_cmd = false;
    cmd_pub_->publish(cmd);

    has_prev_target_ = false;
    ax_ema_ = 0.0; ay_ema_ = 0.0; az_ema_ = 0.0;

    geometry_msgs::msg::Twist twist;
    twist.angular.x = 0.0;  // fire off
    twist_pub_->publish(twist);
    return;
  }

  // Suppress fire during TEMP_LOST — EKF is coasting without measurements
  bool temp_lost = (msg->tracker_state == auto_aim_interfaces::msg::Target::TEMP_LOST);

  // H4: Adaptive latency compensation — EMA with outlier rejection
  double measured_latency = (this->now() - msg->header.stamp).seconds();
  if (measured_latency > 0.0 && measured_latency < 0.5) {
    double residual = measured_latency - time_bias_;
    double sigma = std::sqrt(std::max(time_bias_var_, 1e-8));
    // Reject outlier spikes (GPU thermal throttle, USB stall)
    if (std::abs(residual) < latency_gate_sigma_ * sigma) {
      time_bias_ = time_bias_alpha_ * measured_latency
                   + (1.0 - time_bias_alpha_) * time_bias_;
    }
    // Always update variance (tracks spread even during rejection)
    time_bias_var_ = time_bias_alpha_ * (residual * residual)
                     + (1.0 - time_bias_alpha_) * time_bias_var_;
  }

  double xc = msg->position.x, yc = msg->position.y, za = msg->position.z;
  double vx = msg->velocity.x, vy = msg->velocity.y, vz = msg->velocity.z;

  // Acceleration estimation (EMA over velocity differences)
  double ax = 0.0, ay = 0.0, az = 0.0;
  if (has_prev_target_) {
    double dt_tgt = (rclcpp::Time(msg->header.stamp) - prev_target_time_).seconds();
    if (dt_tgt > 0.005 && dt_tgt < 0.2) {
      auto clamp = [this](double val) {
        return std::max(-max_accel_, std::min(val, max_accel_));
      };
      // Time-adjusted alpha: consistent bandwidth regardless of frame rate
      double alpha_dt = 1.0 - std::pow(1.0 - accel_ema_alpha_, dt_tgt / (1.0 / 30.0));
      ax_ema_ = alpha_dt * clamp((vx - prev_vx_) / dt_tgt)
                + (1.0 - alpha_dt) * ax_ema_;
      ay_ema_ = alpha_dt * clamp((vy - prev_vy_) / dt_tgt)
                + (1.0 - alpha_dt) * ay_ema_;
      az_ema_ = alpha_dt * clamp((vz - prev_vz_) / dt_tgt)
                + (1.0 - alpha_dt) * az_ema_;
    }
  }
  ax = ax_ema_; ay = ay_ema_; az = az_ema_;
  has_prev_target_ = true;
  prev_target_time_ = msg->header.stamp;
  prev_vx_ = vx; prev_vy_ = vy; prev_vz_ = vz;

  // --- Pass 1: estimate flight time from center position ---
  double dist_center = std::sqrt(xc * xc + yc * yc + za * za);
  if (bullet_speed_ < 1e-3) {
    RCLCPP_WARN(this->get_logger(), "bullet_speed is near zero, skipping");
    return;
  }
  double t0 = dist_center / bullet_speed_;
  double pt = t0 + time_bias_;

  // --- Pass 2: find best-facing armor face, verify with actual flight time ---
  int n_faces = std::max(msg->armors_num, 1);
  double face_spacing = 2.0 * M_PI / n_faces;

  auto findBestFace = [&](double predict_time) -> int {
    double pred_yaw = msg->yaw + msg->v_yaw * predict_time;
    double pred_cx = xc + vx * predict_time + 0.5 * ax * predict_time * predict_time;
    double pred_cy = yc + vy * predict_time + 0.5 * ay * predict_time * predict_time;
    double bear = std::atan2(pred_cy, pred_cx);
    double best_d = M_PI;
    int best_f = 0;
    for (int i = 0; i < n_faces; i++) {
      double fy = pred_yaw + i * face_spacing;
      double d = std::abs(angles::shortest_angular_distance(bear, fy));
      if (d < best_d) { best_d = d; best_f = i; }
    }
    return best_f;
  };

  int best_face = findBestFace(pt);
  double pitch = 0.0, t_flight = 0.0;
  double tx = 0.0, ty = 0.0, tz = 0.0;
  double ground_dist = 0.0, yaw = 0.0;

  for (int pass = 0; pass < 2; pass++) {
    double total_t = (pass == 0) ? pt : (t_flight + time_bias_);
    double yaw_at = msg->yaw + msg->v_yaw * total_t;
    double face_yaw = yaw_at + best_face * face_spacing;
    bool is_current_pair = (best_face % 2 == 0);
    double r = is_current_pair ? msg->radius_1 : msg->radius_2;
    double dz_offset = (n_faces == 4 && !is_current_pair) ? msg->dz : 0.0;

    double pcx = xc + vx * total_t + 0.5 * ax * total_t * total_t;
    double pcy = yc + vy * total_t + 0.5 * ay * total_t * total_t;
    double pcz = za + vz * total_t + 0.5 * az * total_t * total_t;

    tx = pcx - r * std::cos(face_yaw);
    ty = pcy - r * std::sin(face_yaw);
    tz = pcz + dz_offset;

    ground_dist = std::sqrt(tx * tx + ty * ty);
    yaw = std::atan2(ty, tx);

    auto result = solveTrajectory(ground_dist, tz, bullet_speed_);
    pitch = result.first;
    t_flight = result.second;

    if (pass == 0) {
      int recheck_face = findBestFace(t_flight + time_bias_);
      if (recheck_face == best_face) break;  // Face consistent, done
      best_face = recheck_face;
    }
  }

  // Guard: solveTrajectory returns (0,0) when ground_dist ≈ 0
  if (t_flight < 1e-6) {
    auto_aim_interfaces::msg::GimbalCmd cmd;
    cmd.header = msg->header;
    cmd.pitch    = 0.0;
    cmd.yaw      = 0.0;
    cmd.distance = 0.0;
    cmd.fire_cmd = false;
    cmd_pub_->publish(cmd);
    return;
  }

  double range = std::sqrt(tx * tx + ty * ty + tz * tz);
  bool dist_ok = (range >= min_fire_dist_) && (range <= max_fire_dist_);

  // Fire gate: unified check for all spin rates
  bool in_range = false;
  if (dist_ok && !temp_lost) {
    double final_yaw = msg->yaw + msg->v_yaw * (t_flight + time_bias_);

    // Find closest face alignment at impact time
    double min_face_diff = M_PI;
    for (int i = 0; i < n_faces; i++) {
      double fy = final_yaw + i * face_spacing;
      double diff = std::abs(angles::shortest_angular_distance(yaw, fy));
      if (diff < min_face_diff) min_face_diff = diff;
    }

    // Variance-based window, capped to physical limits
    double sigma_vyaw = std::sqrt(std::max(msg->v_yaw_variance, 1e-6));
    double yaw_uncertainty = sigma_vyaw * (t_flight + time_bias_);
    double max_window = face_spacing * 0.5;  // Can't exceed half the face spacing
    double effective_window = std::min(angular_window_ + yaw_uncertainty, max_window);

    in_range = (min_face_diff < effective_window);
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
