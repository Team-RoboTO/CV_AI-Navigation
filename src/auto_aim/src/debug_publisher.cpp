#include "auto_aim/debug_publisher.hpp"

#include <sstream>

namespace auto_aim
{

namespace
{
const char * blockerName(uint8_t b)
{
  switch (b) {
    case 0: return "ALLOWED";
    case 1: return "NOT_TRACKING";
    case 2: return "OUT_OF_RANGE";
    case 3: return "MARGIN_NEGATIVE";
    case 4: return "OFF_AXIS";
    case 5: return "INVALID_TARGET";
    case 6: return "INVALID_BALLISTIC";
    case 7: return "SMOOTHING_LAG";
    case 8: return "STALE_MEASUREMENT";
    case 9: return "ANTI_GYRO_TIMING";
    default: return "UNKNOWN";
  }
}
}  // namespace

DebugPublisher::DebugPublisher(rclcpp::Node * node, const std::string & topic,
                               double fire_log_period_s,
                               bool publish_enabled,
                               bool histogram_log_enabled)
: node_(node),
  last_log_time_(node->now()),
  fire_log_period_s_(fire_log_period_s),
  publish_enabled_(publish_enabled),
  histogram_log_enabled_(histogram_log_enabled)
{
  if (publish_enabled_) {
    pub_ = node_->create_publisher<auto_aim::msg::AutoAimDebug>(topic, 10);
  }
}

void DebugPublisher::publish(const std_msgs::msg::Header & header, const DebugFrame & f)
{
  if (!publish_enabled_ && !histogram_log_enabled_) return;

  auto emit_histogram = [&]() {
    if (!histogram_log_enabled_) return;
    blocker_counts_[f.fire_blocker]++;
    auto now = node_->now();
    if ((now - last_log_time_).seconds() < fire_log_period_s_) return;

    std::ostringstream os;
    os << "Fire blocker histogram (last " << fire_log_period_s_ << "s):";
    uint64_t total = 0;
    for (auto & kv : blocker_counts_) total += kv.second;
    if (total > 0) {
      for (auto & kv : blocker_counts_) {
        os << " " << blockerName(kv.first) << "=" << kv.second
           << "(" << (100.0 * kv.second / total) << "%)";
      }
    } else {
      os << " (no frames)";
    }
    RCLCPP_INFO(node_->get_logger(), "%s", os.str().c_str());
    blocker_counts_.clear();
    last_log_time_ = now;
  };

  if (!publish_enabled_) {
    emit_histogram();
    return;
  }

  auto_aim::msg::AutoAimDebug m;
  m.header = header;

  m.detection_present = f.detection_present;
  m.bbox_cx = f.bbox_cx; m.bbox_cy = f.bbox_cy;
  m.bbox_w = f.bbox_w; m.bbox_h = f.bbox_h;
  m.detect_confidence = f.detect_confidence;
  m.class_id = f.class_id;
  m.raw_kp_detection_count = f.raw_kp_detection_count;
  m.class_reject_count = f.class_reject_count;
  m.kp_low_score_reject_count = f.kp_low_score_reject_count;
  m.kp_geometry_reject_count = f.kp_geometry_reject_count;
  m.pnp_failed_count = f.pnp_failed_count;
  m.pose_z_reject_count = f.pose_z_reject_count;
  m.pose_range_reject_count = f.pose_range_reject_count;
  m.armors_passed_to_tracker_count = f.armors_passed_to_tracker_count;

  m.pnp_ok = f.pnp_ok;
  m.pnp_is_large = f.pnp_is_large;
  m.pnp_reproj_err = f.pnp_reproj_err;
  m.pnp_reproj_err_norm = f.pnp_reproj_err_norm;
  m.pnp_small_reproj_err = f.pnp_small_reproj_err;
  m.pnp_large_reproj_err = f.pnp_large_reproj_err;
  m.pnp_size_margin = f.pnp_size_margin;
  for (size_t i = 0; i < 8; ++i) m.pnp_image_points[i] = f.pnp_image_points[i];
  m.pnp_tvec_x = f.pnp_tvec_x;
  m.pnp_tvec_y = f.pnp_tvec_y;
  m.pnp_tvec_z = f.pnp_tvec_z;
  m.pnp_reject_reason = f.pnp_reject_reason;

  m.meas_source = f.meas_source;
  for (size_t i = 0; i < 8; ++i) m.kp_image_points[i] = f.kp_image_points[i];
  for (size_t i = 0; i < 4; ++i) m.kp_scores[i] = f.kp_scores[i];
  m.kp_min_score = f.kp_min_score;
  m.kp_mean_score = f.kp_mean_score;
  m.kp_geometry_valid = f.kp_geometry_valid;
  m.kp_reject_reason = f.kp_reject_reason;

  m.odom_x = f.odom_x; m.odom_y = f.odom_y; m.odom_z = f.odom_z;
  m.odom_yaw = f.odom_yaw;

  m.tracker_state = f.tracker_state;
  m.tracker_target_id = f.tracker_target_id;
  m.tracker_assigned_count = f.tracker_assigned_count;
  m.tracker_association_reject_count = f.tracker_association_reject_count;
  m.tracker_miss_reason = f.tracker_miss_reason;
  for (size_t i = 0; i < 9; ++i) m.ekf_state[i] = f.ekf_state[i];
  m.ekf_innovation_norm     = f.ekf_innovation_norm;
  m.ekf_innovation_pos_norm = f.ekf_innovation_pos_norm;
  m.ekf_innovation_yaw_abs  = f.ekf_innovation_yaw_abs;
  m.ekf_mahalanobis     = f.ekf_mahalanobis;
  m.ekf_q_pos_eff       = f.ekf_q_pos_eff;
  m.ekf_q_yaw_eff       = f.ekf_q_yaw_eff;
  m.ekf_r_pos_eff       = f.ekf_r_pos_eff;
  m.ekf_r_yaw_eff       = f.ekf_r_yaw_eff;
  m.ekf_pos_sigma       = f.ekf_pos_sigma;
  m.ekf_yaw_sigma       = f.ekf_yaw_sigma;
  m.ekf_match_count     = f.ekf_match_count;
  m.ekf_miss_count      = f.ekf_miss_count;
  m.ekf_measurement_age_s = f.ekf_measurement_age_s;
  m.ekf_measurement_quality = f.ekf_measurement_quality;
  m.best_match_mahalanobis = f.best_match_mahalanobis;
  m.best_match_position_diff = f.best_match_position_diff;
  m.best_match_yaw_diff = f.best_match_yaw_diff;
  m.match_reject_reason = f.match_reject_reason;

  m.aim_target_valid = f.aim_target_valid;
  m.aim_face_index = f.aim_face_index;
  m.aim_abs_yaw = f.aim_abs_yaw;
  m.aim_abs_pitch = f.aim_abs_pitch;
  m.aim_distance = f.aim_distance;
  m.aim_target_x = f.aim_target_x;
  m.aim_target_y = f.aim_target_y;
  m.aim_target_z = f.aim_target_z;
  m.aim_flight_time = f.aim_flight_time;
  m.aim_pred_t = f.aim_pred_t;

  m.cmd_yaw_pre_smooth = f.cmd_yaw_pre_smooth;
  m.cmd_pitch_pre_smooth = f.cmd_pitch_pre_smooth;
  m.cmd_yaw_published = f.cmd_yaw_published;
  m.cmd_pitch_published = f.cmd_pitch_published;
  m.cmd_fire = f.cmd_fire;

  m.fire_blocker = f.fire_blocker;
  m.fire_blocker_reason = f.fire_blocker_reason;

  m.latency_capture_to_process_s = f.latency_capture_to_process_s;
  m.latency_process_s = f.latency_process_s;
  m.latency_process_to_publish_s = f.latency_process_to_publish_s;
  m.latency_total_s = f.latency_total_s;
  m.latency_estimate_ema_s = f.latency_estimate_ema_s;

  m.pred_lead_measured_s   = f.pred_lead_measured_s;
  m.pred_lead_gimbal_s     = f.pred_lead_gimbal_s;
  m.pred_lead_time_bias_s  = f.pred_lead_time_bias_s;
  m.pred_lead_ema_s        = f.pred_lead_ema_s;
  m.pred_flight_time_s     = f.pred_flight_time_s;
  m.pred_t_total_s         = f.pred_t_total_s;

  m.smoothing_lag_rad      = f.smoothing_lag_rad;

  pub_->publish(m);

  // accumulate blocker histogram and emit periodically.
  emit_histogram();
}

}  // namespace auto_aim
