#ifndef AUTO_AIM__DEBUG_FRAME_HPP_
#define AUTO_AIM__DEBUG_FRAME_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace auto_aim
{

// Plain-old-data struct that gets filled stage-by-stage during a detection
// callback. The DebugPublisher copies it into the AutoAimDebug ROS message
// at the end of the callback. Adding a new field here is cheap; consumers
// simply ignore unknown fields.
struct DebugFrame
{
  // Detection
  bool detection_present = false;
  float bbox_cx = 0, bbox_cy = 0, bbox_w = 0, bbox_h = 0;
  float detect_confidence = 0;
  std::string class_id;
  uint32_t raw_kp_detection_count = 0;
  uint32_t class_reject_count = 0;
  uint32_t kp_low_score_reject_count = 0;
  uint32_t kp_geometry_reject_count = 0;
  uint32_t pnp_failed_count = 0;
  uint32_t pose_z_reject_count = 0;
  uint32_t pose_range_reject_count = 0;
  uint32_t armors_passed_to_tracker_count = 0;

  // PnP
  bool pnp_ok = false;
  bool pnp_is_large = false;
  bool pnp_size_hysteresis_kept = false;
  float pnp_reproj_err = 0;       // mean pixel error
  float pnp_reproj_err_norm = 0;  // / keypoint AABB diagonal
  float pnp_small_reproj_err = 0;
  float pnp_large_reproj_err = 0;
  float pnp_size_margin = 0;
  std::array<float, 8> pnp_image_points{};  // post-check corners fed to PnP
  float pnp_tvec_x = 0, pnp_tvec_y = 0, pnp_tvec_z = 0;
  // 0=OK 1=DEGEN_BBOX 2=NONFINITE 3=BAD_LIGHT_RATIO 4=SOLVER_FAIL
  // 5=REPROJ_ABS 6=REPROJ_NORM 7=DEPTH_TOO_CLOSE 8=DEPTH_TOO_FAR
  uint8_t pnp_reject_reason = 0;

  // YOLO-pose keypoint diagnostics. The pipeline currently has only one
  // measurement model (keypoints); the source flag exists so a future bbox
  // fallback would be visible without changing the message schema.
  uint8_t meas_source = 0;        // 0=NONE 1=KEYPOINT
  std::array<float, 8> kp_image_points{};   // raw model-order TL,TR,BR,BL
  std::array<float, 4> kp_scores{};
  float   kp_min_score = 0;
  float   kp_mean_score = 0;
  bool    kp_geometry_valid = false;
  uint8_t kp_reject_reason = 0;   // 0=OK 1=NONFINITE 2=LOW_SCORE 3=INVALID_GEOMETRY 4=OUT_OF_IMAGE

  // Frame transform
  std::string pose_source = "micro_imu";
  bool pose_present = false;
  bool pose_fresh = false;
  float pose_age_s = 0;
  float imu_yaw = 0;
  float imu_pitch = 0;
  float odom_x = 0, odom_y = 0, odom_z = 0, odom_yaw = 0;

  // EKF
  uint8_t tracker_state = 0;
  std::string tracker_target_id;
  uint32_t tracker_assigned_count = 0;
  uint32_t tracker_association_reject_count = 0;
  std::string tracker_miss_reason;
  std::array<float, 9> ekf_state{};
  float ekf_innovation_norm = 0;       // legacy, mixes meters and radians
  float ekf_innovation_pos_norm = 0;   // sqrt(yx^2 + yy^2 + yz^2) [m]
  float ekf_innovation_yaw_abs = 0;    // |y_yaw| [rad]
  float ekf_mahalanobis = 0;
  float ekf_q_pos_eff = 0;             // effective q_pos used by ekfPredict
  float ekf_q_yaw_eff = 0;             // effective q_yaw used by ekfPredict
  float ekf_r_pos_eff = 0;             // effective sqrt(R_pos) at last update [m]
  float ekf_r_yaw_eff = 0;             // effective sqrt(R_yaw) at last update [rad]
  float ekf_pos_sigma = 0;             // sqrt(P(0,0)+P(2,2)+P(4,4)) [m]
  float ekf_yaw_sigma = 0;             // sqrt(P(6,6)) [rad]
  uint32_t ekf_match_count = 0;        // consecutive matched frames
  uint32_t ekf_miss_count = 0;         // consecutive missed frames
  float ekf_measurement_age_s = 0;     // s since last accepted measurement
  // 0=NONE 1=ACCEPTED 2=DEGRADED 3=REJECTED
  uint8_t ekf_measurement_quality = 0;
  float best_match_mahalanobis = 0;
  float best_match_position_diff = 0;
  float best_match_yaw_diff = 0;
  std::string match_reject_reason;

  // Aim
  bool aim_target_valid = false;
  int aim_face_index = -1;
  float aim_abs_yaw = 0;
  float aim_abs_pitch = 0;
  float aim_distance = 0;
  float aim_target_x = 0, aim_target_y = 0, aim_target_z = 0;
  float aim_flight_time = 0;
  float aim_pred_t = 0;
  float aim_rel_yaw = 0;
  float aim_rel_pitch = 0;
  float aim_fire_margin = 0;
  bool aim_anti_gyro_active = false;
  float aim_anti_gyro_residual = 0;
  float aim_cam_yaw = 0;
  float aim_cam_pitch = 0;
  float aim_cam_total_angle = 0;

  // Command
  float cmd_yaw_pre_smooth = 0;
  float cmd_pitch_pre_smooth = 0;
  float cmd_yaw_published = 0;
  float cmd_pitch_published = 0;
  bool cmd_fire = false;
  bool cmd_published = false;
  bool cmd_hold_active = false;
  bool cmd_coast_active = false;
  bool tracker_fresh_enough_for_command = false;
  float coast_age_s = 0;

  // Fire gate
  uint8_t fire_blocker = 0;
  uint32_t fire_blocker_mask = 0;
  std::string fire_blockers_active;
  std::string fire_blocker_reason;
  std::string fire_alignment_source;
  float fire_alignment_error = 0;

  // Latency (P7+)
  double latency_capture_to_process_s = 0;
  double latency_process_s = 0;
  double latency_process_to_publish_s = 0;
  double latency_total_s = 0;
  double latency_estimate_ema_s = 0;     // EMA-smoothed total latency

  // Prediction lead components (planner used pred_t = sum)
  double pred_lead_measured_s = 0;       // EMA latency contribution (else 0)
  double pred_lead_gimbal_s = 0;         // gimbal_response_delay used (else 0)
  double pred_lead_time_bias_s = 0;      // cfg.time_bias contribution (else 0)
  double pred_lead_ema_s = 0;            // EMA-smoothing delay contribution
  double pred_flight_time_s = 0;         // ballistic flight time
  double pred_t_total_s = 0;             // final pred_t passed to face loop

  // Smoothing diagnostic (always logged; gate is opt-in)
  float smoothing_lag_rad = 0;
};

}  // namespace auto_aim
#endif
