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

  // PnP
  bool pnp_ok = false;
  bool pnp_is_large = false;
  float pnp_reproj_err = 0;       // mean pixel error
  float pnp_reproj_err_norm = 0;  // / bbox diagonal
  std::array<float, 8> pnp_image_points{};  // x0 y0 x1 y1 x2 y2 x3 y3
  float pnp_tvec_x = 0, pnp_tvec_y = 0, pnp_tvec_z = 0;
  // 0=OK 1=DEGEN_BBOX 2=NONFINITE 3=BAD_LIGHT_RATIO 4=SOLVER_FAIL
  // 5=REPROJ_ABS 6=REPROJ_NORM 7=DEPTH_TOO_CLOSE 8=DEPTH_TOO_FAR
  uint8_t pnp_reject_reason = 0;

  // Frame transform
  float odom_x = 0, odom_y = 0, odom_z = 0, odom_yaw = 0;

  // EKF
  uint8_t tracker_state = 0;
  std::string tracker_target_id;
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

  // Aim
  bool aim_target_valid = false;
  int aim_face_index = -1;
  float aim_abs_yaw = 0;
  float aim_abs_pitch = 0;
  float aim_distance = 0;
  float aim_target_x = 0, aim_target_y = 0, aim_target_z = 0;
  float aim_flight_time = 0;
  float aim_pred_t = 0;

  // Command
  float cmd_yaw_pre_smooth = 0;
  float cmd_pitch_pre_smooth = 0;
  float cmd_yaw_published = 0;
  float cmd_pitch_published = 0;
  bool cmd_fire = false;

  // Fire gate
  uint8_t fire_blocker = 0;
  std::string fire_blocker_reason;

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
