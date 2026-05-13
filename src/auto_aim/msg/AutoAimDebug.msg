# Per-frame structured debug for auto_aim.
#
# Published on /auto_aim/debug each detection callback. Use this together
# with rosbag to diagnose where jitter or bias enters the pipeline. See
# docs/auto_aim_debugging_guide.md.
std_msgs/Header header

# ----- Detection (best detection in the frame, after class filter) -----
bool   detection_present
float32 bbox_cx
float32 bbox_cy
float32 bbox_w
float32 bbox_h
float32 detect_confidence
string  class_id

# Per-frame detection/filter counters. These identify where measurements
# disappear before the EKF sees them.
uint32 raw_kp_detection_count
uint32 class_reject_count
uint32 kp_low_score_reject_count
uint32 kp_geometry_reject_count
uint32 pnp_failed_count
uint32 pose_z_reject_count
uint32 pose_range_reject_count
uint32 armors_passed_to_tracker_count

# ----- PnP -----
bool   pnp_ok
bool   pnp_is_large
bool   pnp_size_hysteresis_kept # true when auto mode kept prior size despite small reproj margin
float32 pnp_reproj_err          # mean pixel reprojection error
float32 pnp_reproj_err_norm     # divided by the keypoint AABB diagonal
float32 pnp_small_reproj_err
float32 pnp_large_reproj_err
float32 pnp_size_margin         # abs(small-large), pixels; small margin means size choice is unstable
float32[8] pnp_image_points     # 4 corners actually fed to PnP (post reorder/check)
float32 pnp_tvec_x
float32 pnp_tvec_y
float32 pnp_tvec_z
uint8  pnp_reject_reason        # 0=OK, see PNP_REJECT_* below

uint8 PNP_OK                = 0
uint8 PNP_DEGEN_BBOX        = 1
uint8 PNP_NONFINITE         = 2
uint8 PNP_BAD_LIGHT_RATIO   = 3
uint8 PNP_SOLVER_FAIL       = 4
uint8 PNP_REPROJ_ABS        = 5
uint8 PNP_REPROJ_NORM       = 6
uint8 PNP_DEPTH_TOO_CLOSE   = 7
uint8 PNP_DEPTH_TOO_FAR     = 8

# Which measurement model produced this frame's PnP. Operators MUST verify
# this is MEAS_SOURCE_KEYPOINT in /auto_aim/debug; if it ever shows
# MEAS_SOURCE_NONE, no detection passed the keypoint gate this frame.
uint8  meas_source              # see MEAS_SOURCE_* below
uint8 MEAS_SOURCE_NONE     = 0
uint8 MEAS_SOURCE_KEYPOINT = 1

# ----- Keypoints (YOLO-pose path) -----
# Raw keypoints in image pixels, model-index order TL,TR,BR,BL (per
# ArmorKeypoint.msg contract). Compare against pnp_image_points to see
# whether the geometry sanity check reordered or rejected the quad.
float32[8] kp_image_points
float32[4] kp_scores            # per-keypoint detector scores
float32 kp_min_score            # min(kp_scores)
float32 kp_mean_score           # mean(kp_scores)
bool   kp_geometry_valid        # winding + convexity check passed
uint8  kp_reject_reason         # 0=OK, see KP_REJECT_* below

uint8 KP_OK                = 0
uint8 KP_NONFINITE         = 1
uint8 KP_LOW_SCORE         = 2
uint8 KP_INVALID_GEOMETRY  = 3
uint8 KP_OUT_OF_IMAGE      = 4

# ----- Frame transform (odom = micro reference) -----
string pose_source
bool   pose_present
bool   pose_fresh
float32 pose_age_s
float32 imu_yaw
float32 imu_pitch
float32 odom_x
float32 odom_y
float32 odom_z
float32 odom_yaw

# ----- Tracker EKF -----
uint8  tracker_state            # 0=LOST 1=DETECTING 2=TRACKING 3=TEMP_LOST
string tracker_target_id
uint32 tracker_assigned_count
uint32 tracker_association_reject_count
string tracker_miss_reason
float32[9] ekf_state            # xc vxc yc vyc za vza yaw vyaw r
float32 ekf_innovation_norm     # legacy, mixes meters and radians
float32 ekf_innovation_pos_norm # sqrt(yx^2 + yy^2 + yz^2) [m]
float32 ekf_innovation_yaw_abs  # |y_yaw| [rad]
float32 ekf_mahalanobis
float32 ekf_q_pos_eff           # effective q_pos used by ekfPredict
float32 ekf_q_yaw_eff           # effective q_yaw used by ekfPredict
float32 ekf_r_pos_eff           # effective sqrt(R_pos) at last update [m]
float32 ekf_r_yaw_eff           # effective sqrt(R_yaw) at last update [rad]
float32 ekf_pos_sigma           # sqrt(P(0,0)+P(2,2)+P(4,4)) [m]
float32 ekf_yaw_sigma           # sqrt(P(6,6)) [rad]
uint32 ekf_match_count          # consecutive matched frames
uint32 ekf_miss_count           # consecutive missed frames
float32 ekf_measurement_age_s   # s since last accepted measurement
uint8 ekf_measurement_quality   # see EKF_MQ_* below
float32 best_match_mahalanobis
float32 best_match_position_diff
float32 best_match_yaw_diff
string match_reject_reason

uint8 EKF_MQ_NONE     = 0
uint8 EKF_MQ_ACCEPTED = 1
uint8 EKF_MQ_DEGRADED = 2
uint8 EKF_MQ_REJECTED = 3

# ----- Aim planner -----
bool   aim_target_valid
int32  aim_face_index           # 0..3, or -1 if no aim
float32 aim_abs_yaw             # absolute yaw, micro reference [rad]
float32 aim_abs_pitch           # absolute pitch, micro reference [rad]
float32 aim_distance            # range to selected face [m]
float32 aim_target_x
float32 aim_target_y
float32 aim_target_z
float32 aim_flight_time         # seconds, ballistic
float32 aim_pred_t              # prediction lead used by the planner
float32 aim_rel_yaw             # current yaw error to command [rad], internal convention
float32 aim_rel_pitch           # current pitch error to command [rad], internal convention
float32 aim_fire_margin         # planner angular margin [rad]; negative blocks fire
bool   aim_anti_gyro_active
float32 aim_anti_gyro_residual  # seconds
float32 aim_cam_yaw             # selected impact ray in camera frame [rad]
float32 aim_cam_pitch           # selected impact ray in camera frame [rad]
float32 aim_cam_total_angle     # sqrt(aim_cam_yaw^2 + aim_cam_pitch^2)

# ----- Command publisher -----
float32 cmd_yaw_pre_smooth      # ballistic ray (before EMA, in micro frame)
float32 cmd_pitch_pre_smooth
float32 cmd_yaw_published       # what is on /tracker/cmd_gimbal angular.z
float32 cmd_pitch_published     # what is on /tracker/cmd_gimbal angular.y
bool   cmd_fire                 # what is on /tracker/cmd_gimbal angular.x
bool   cmd_published            # false only when no safe absolute hold exists
bool   cmd_hold_active          # true when publishing current-pose hold instead of a target
bool   cmd_coast_active         # true when briefly commanding TEMP_LOST prediction
bool   tracker_fresh_enough_for_command
float32 coast_age_s             # age since last accepted measurement while coasting

# ----- Fire gate (P6+ populates the reason; before P6 it stays empty) -----
uint8  fire_blocker             # 0 = allowed, see FireBlocker enum below
uint32 fire_blocker_mask         # bit N set means FIRE_* value N is also active
string fire_blockers_active      # comma-separated names from fire_blocker_mask
string fire_blocker_reason
string fire_alignment_source     # camera_angle, relative_error, or disabled
float32 fire_alignment_error     # angle actually used by OFF_AXIS gate [rad]

# Fire blocker enumeration (mirrors FireGate::Blocker):
uint8 FIRE_ALLOWED        = 0
uint8 FIRE_NOT_TRACKING   = 1
uint8 FIRE_OUT_OF_RANGE   = 2
uint8 FIRE_MARGIN_NEGATIVE= 3
uint8 FIRE_OFF_AXIS       = 4
uint8 FIRE_INVALID_TARGET = 5
uint8 FIRE_INVALID_BALLISTIC = 6
uint8 FIRE_SMOOTHING_LAG  = 7
uint8 FIRE_STALE_MEASUREMENT = 8
uint8 FIRE_ANTI_GYRO_TIMING = 9
uint8 FIRE_TEMP_LOST = 10
uint8 FIRE_STALE_POSE = 11
uint8 FIRE_NO_POSE_SOURCE = 12

# ----- Latency (P7+) -----
# All latencies are float64 seconds.
#
# Timing model (see docs/auto_aim_pipeline.md):
#
#   image_stamp ──capture_to_process──▶ proc_start
#   proc_start  ──process─────────────▶ proc_end
#   proc_end    ──process_to_publish──▶ publish
#   total       = image_stamp -> publish
#
# The EKF state is anchored at image_stamp. computeAim predicts forward
# by pred_t_total_s to the bullet impact moment.
float64 latency_capture_to_process_s
float64 latency_process_s
float64 latency_process_to_publish_s
float64 latency_total_s
float64 latency_estimate_ema_s         # EMA-smoothed total latency

# ----- Prediction lead components -----
# pred_t_total_s = pred_lead_measured_s
#                + pred_lead_gimbal_s
#                + pred_lead_time_bias_s
#                + pred_lead_ema_s
#                + pred_flight_time_s
# Components are zero when the corresponding feature flag is off.
float64 pred_lead_measured_s           # use_measured_latency: latency EMA
float64 pred_lead_gimbal_s             # use_measured_latency: fixed actuator delay
float64 pred_lead_time_bias_s          # cfg.time_bias contribution
float64 pred_lead_ema_s                # enable_ema_delay_compensation: dt*(1-a)/a
float64 pred_flight_time_s             # ballistic flight time
float64 pred_t_total_s                 # final pred_t passed to face loop

# ----- Smoothing diagnostic (always logged; gate is opt-in) -----
float32 smoothing_lag_rad              # |pre_smooth - published| in joint angles
