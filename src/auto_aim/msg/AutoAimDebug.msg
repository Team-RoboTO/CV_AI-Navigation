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

# ----- PnP -----
bool   pnp_ok
bool   pnp_is_large
float32 pnp_reproj_err          # mean pixel reprojection error
float32 pnp_reproj_err_norm     # divided by bbox diagonal
float32[8] pnp_image_points     # 4 corners (x0 y0 x1 y1 x2 y2 x3 y3)
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

# ----- Frame transform (odom = micro reference) -----
float32 odom_x
float32 odom_y
float32 odom_z
float32 odom_yaw

# ----- Tracker EKF -----
uint8  tracker_state            # 0=LOST 1=DETECTING 2=TRACKING 3=TEMP_LOST
string tracker_target_id
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

# ----- Command publisher -----
float32 cmd_yaw_pre_smooth      # ballistic ray (before EMA, in micro frame)
float32 cmd_pitch_pre_smooth
float32 cmd_yaw_published       # what is on /tracker/cmd_gimbal angular.z
float32 cmd_pitch_published     # what is on /tracker/cmd_gimbal angular.y
bool   cmd_fire                 # what is on /tracker/cmd_gimbal angular.x

# ----- Fire gate (P6+ populates the reason; before P6 it stays empty) -----
uint8  fire_blocker             # 0 = allowed, see FireBlocker enum below
string fire_blocker_reason

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
