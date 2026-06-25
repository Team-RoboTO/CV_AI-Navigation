std_msgs/Header header

string state
uint8 state_id
bool tracking
bool aim_fire
bool cmd_fire
bool yaw_locked
bool pitch_locked
bool command_valid
bool holding

float64 yaw_error
float64 yaw_lock_threshold
float64 pitch_error
float64 pitch_lock_threshold

float64 vyaw
float64 vyaw_rpm
bool jump_detected
float64 jump_abs
float64 jump_dir
bool one_face_jump
float64 dt_jump
float64 vyaw_est_from_timing
bool vyaw_timing_accepted
bool vyaw_confident
bool phase_confident
bool static_confirmed
bool confirmed_spin
int32 consecutive_same_dir_jumps
float64 p_vyaw
# q_pos and q_yaw are now STATIC process-noise values (the adaptive/NIS q_pos
# controller was removed — see INSTRUCTIONS.md "Spinning-Target Tuning").
float64 q_pos
float64 q_yaw
uint8 faces
float64 phase
int32 best_face_idx
uint8 faces_checked
float64 phase_error
int32 selected_face_idx
bool face_lookahead_active
float64 face_time_to_window
float64 face_margin
float64 face_phase_error
string face_switch_reason
float64 fire_window
float64 flight_time
float64 prediction_bias
float64 prediction_time
float64 margin
float64 distance

# Impact-inside-plate fire gate (WORKLOG §4e). impact_inside is the physical fire
# trigger (predicted bullet impact lands on the selected plate board);
# impact_lateral/vertical are the signed metric miss [m] for tuning.
bool impact_inside
float64 impact_lateral
float64 impact_vertical

float64 cmd_yaw
float64 cmd_pitch
float64 micro_yaw
float64 micro_pitch
float64 pitch_raw
float64 pitch_lock
float64 pitch_cmd_echo
bool pitch_cmd_echo_valid

bool pixels_valid
bool pixels_inside
int32 aim_px_x
int32 aim_px_y
int32 impact_px_x
int32 impact_px_y

float64 radius
float64 dz

# ── Raw inputs + state for OFFLINE REPLAY and geometry inspection (WORKLOG §4d) ──
# det_*_raw: the raw selected armor detection in odom (BEFORE EKF), so a bag can be
#   replayed through the tracker / the spin observer re-run offline.
# ego_*: robot position in odom this frame.
# state_*: EKF center position + velocity (state_yaw/vyaw/radius are above).
# spin_*: position-based spin observer internals.
float64 det_x_raw
float64 det_y_raw
float64 det_z_raw
float64 det_yaw_raw
float64 ego_x
float64 ego_y
float64 state_xc
float64 state_vxc
float64 state_yc
float64 state_vyc
float64 state_vza
float64 spin_centroid_x
float64 spin_centroid_y
float64 spin_omega
int32 spin_invalid_streak

# Adaptive-follow telemetry (WORKLOG §4g). adaptive_follow_w: 1 = light damping
# (target moving, lead held → fast follow) … 0 = hard damping (stop/reversal, lead
# killed → less overshoot); 1.0 when off or in the spin regime.
# follow_signal_raw: the un-clamped signal (<0 ⇒ reversal, >1 ⇒ accelerating).
# realized_center_speed: |EMA of the per-frame EKF-center velocity| [m/s].
# spinning_regime: regime flag the EKF used this frame (q_yaw/q_pos/yaw-trust + it
#   gates the adaptive follow off; distinct from confirmed_spin which gates the aim).
float64 adaptive_follow_w
float64 follow_signal_raw
float64 realized_center_speed
bool spinning_regime

# ── Aim / lead ACTUALLY APPLIED + aim mode (WORKLOG §4g debug surface) ──
# aim_target_*: the odom point the gimbal is aiming at (after center-aim / ballistics).
# applied_lead_v{x,y}: translational lead velocity folded into the aim [m/s], AFTER
#   the spin_trans_lead_cap — compare to state_vxc/vyc to see how much the cap bit.
# applied_lead_vyaw: rotational lead used [rad/s] (0 when omega-gated; compare to vyaw).
# omega_reliable: was the rotational (vyaw) lead applied (false ⇒ aliased/unconfirmed).
# center_aimed: center-aim engaged (gimbal points at the spin-centre facing point).
float64 aim_target_x
float64 aim_target_y
float64 aim_target_z
float64 applied_lead_vx
float64 applied_lead_vy
float64 applied_lead_vyaw
bool omega_reliable
bool center_aimed

# ── Timing / detector health ──
# frame_dt: detection interval [s] (rate/aliasing diagnostic — 1/dt = live Hz).
# pipeline_latency: measured capture→aim latency [s] (the horizon = this + actuation).
# num_target_detections: target-class plates seen this frame (0 ⇒ detector miss).
# target_generation: bumps on every track (re)acquire/switch — spots track resets.
float64 frame_dt
float64 pipeline_latency
int32 num_target_detections
int32 target_generation

bool matched
float64 mahalanobis
int32 matched_face_idx
float64 association_mahalanobis
bool association_yaw_valid
int32 association_yaw_hypothesis
int32 last_matched_face_idx
bool face_transition_observed
float64 pd
float64 yd
bool oblique
float64 det_yaw
float64 state_yaw
bool yaw_replaced_by_bearing
float64 reproj_error
string selected_class_id

# Fire gate diagnostics. These are debug-only mirrors of the final checks used
# before /cmd_vel_AI.angular.x is set.
string fire_block_reason
bool fire_check_aim_tracking
bool fire_check_tracker_tracking
bool fire_check_target_valid
bool fire_check_distance_ok
bool fire_check_margin_ok
bool fire_check_finite_cmd
bool fire_check_pixels_ok
bool fire_check_command_valid
bool fire_check_yaw_locked
bool fire_check_pitch_locked
bool fire_check_command_locked
bool fire_check_not_holding
