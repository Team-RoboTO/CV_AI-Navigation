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
