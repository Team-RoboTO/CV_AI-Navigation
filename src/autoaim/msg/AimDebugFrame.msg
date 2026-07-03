std_msgs/Header header

uint64 frame_id
string input_mode
int32 enemy_class
bool solver_ready
bool gimbal_sample_valid
bool status_valid
bool serial_open
bool shooting_active

uint32 image_width
uint32 image_height
float64 camera_fx
float64 camera_fy
float64 camera_cx
float64 camera_cy

uint32 detection_count
uint32 accepted_count
AimDebugArmor[] armors

float64 capture_age_ms
float64 pipeline_latency_ms
float64 publish_period_ms

float64 gimbal_capture_yaw
float64 gimbal_capture_pitch
float64 gimbal_capture_yaw_raw
float64 gimbal_capture_pitch_raw
float64 gimbal_now_yaw
float64 gimbal_now_pitch
float64 gimbal_now_yaw_raw
float64 gimbal_now_pitch_raw

float64 micro_vx
float64 micro_vy
int32 micro_color
int32 micro_game_progress
float64 micro_hp
float64 micro_reserved9

float64 ego_x
float64 ego_y
float64 ego_vx
float64 ego_vy

int32 tracker_state
bool tracker_tracking
int32 tracker_generation
int32 tracker_last_face
int32 tracker_matched_count
float64[11] tracker_x
float64[11] tracker_p_diag

int32 fire_regime
int32 selected_face
float64 hit_probability
float64 target_omega
float64 target_distance
float64 flight_time
float64 prediction_horizon
float64 lock_error_yaw
float64 lock_error_pitch
bool locked
float64 aim_point_x
float64 aim_point_y
float64 aim_point_z
float64 next_shot_in

bool planned_command_valid
float64 planned_yaw_micro
float64 planned_pitch_micro
bool planned_shoot
bool planned_shoot_scheduled
float64 planned_ff_yaw_rate
float64 planned_ff_pitch_rate

bool tx_command_valid
float64 tx_yaw_micro
float64 tx_pitch_micro
bool tx_shoot
bool tx_shoot_scheduled
float64 tx_ff_yaw_rate
float64 tx_ff_pitch_rate
