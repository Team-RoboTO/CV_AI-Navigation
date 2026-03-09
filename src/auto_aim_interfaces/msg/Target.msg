std_msgs/Header header
bool tracking
string id
int32 tracker_id
int32 armors_num

# Tracker state machine
uint8 tracker_state
uint8 LOST=0
uint8 DETECTING=1
uint8 TRACKING=2
uint8 TEMP_LOST=3

# EKF v_yaw variance (P_post(7,7)) — spin-rate confidence
float64 v_yaw_variance
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 yaw
float64 v_yaw
float64 radius_1
float64 radius_2
float64 dz
builtin_interfaces/Time last_measurement_stamp
