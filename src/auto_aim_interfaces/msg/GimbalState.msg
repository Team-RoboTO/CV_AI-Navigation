std_msgs/Header header
# Absolute lower-computer/gimbal yaw in the command reference frame [rad].
float64 yaw
# Absolute lower-computer/gimbal pitch in the command reference frame [rad].
float64 pitch
# Optional measured yaw rate [rad/s]. Set to 0.0 if unavailable.
float64 yaw_velocity
# Optional measured pitch rate [rad/s]. Set to 0.0 if unavailable.
float64 pitch_velocity
# False means the sample should be ignored by targeting.
bool valid
