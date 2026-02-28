# Difference between the current measurement and prediction
float64 position_diff
float64 yaw_diff
# Unfiltered position and yaw
geometry_msgs/Point position
float64 yaw
# Adaptive damping (effective alpha this frame)
float64 xyz_damping_alpha
float64 yaw_damping_alpha
