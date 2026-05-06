std_msgs/Header header
# Absolute pitch command in the targeting/world frame [deg].
float64 pitch
# Absolute yaw command in the targeting/world frame [deg].
float64 yaw
# Target range used for the shot plan [m].
float64 distance
# Fire request produced by the current targeting pipeline.
bool fire_cmd
