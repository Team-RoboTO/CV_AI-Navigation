# UNUSED at runtime. The runtime command contract is geometry_msgs/Twist
# published on /tracker/cmd_gimbal. See docs/gimbal_command_contract.md.
#
# This message is preserved as a placeholder for a possible future migration
# but is currently neither produced nor consumed by the auto_aim node.
std_msgs/Header header
float64 yaw        # yaw command [degrees]
float64 pitch      # pitch command [degrees]
float64 distance   # target distance [m]
bool fire_cmd      # true when fire is allowed
