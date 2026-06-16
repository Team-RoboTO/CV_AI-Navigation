# 1. Bring the interface up (replace eno1 with your actual interface name)
sudo ip link set eno1 up

# 2. Assign the static IP the driver expects
sudo ip addr flush dev eno1
sudo ip addr add 192.168.1.5/24 dev eno1

# 3. Double check the LiDAR is actually there
ping 192.168.1.3

FOR MAPPING
1
ros2 launch nav2_new sensors.launch.py mount:=normal rviz:=false use_livox_filter:=false


2
ros2 launch nav2_new navigation.launch.py \
  map:=/root/nav2_ws/maps/robomaster_map.yaml \
  use_sim_time:=false

FOR NAVIGATION WITH THE MAP MADE
1
ros2 launch nav2_new sensors.launch.py mount:=normal rviz:=false
2
ros2 launch nav2_new navigation.launch.py map:=/root/nav2_ws/maps/robomaster_map.yaml use_sim_time:=false
