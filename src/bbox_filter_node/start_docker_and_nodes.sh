sleep 15
docker start isaac_ros_dev-aarch64-container

tmux new-session -d -s tmux1 'docker exec -t isaac_ros_dev-aarch64-container bash -c "source install/setup.bash && ros2 launch launch_pkg realsense_only_det.launch.py"'

tmux new-session -d -s tmux2 'docker exec -t isaac_ros_dev-aarch64-container bash -c "source install/setup.bash && ros2 launch filter __sentry_red.launch.py"'

echo roboto | sudo -S  chmod 666 /dev/ttyUSB0
tmux new-session -d -s tmux3 'docker exec -t isaac_ros_dev-aarch64-container bash -c "source install/setup.bash && ros2 run cmd_vel_serial cmd_vel_subscriber"'

### CRONTAB SETTING
# $ crontab -e
# @reboot /path_to/start_docker_and_nodes.sh