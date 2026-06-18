#!/usr/bin/env bash
set -euo pipefail

# Manual shell entry for the Standard + RealSense container.
# This intentionally does not depend on X11/Wayland because this device should
# be able to run and be debugged over SSH without a monitor attached.

CONTAINER="autoaim_headless"
WORKDIR="/workspaces/isaac_ros-dev"

if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null || echo false)" != "true" ]; then
  docker start "$CONTAINER" >/dev/null
fi

exec docker exec -it \
  -w "$WORKDIR" \
  "$CONTAINER" \
  bash -lc '
    cd /workspaces/isaac_ros-dev
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    echo "[autoaim] entered isaac_ros_dev-aarch64-container at $(pwd)"
    echo "[autoaim] ROS package check:"
    ros2 pkg prefix autoaim
    ros2 pkg prefix autoaim_realsense
    exec bash
  '
