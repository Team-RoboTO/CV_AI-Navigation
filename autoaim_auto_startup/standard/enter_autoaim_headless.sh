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
    # Keep the manual shell in the same ROS 2 graph as the auto-started launch.
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
    echo "[autoaim] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

    cd /workspaces/isaac_ros-dev
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    echo "[autoaim] entered isaac_ros_dev-aarch64-container at $(pwd)"
    echo "[autoaim] ROS package check:"
    ros2 pkg prefix autoaim
    ros2 pkg prefix autoaim_realsense
    exec bash
  '
