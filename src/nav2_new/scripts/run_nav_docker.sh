#!/usr/bin/env bash
# =============================================================================
# run_nav_docker.sh — Start the Nav container on Jetson
# =============================================================================
# Expects the workspace at:
#   ~/roboto/nav2_ws/src/
# and map persistence at:
#   ~/roboto/roboto_maps/
#
# Uses --net=host so ROS 2 topics flow between this container and the CV
# container. Both containers MUST share the same ROS_DOMAIN_ID (default 0).
# =============================================================================
set -e

IMAGE="${IMAGE:-roboto/nav:latest}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Workspace root (parent of nav2_ws/)
WS_ROOT="${WS_ROOT:-$HOME/roboto}"

if [ ! -d "${WS_ROOT}/nav2_ws/src" ]; then
    echo "ERROR: ${WS_ROOT}/nav2_ws/src does not exist."
    echo "Set WS_ROOT=<path> or create the workspace first."
    exit 1
fi

mkdir -p "${WS_ROOT}/roboto_maps"

# X11 access for GUI apps (RViz, rqt_tf_tree)
xhost +local:docker >/dev/null 2>&1 || true

docker run -it --rm \
  --privileged --net=host \
  --env="DISPLAY=${DISPLAY}" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  --env="RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="${WS_ROOT}/nav2_ws/src:/root/nav2_ws/src" \
  --volume="${WS_ROOT}/roboto_maps:/root/roboto_maps" \
  --device /dev/dri:/dev/dri \
  --name roboto_nav \
  "${IMAGE}"
