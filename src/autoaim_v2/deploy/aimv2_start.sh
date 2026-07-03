#!/usr/bin/env bash
# autoaim_v2 autostart — runs the aim node inside the autoaim_headless
# container. Mirrors autoaim_autostart.sh (standard) but launches the new
# single-process pipeline with the fire switch ON.
set -euo pipefail

CONTAINER="autoaim_headless"
WORKDIR="/workspaces/isaac_ros-dev"
SERIAL_DEV="/dev/ttyACM0"

echo "[aimv2] waiting for micro on ${SERIAL_DEV}..."
for _ in $(seq 1 60); do
  [ -e "${SERIAL_DEV}" ] && break
  sleep 1
done
[ -e "${SERIAL_DEV}" ] || echo "[aimv2] WARNING: ${SERIAL_DEV} still absent — node will retry internally"

if [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null || echo false)" != "true" ]; then
  echo "[aimv2] starting container ${CONTAINER}"
  docker start "${CONTAINER}" >/dev/null
fi

# Foreground exec so systemd owns the lifecycle; PID file lets ExecStop send
# SIGINT to the whole process group for a clean ROS shutdown.
exec docker exec -w "${WORKDIR}" "${CONTAINER}" bash -lc '
  set -e
  cd /workspaces/isaac_ros-dev
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  echo "[aimv2] launching autoaim_v2 (shooting_active:=true)"
  setsid ros2 launch autoaim_v2 standard_1v1.launch.py shooting_active:=true &
  LAUNCH_PID=$!
  echo "$LAUNCH_PID" > /tmp/aimv2_launch.pid
  wait "$LAUNCH_PID"
'
