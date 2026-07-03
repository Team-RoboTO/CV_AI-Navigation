#!/usr/bin/env bash
# autoaim_v2 autostart (RealSense variant) — runs the aim node inside the
# autoaim_headless container with the RealSense D455 camera.
set -euo pipefail

CONTAINER="autoaim_headless"
WORKDIR="/workspaces/isaac_ros-dev"
SERIAL_DEV="/dev/ttyACM0"

echo "[aimv2-rs] waiting for micro on ${SERIAL_DEV}..."
for _ in $(seq 1 60); do
  [ -e "${SERIAL_DEV}" ] && break
  sleep 1
done
[ -e "${SERIAL_DEV}" ] || echo "[aimv2-rs] WARNING: ${SERIAL_DEV} still absent — node will retry internally"

if [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null || echo false)" != "true" ]; then
  echo "[aimv2-rs] starting container ${CONTAINER}"
  docker start "${CONTAINER}" >/dev/null
fi

exec docker exec -w "${WORKDIR}" "${CONTAINER}" bash -lc '
  set -e
  cd /workspaces/isaac_ros-dev
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  echo "[aimv2-rs] launching autoaim_v2 with RealSense (shooting_active:=true)"
  setsid ros2 launch autoaim_v2 standard_rs.launch.py shooting_active:=true &
  LAUNCH_PID=$!
  echo "$LAUNCH_PID" > /tmp/aimv2_launch.pid
  wait "$LAUNCH_PID"
'
