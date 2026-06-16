#!/usr/bin/env bash
set -euo pipefail

CONTAINER="isaac_ros_dev-aarch64-container"

# Your camera_info topic
HEALTH_TOPIC="/camera_info"

# Increase this if first boot still starts too early.
# Try 45 or 60 seconds on Jetson.
BOOT_DELAY=10

echo "[autoaim] script started"

echo "[autoaim] waiting for Docker..."
until docker info >/dev/null 2>&1; do
  echo "[autoaim] Docker not ready yet..."
  sleep 2
done
echo "[autoaim] Docker is ready"

echo "[autoaim] waiting ${BOOT_DELAY}s after boot before starting container..."
sleep "$BOOT_DELAY"

echo "[autoaim] waiting for boot devices..."
udevadm settle || true
sleep 5

echo "[autoaim] checking video devices..."
if ls /dev/video* >/dev/null 2>&1; then
  echo "[autoaim] video device found"
else
  echo "[autoaim] no /dev/video* found, continuing anyway"
fi

echo "[autoaim] checking nvargus-daemon if present..."
if systemctl list-unit-files | grep -q nvargus-daemon.service; then
  systemctl is-active --quiet nvargus-daemon.service || systemctl restart nvargus-daemon.service || true
  sleep 3
fi

echo "[autoaim] recreating session mount paths..."
install -d -o 1000 -g 1000 -m 700 /run/user/1000 || true
install -d -o 1000 -g 1000 -m 700 /run/user/1000/keyring || true

if [ ! -e /run/user/1000/keyring/ssh ]; then
  touch /run/user/1000/keyring/ssh
  chown 1000:1000 /run/user/1000/keyring/ssh
fi

mkdir -p /tmp/.X11-unix || true

echo "[autoaim] stopping old container if running..."
docker stop -t 3 "$CONTAINER" >/dev/null 2>&1 || true
sleep 3

echo "[autoaim] starting container..."
docker start "$CONTAINER"

echo "[autoaim] waiting 5 seconds for container init..."
sleep 5

echo "[autoaim] container status:"
docker ps --filter "name=$CONTAINER"

echo "[autoaim] starting ROS launch and checking real camera_info messages..."

exec docker exec -i "$CONTAINER" bash -s -- "$HEALTH_TOPIC" <<'INSIDE_CONTAINER'
set -eo pipefail

HEALTH_TOPIC="$1"

# Do NOT use set -u while sourcing ROS.
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
set -u

echo "[autoaim] inside container"
echo "[autoaim] health topic: $HEALTH_TOPIC"

check_camera_info_messages() {
  local topic="$1"
  local needed="$2"

  echo "[autoaim] checking for $needed real messages on $topic"

  for n in $(seq 1 "$needed"); do
    echo "[autoaim] waiting for camera_info message $n/$needed"

    if timeout 8 ros2 topic echo --once "$topic" --qos-profile sensor_data >/tmp/autoaim_topic_check.txt 2>&1; then
      echo "[autoaim] received camera_info message $n/$needed with sensor_data QoS"
      continue
    fi

    if timeout 8 ros2 topic echo --once "$topic" >/tmp/autoaim_topic_check.txt 2>&1; then
      echo "[autoaim] received camera_info message $n/$needed with default QoS"
      continue
    fi

    echo "[autoaim] failed to receive camera_info message"
    cat /tmp/autoaim_topic_check.txt || true
    return 1
  done

  return 0
}

ros2 launch autoaim sentry.launch.py &
LAUNCH_PID=$!

echo "[autoaim] ROS launch started, PID: $LAUNCH_PID"
echo "[autoaim] waiting for camera_info topic to become stable..."

for i in {1..6}; do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[autoaim] ERROR: ROS launch died"
    wait "$LAUNCH_PID"
    exit 1
  fi

  echo "[autoaim] health check attempt $i"

  echo "[autoaim] available camera_info topics:"
  ros2 topic list 2>/dev/null | grep camera_info || true

  if check_camera_info_messages "$HEALTH_TOPIC" 3; then
    echo "[autoaim] first camera_info check passed"

    echo "[autoaim] waiting 10 seconds, then checking camera_info again..."
    sleep 10

    if check_camera_info_messages "$HEALTH_TOPIC" 3; then
      echo "[autoaim] OK: camera_info is stable and publishing real messages"
      wait "$LAUNCH_PID"
      exit $?
    fi
  fi

  echo "[autoaim] camera_info not stable yet, retrying..."
  sleep 3
done

echo "[autoaim] ERROR: camera_info did not become stable, restarting service"
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 2
kill -9 "$LAUNCH_PID" 2>/dev/null || true
exit 1
INSIDE_CONTAINER
