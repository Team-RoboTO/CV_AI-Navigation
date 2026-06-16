#!/usr/bin/env bash
set -euo pipefail

CONTAINER="autoaim_headless"

LAUNCH_PKG="autoaim"
LAUNCH_FILE="sentry.launch.py"
HEALTH_TOPIC="/camera_info"

DISPLAY_ID=""
X_SOCKET=""
AUTH_FILE=""

BOOT_DELAY=5

WAIT_SERIAL=1
SERIAL_DEV="/dev/ttyACM0"

echo "[autoaim] script started"

echo "[autoaim] waiting for Docker..."
until docker info >/dev/null 2>&1; do
  echo "[autoaim] Docker not ready yet..."
  sleep 2
done
echo "[autoaim] Docker is ready"

echo "[autoaim] waiting ${BOOT_DELAY}s after boot..."
sleep "$BOOT_DELAY"

echo "[autoaim] waiting for graphical X display socket..."
for i in {1..180}; do
  FOUND_SOCKET="$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n 1 || true)"

  if [ -n "$FOUND_SOCKET" ] && [ -S "$FOUND_SOCKET" ]; then
    X_SOCKET="$FOUND_SOCKET"
    X_NUM="$(basename "$X_SOCKET" | sed 's/^X//')"
    DISPLAY_ID=":$X_NUM"

    echo "[autoaim] X display socket found: $X_SOCKET"
    echo "[autoaim] using DISPLAY=$DISPLAY_ID"
    break
  fi

  echo "[autoaim] no X display socket ready yet $i/180"
  sleep 1
done

if [ -z "$DISPLAY_ID" ] || [ ! -S "$X_SOCKET" ]; then
  echo "[autoaim] ERROR: no X display socket found in /tmp/.X11-unix/"
  ls -l /tmp/.X11-unix/ || true
  exit 1
fi

echo "[autoaim] finding X authority file..."
AUTH_FILE="$(ps aux | grep -E 'Xorg|Xwayland' | grep -v grep | sed -n 's/.* -auth \([^ ]*\).*/\1/p' | head -n 1 || true)"

if [ -n "$AUTH_FILE" ]; then
  echo "[autoaim] using XAUTHORITY=$AUTH_FILE"
else
  echo "[autoaim] WARNING: could not find XAUTHORITY file from Xorg process"
fi

echo "[autoaim] checking xrandr on DISPLAY=${DISPLAY_ID}..."
if [ -n "$AUTH_FILE" ]; then
  if env XAUTHORITY="$AUTH_FILE" DISPLAY="$DISPLAY_ID" xrandr >/tmp/autoaim_xrandr.txt 2>&1; then
    echo "[autoaim] xrandr OK on DISPLAY=${DISPLAY_ID} using XAUTHORITY"
  else
    echo "[autoaim] WARNING: xrandr failed with XAUTHORITY"
    cat /tmp/autoaim_xrandr.txt || true
  fi
else
  if DISPLAY="$DISPLAY_ID" xrandr >/tmp/autoaim_xrandr.txt 2>&1; then
    echo "[autoaim] xrandr OK on DISPLAY=${DISPLAY_ID}"
  else
    echo "[autoaim] WARNING: xrandr failed"
    cat /tmp/autoaim_xrandr.txt || true
  fi
fi

echo "[autoaim] allowing local root/docker display access..."
if [ -n "$AUTH_FILE" ]; then
  env XAUTHORITY="$AUTH_FILE" DISPLAY="$DISPLAY_ID" xhost +local: || true
  env XAUTHORITY="$AUTH_FILE" DISPLAY="$DISPLAY_ID" xhost +SI:localuser:root || true
  env XAUTHORITY="$AUTH_FILE" DISPLAY="$DISPLAY_ID" xhost +local:root || true
  env XAUTHORITY="$AUTH_FILE" DISPLAY="$DISPLAY_ID" xhost +local:docker || true
else
  DISPLAY="$DISPLAY_ID" xhost +local: || true
  DISPLAY="$DISPLAY_ID" xhost +SI:localuser:root || true
  DISPLAY="$DISPLAY_ID" xhost +local:root || true
  DISPLAY="$DISPLAY_ID" xhost +local:docker || true
fi

echo "[autoaim] checking /dev/fb0..."
if [ -e /dev/fb0 ]; then
  echo "[autoaim] /dev/fb0 found"
else
  echo "[autoaim] WARNING: /dev/fb0 not found"
fi

echo "[autoaim] waiting for nvargus-daemon..."
for i in {1..60}; do
  if systemctl is-active --quiet nvargus-daemon.service; then
    echo "[autoaim] nvargus-daemon active"
    break
  fi

  echo "[autoaim] nvargus-daemon not active yet $i/60"
  sleep 1
done

if ! systemctl is-active --quiet nvargus-daemon.service; then
  echo "[autoaim] WARNING: nvargus-daemon not active, trying restart"
  systemctl restart nvargus-daemon.service || true
  sleep 5
fi

echo "[autoaim] checking zed_x_daemon if present..."
if systemctl list-unit-files | grep -q "^zed_x_daemon.service"; then
  echo "[autoaim] zed_x_daemon service exists, waiting for it..."

  for i in {1..60}; do
    if systemctl is-active --quiet zed_x_daemon.service; then
      echo "[autoaim] zed_x_daemon active"
      break
    fi

    echo "[autoaim] zed_x_daemon not active yet $i/60"
    sleep 1
  done

  if ! systemctl is-active --quiet zed_x_daemon.service; then
    echo "[autoaim] WARNING: zed_x_daemon not active, trying restart"
    systemctl restart zed_x_daemon.service || true
    sleep 5
  fi
else
  echo "[autoaim] zed_x_daemon.service not found, continuing"
fi

echo "[autoaim] checking video devices, non-blocking..."
ls -l /dev/video* 2>/dev/null || echo "[autoaim] no /dev/video* visible, continuing because GMSL/Argus may not expose normal USB video devices"

if [ "$WAIT_SERIAL" = "1" ]; then
  echo "[autoaim] waiting for serial device $SERIAL_DEV..."
  for i in {1..90}; do
    if [ -e "$SERIAL_DEV" ]; then
      echo "[autoaim] serial device found: $SERIAL_DEV"
      break
    fi

    echo "[autoaim] $SERIAL_DEV not ready yet $i/90"
    sleep 1
  done

  if [ ! -e "$SERIAL_DEV" ]; then
    echo "[autoaim] ERROR: serial device not found: $SERIAL_DEV"
    exit 1
  fi
fi

echo "[autoaim] starting container if needed..."
if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null || echo false)" != "true" ]; then
  docker start "$CONTAINER"
  echo "[autoaim] container started"
else
  echo "[autoaim] container already running"
fi

echo "[autoaim] waiting 5 seconds for container init..."
sleep 5

echo "[autoaim] container status:"
docker ps --filter "name=$CONTAINER"

echo "[autoaim] launching ROS and checking camera_info..."

exec docker exec -i \
  -e DISPLAY="$DISPLAY_ID" \
  -e QT_QPA_PLATFORM=xcb \
  "$CONTAINER" \
  bash -s -- "$LAUNCH_PKG" "$LAUNCH_FILE" "$HEALTH_TOPIC" <<'INSIDE_CONTAINER'
set -eo pipefail

LAUNCH_PKG="$1"
LAUNCH_FILE="$2"
HEALTH_TOPIC="$3"

source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
set -u

echo "[autoaim] inside container"
echo "[autoaim] DISPLAY=$DISPLAY"
echo "[autoaim] package=$LAUNCH_PKG"
echo "[autoaim] launch=$LAUNCH_FILE"
echo "[autoaim] health topic=$HEALTH_TOPIC"

echo "[autoaim] checking display socket inside container..."
ls -l /tmp/.X11-unix/ || true

echo "[autoaim] checking camera/video devices inside container..."
ls -l /dev/video* 2>/dev/null || true
ls -l /dev/nv* 2>/dev/null || true

echo "[autoaim] checking serial inside container..."
ls -l /dev/ttyACM* 2>/dev/null || true

if ! ros2 pkg prefix "$LAUNCH_PKG" >/dev/null 2>&1; then
  echo "[autoaim] ERROR: ROS package not found: $LAUNCH_PKG"
  echo "[autoaim] available packages containing auto/aim:"
  ros2 pkg list | grep -Ei "auto|aim" || true
  exit 1
fi

if [ -f /tmp/autoaim_launch.pid ]; then
  OLD_PID="$(cat /tmp/autoaim_launch.pid || true)"
  if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
    echo "[autoaim] old launch process still running with PID $OLD_PID"
    echo "[autoaim] stopping old launch process group"
    kill -INT "-$OLD_PID" 2>/dev/null || kill -INT "$OLD_PID" 2>/dev/null || true
    sleep 3
    kill -TERM "-$OLD_PID" 2>/dev/null || true
  fi
  rm -f /tmp/autoaim_launch.pid
fi

check_camera_info_messages() {
  local topic="$1"
  local needed="$2"

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

echo "[autoaim] starting launch:"
echo "ros2 launch $LAUNCH_PKG $LAUNCH_FILE"

rm -f /tmp/autoaim_launch.pid

setsid ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" &
LAUNCH_PID=$!

echo "$LAUNCH_PID" > /tmp/autoaim_launch.pid
echo "[autoaim] ROS launch started, PID: $LAUNCH_PID"
echo "[autoaim] saved launch PID to /tmp/autoaim_launch.pid"

for i in {1..8}; do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[autoaim] ERROR: ROS launch died"
    wait "$LAUNCH_PID"
    exit 1
  fi

  echo "[autoaim] health check attempt $i"

  echo "[autoaim] camera_info topics:"
  ros2 topic list 2>/dev/null | grep camera_info || true

  if check_camera_info_messages "$HEALTH_TOPIC" 3; then
    echo "[autoaim] first camera_info check passed"

    echo "[autoaim] waiting 10 seconds, then checking again..."
    sleep 10

    if check_camera_info_messages "$HEALTH_TOPIC" 3; then
      echo "[autoaim] OK: camera_info is stable"
      wait "$LAUNCH_PID"
      exit $?
    fi
  fi

  echo "[autoaim] camera_info not stable yet, retrying..."
  sleep 3
done

echo "[autoaim] ERROR: camera_info did not become stable, restarting service"
kill -INT "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 2
kill -TERM "-$LAUNCH_PID" 2>/dev/null || true
rm -f /tmp/autoaim_launch.pid
exit 1
INSIDE_CONTAINER
