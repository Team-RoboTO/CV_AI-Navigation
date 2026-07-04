#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# Standard + RealSense auto-start launcher for the isaac_ros_dev-aarch64 container.
#
# This script is intentionally NOT generic: it is tuned for this Jetson/device.
# It does not wait for ZED/Argus services and it does not require an X display.
# The service can therefore start cleanly in true headless mode after boot.
# =============================================================================

CONTAINER="autoaim_headless"
WORKDIR="/workspaces/isaac_ros-dev"
# Single-mount setup: workspace is mounted only at WORKDIR.
# colcon --symlink-install creates absolute symlinks that resolve inside the same mount.
SYMLINK_TARGET_WS="/workspaces/isaac_ros-dev"

LAUNCH_PKG="autoaim"
LAUNCH_FILE="standard.launch.py"
CAMERA="realsense"
HEALTH_TOPIC="/camera_info"

# Intel RealSense USB vendor id. This is only a startup hint; the ROS detector is
# still responsible for opening the camera and will be validated by /camera_info.
REALSENSE_USB_ID="8086"

BOOT_DELAY=5

# Standard needs the microcontroller link for /micro_status and gimbal commands.
WAIT_SERIAL=1
SERIAL_DEV="/dev/ttyACM0"

CAMERA_INFO_MESSAGES=3
FIRST_HEALTH_ATTEMPTS=8
TOPIC_TIMEOUT_SEC=8

# Dataset recording: rosbag of CLEAN camera frames (JPEG) for training data.
# RECORD_EVERY throttles the detector's recording publisher: camera fps / N
# images per second (60 fps / 2 = 30 img/s, ~90 KB/frame at 848x480 q95,
# ~10 GB/hour). Bags land in WORKDIR/RECORD_DIR (host-visible via the mount);
# oldest recordings are deleted once the folder exceeds RECORD_MAX_TOTAL_GB.
# NOTE: the JPEG encode runs inside the detector frame loop — RECORD_EVERY=1
# (60 img/s) would spend extra ms per frame in the aiming loop; keep >=2 in matches.
RECORD_ENABLE=1
RECORD_EVERY=1
RECORD_TOPIC="/camera/image_raw/compressed"
RECORD_DIR="bags/autostart"
RECORD_MAX_TOTAL_GB=23

log() {
  echo "[autoaim] $*"
}

log "script started: Standard + RealSense profile"

log "waiting for Docker..."
until docker info >/dev/null 2>&1; do
  log "Docker not ready yet..."
  sleep 2
done
log "Docker is ready"

log "waiting ${BOOT_DELAY}s after boot..."
sleep "$BOOT_DELAY"

log "checking RealSense USB visibility (vendor ${REALSENSE_USB_ID})..."
if command -v lsusb >/dev/null 2>&1; then
  for i in {1..60}; do
    if lsusb 2>/dev/null | grep -qi " ${REALSENSE_USB_ID}:"; then
      log "RealSense USB device found"
      break
    fi

    log "RealSense USB not visible yet $i/60"
    sleep 1
  done

  if ! lsusb 2>/dev/null | grep -qi " ${REALSENSE_USB_ID}:"; then
    log "WARNING: RealSense USB device not visible; continuing because the detector/health-check will be the final authority"
  fi
else
  log "WARNING: lsusb not installed on host; skipping RealSense USB pre-check"
fi

log "checking device nodes, non-blocking..."
ls -l /dev/bus/usb 2>/dev/null || log "WARNING: /dev/bus/usb not visible on host"
ls -l /dev/video* 2>/dev/null || log "no /dev/video* visible yet; continuing"

if [ "$WAIT_SERIAL" = "1" ]; then
  log "waiting for serial device $SERIAL_DEV..."
  for i in {1..90}; do
    if [ -e "$SERIAL_DEV" ]; then
      log "serial device found: $SERIAL_DEV"
      break
    fi

    log "$SERIAL_DEV not ready yet $i/90"
    sleep 1
  done

  if [ ! -e "$SERIAL_DEV" ]; then
    log "ERROR: serial device not found: $SERIAL_DEV"
    exit 1
  fi
fi

log "starting container if needed..."
if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null || echo false)" != "true" ]; then
  docker start "$CONTAINER"
  log "container started"
else
  log "container already running"
fi

log "waiting 5 seconds for container init..."
sleep 5

log "container status:"
docker ps --filter "name=$CONTAINER"

log "launching ROS and checking ${HEALTH_TOPIC}..."

exec docker exec -i \
  -w "$WORKDIR" \
  "$CONTAINER" \
  bash -s -- "$LAUNCH_PKG" "$LAUNCH_FILE" "$HEALTH_TOPIC" "$CAMERA" "$WORKDIR" "$SYMLINK_TARGET_WS" "$CAMERA_INFO_MESSAGES" "$FIRST_HEALTH_ATTEMPTS" "$TOPIC_TIMEOUT_SEC" "$RECORD_ENABLE" "$RECORD_EVERY" "$RECORD_TOPIC" "$RECORD_DIR" "$RECORD_MAX_TOTAL_GB" <<'INSIDE_CONTAINER'
set -eo pipefail

LAUNCH_PKG="$1"
LAUNCH_FILE="$2"
HEALTH_TOPIC="$3"
CAMERA="$4"
WORKDIR="$5"
SYMLINK_TARGET_WS="$6"
CAMERA_INFO_MESSAGES="$7"
FIRST_HEALTH_ATTEMPTS="$8"
TOPIC_TIMEOUT_SEC="$9"
RECORD_ENABLE="${10}"
RECORD_EVERY="${11}"
RECORD_TOPIC="${12}"
RECORD_DIR="${13}"
RECORD_MAX_TOTAL_GB="${14}"

log() {
  echo "[autoaim] $*"
}

if [ ! -d "$WORKDIR" ]; then
  log "ERROR: workspace directory not found inside container: $WORKDIR"
  log "The container must be created with: -v HOST_WS:$WORKDIR"
  exit 1
fi

if [ ! -e "$SYMLINK_TARGET_WS/install/setup.bash" ]; then
  log "ERROR: workspace install not found: $SYMLINK_TARGET_WS/install/setup.bash"
  log "Build the workspace first: colcon build --symlink-install"
  exit 1
fi

cd "$WORKDIR"

source /opt/ros/humble/setup.bash
source "$WORKDIR/install/setup.bash"
set -u

log "inside container"
log "pwd=$(pwd)"
log "package=$LAUNCH_PKG"
log "launch=$LAUNCH_FILE"
log "camera=$CAMERA"
log "health topic=$HEALTH_TOPIC"

log "checking RealSense/USB/video devices inside container..."
ls -l /dev/bus/usb 2>/dev/null || true
ls -l /dev/video* 2>/dev/null || true

log "checking serial inside container..."
ls -l /dev/ttyACM* 2>/dev/null || true

if ! ros2 pkg prefix "$LAUNCH_PKG" >/dev/null 2>&1; then
  log "ERROR: ROS package not found: $LAUNCH_PKG"
  log "available packages containing auto/aim:"
  ros2 pkg list | grep -Ei "auto|aim" || true
  exit 1
fi

if ! ros2 pkg prefix autoaim_realsense >/dev/null 2>&1; then
  log "ERROR: RealSense package not found: autoaim_realsense"
  log "available packages containing realsense/auto/aim:"
  ros2 pkg list | grep -Ei "realsense|auto|aim" || true
  exit 1
fi

if [ -f /tmp/autoaim_launch.pid ]; then
  OLD_PID="$(cat /tmp/autoaim_launch.pid || true)"
  if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
    log "old launch process still running with PID $OLD_PID"
    log "stopping old launch process group"
    kill -INT "-$OLD_PID" 2>/dev/null || kill -INT "$OLD_PID" 2>/dev/null || true
    sleep 3
    kill -TERM "-$OLD_PID" 2>/dev/null || true
  fi
  rm -f /tmp/autoaim_launch.pid
fi

# SIGINT (not TERM/KILL) so ros2 bag finalizes metadata.yaml; a bag killed hard
# still keeps its data but needs `ros2 bag reindex` before it can be read.
if [ -f /tmp/autoaim_bag.pid ]; then
  OLD_BAG_PID="$(cat /tmp/autoaim_bag.pid || true)"
  if [ -n "$OLD_BAG_PID" ] && kill -0 "$OLD_BAG_PID" 2>/dev/null; then
    log "stopping old rosbag recorder PID $OLD_BAG_PID"
    kill -INT "-$OLD_BAG_PID" 2>/dev/null || kill -INT "$OLD_BAG_PID" 2>/dev/null || true
    sleep 2
    kill -TERM "-$OLD_BAG_PID" 2>/dev/null || true
  fi
  rm -f /tmp/autoaim_bag.pid
fi

BAG_PID=""

start_recording_bag() {
  [ "$RECORD_ENABLE" = "1" ] || return 0

  local bag_root="$WORKDIR/$RECORD_DIR"
  mkdir -p "$bag_root"

  # Bags from a hard power-off (no clean service stop) are missing
  # metadata.yaml and unreadable until reindexed. Fix them in the background
  # so every boot leaves the previous match's bag readable.
  local d
  for d in "$bag_root"/rec_*/; do
    if [ -d "$d" ] && [ ! -f "${d}metadata.yaml" ]; then
      log "reindexing bag from unclean shutdown: $d"
      setsid ros2 bag reindex "$d" >>/tmp/autoaim_bag_reindex.log 2>&1 &
    fi
  done

  # Retention: drop oldest recordings until under the cap, so autostart
  # recording can never fill the disk.
  local max_kb=$((RECORD_MAX_TOTAL_GB * 1024 * 1024))
  local oldest
  while [ "$(du -sk "$bag_root" 2>/dev/null | cut -f1)" -gt "$max_kb" ]; do
    oldest="$(ls -1t "$bag_root" | tail -n 1)"
    [ -n "$oldest" ] || break
    log "recording retention: removing old bag $oldest"
    rm -rf "${bag_root:?}/${oldest}"
  done

  # ros2 bag record refuses to start if the output folder already exists
  # (seen after a service restart + clock step: recorder died instantly and
  # nothing was recorded). Pick a unique name, verify the recorder survives
  # startup, and retry with a fresh name instead of failing silently.
  local attempt bag_path
  for attempt in 1 2 3; do
    bag_path="$bag_root/rec_$(date +%Y%m%d_%H%M%S)"
    while [ -e "$bag_path" ]; do
      bag_path="${bag_path}_${RANDOM}"
    done

    log "starting dataset rosbag (attempt $attempt/3): $bag_path ($RECORD_TOPIC)"
    setsid ros2 bag record -o "$bag_path" "$RECORD_TOPIC" /camera_info \
      >/tmp/autoaim_bag.log 2>&1 &
    BAG_PID=$!

    sleep 3
    if kill -0 "$BAG_PID" 2>/dev/null; then
      echo "$BAG_PID" > /tmp/autoaim_bag.pid
      log "rosbag recorder started, PID: $BAG_PID"
      return 0
    fi

    log "ERROR: rosbag recorder died right after start; last log lines:"
    tail -n 5 /tmp/autoaim_bag.log 2>/dev/null || true
  done

  BAG_PID=""
  log "ERROR: dataset recording failed after 3 attempts; launch continues WITHOUT recording"
}

stop_recording_bag() {
  if [ -n "$BAG_PID" ] && kill -0 "$BAG_PID" 2>/dev/null; then
    log "stopping rosbag recorder"
    kill -INT "-$BAG_PID" 2>/dev/null || kill -INT "$BAG_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      kill -0 "$BAG_PID" 2>/dev/null || break
      sleep 1
    done
    kill -TERM "-$BAG_PID" 2>/dev/null || true
  fi
  rm -f /tmp/autoaim_bag.pid
}

check_camera_info_messages() {
  local topic="$1"
  local needed="$2"

  for n in $(seq 1 "$needed"); do
    log "waiting for camera_info message $n/$needed"

    if timeout "$TOPIC_TIMEOUT_SEC" ros2 topic echo --once "$topic" --qos-profile sensor_data >/tmp/autoaim_topic_check.txt 2>&1; then
      log "received camera_info message $n/$needed with sensor_data QoS"
      continue
    fi

    if timeout "$TOPIC_TIMEOUT_SEC" ros2 topic echo --once "$topic" >/tmp/autoaim_topic_check.txt 2>&1; then
      log "received camera_info message $n/$needed with default QoS"
      continue
    fi

    log "failed to receive camera_info message"
    cat /tmp/autoaim_topic_check.txt || true
    return 1
  done

  return 0
}

RECORD_ARGS=()
if [ "$RECORD_ENABLE" = "1" ]; then
  RECORD_ARGS=(record_every:="$RECORD_EVERY")
fi

log "starting launch:"
log "ros2 launch $LAUNCH_PKG $LAUNCH_FILE camera:=$CAMERA ${RECORD_ARGS[*]:-}"

rm -f /tmp/autoaim_launch.pid

setsid ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" camera:="$CAMERA" ${RECORD_ARGS[@]+"${RECORD_ARGS[@]}"} &
LAUNCH_PID=$!

echo "$LAUNCH_PID" > /tmp/autoaim_launch.pid
log "ROS launch started, PID: $LAUNCH_PID"
log "saved launch PID to /tmp/autoaim_launch.pid"

for i in $(seq 1 "$FIRST_HEALTH_ATTEMPTS"); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    log "ERROR: ROS launch died"
    wait "$LAUNCH_PID"
    exit 1
  fi

  log "health check attempt $i/$FIRST_HEALTH_ATTEMPTS"

  log "camera_info topics:"
  ros2 topic list 2>/dev/null | grep camera_info || true

  if check_camera_info_messages "$HEALTH_TOPIC" "$CAMERA_INFO_MESSAGES"; then
    log "first camera_info check passed"

    log "waiting 10 seconds, then checking again..."
    sleep 10

    if check_camera_info_messages "$HEALTH_TOPIC" "$CAMERA_INFO_MESSAGES"; then
      log "OK: camera_info is stable"
      start_recording_bag
      LAUNCH_RC=0
      wait "$LAUNCH_PID" || LAUNCH_RC=$?
      stop_recording_bag
      exit "$LAUNCH_RC"
    fi
  fi

  log "camera_info not stable yet, retrying..."
  sleep 3
done

log "ERROR: camera_info did not become stable, restarting service"
kill -INT "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 2
kill -TERM "-$LAUNCH_PID" 2>/dev/null || true
rm -f /tmp/autoaim_launch.pid
exit 1
INSIDE_CONTAINER
