# Standard RealSense autoaim autostart on this Jetson

This folder is for this specific Jetson/robot profile:

- robot launch file: `standard.launch.py`
- camera: `realsense`
- health topic: `/camera_info`
- host workspace: `~/workspaces/isaac_ros-dev`
- container name: `autoaim_headless`
- image name: `autoaim_headless_image:latest`

This setup is intentionally RealSense-only. It does not wait for ZED, Argus, or
`zed_x_daemon`, and it does not require X11/Wayland. The service can start in
true headless mode over SSH without a monitor attached.

The standard setup uses a **single workspace mount** only:

    ~/workspaces/isaac_ros-dev -> /workspaces/isaac_ros-dev

There is no second symlink mount because the host path maps directly to the
container path, so all `colcon --symlink-install` absolute symlinks resolve
within the same mount.

## 1. Build the workspace

Build inside the dev container (`isaac_ros_dev-aarch64-container`):

    docker exec -it isaac_ros_dev-aarch64-container bash
    cd /workspaces/isaac_ros-dev
    colcon build --symlink-install

## 2. Create the headless image

Commit the dev container into the autoaim headless image:

    docker commit isaac_ros_dev-aarch64-container autoaim_headless_image:latest

## 3. Recreate the headless container

Stop the service and remove any old container first. Docker mounts cannot be
changed on an existing container.

    sudo systemctl stop autoaim.service || true
    docker rm -f autoaim_headless || true

Create and start the Standard/RealSense headless container:

    HOST_WS=~/workspaces/isaac_ros-dev

    docker create \
      --name autoaim_headless \
      --network host \
      --ipc host \
      --pid host \
      --privileged \
      --runtime nvidia \
      -e NVIDIA_VISIBLE_DEVICES=all \
      -e NVIDIA_DRIVER_CAPABILITIES=all \
      -w /workspaces/isaac_ros-dev \
      -v "$HOST_WS:/workspaces/isaac_ros-dev" \
      -v /dev:/dev \
      -v /tmp:/tmp \
      -v /run/udev:/run/udev:ro \
      -v /etc/localtime:/etc/localtime:ro \
      --entrypoint /bin/bash \
      autoaim_headless_image:latest \
      -lc "sleep infinity"

    docker start autoaim_headless

No second workspace mount is needed. The host path `~/workspaces/isaac_ros-dev`
maps directly to `/workspaces/isaac_ros-dev` inside the container, so all
absolute symlinks resolve within the same mount.

No ZED/X11 mounts are needed for this RealSense service profile.

## 4. Verify the container

Check the workspace mount:

    docker inspect autoaim_headless \
      --format '{{range .Mounts}}{{println .Source "->" .Destination}}{{end}}' \
      | grep isaac_ros-dev

Expected output:

    ~/workspaces/isaac_ros-dev -> /workspaces/isaac_ros-dev

Verify ROS package resolution inside the container:

    docker exec -it -w /workspaces/isaac_ros-dev autoaim_headless bash -lc '
    source /opt/ros/humble/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    ros2 pkg prefix autoaim
    ros2 pkg prefix autoaim_realsense
    '

Both commands must print a path. There must be no `not found` setup warnings.

## 5. Install the autostart files

Copy the files to the system locations:

    sudo cp autoaim_autostart.sh      /usr/local/bin/
    sudo cp enter_autoaim_headless.sh /usr/local/bin/
    sudo cp autoaim.service           /etc/systemd/system/

Set executable permissions and reload systemd:

    sudo chmod +x /usr/local/bin/autoaim_autostart.sh
    sudo chmod +x /usr/local/bin/enter_autoaim_headless.sh
    sudo systemctl daemon-reload

Enable and start the service:

    sudo systemctl enable autoaim.service
    sudo systemctl restart autoaim.service

Follow the logs:

    sudo journalctl -fu autoaim.service

Expected launch command in the logs:

    ros2 launch autoaim standard.launch.py camera:=realsense

Expected success condition:

    [autoaim] OK: camera_info is stable

## 6. Dataset recording (rosbag of camera images)

Every autostart run also records a rosbag of CLEAN camera frames (no detector
overlays) for training-data collection. The images are JPEG-compressed by the
detector before publishing (~10 GB/hour at the default 30 img/s, 848x480 q95 —
visually lossless). Raw uncompressed images would be ~13x larger. The JPEG
encode runs inside the detector frame loop, so keep `RECORD_EVERY >= 2` in
matches; `RECORD_EVERY=1` (60 img/s) steals milliseconds from the aiming loop.

How it works:

- the detector publishes `sensor_msgs/CompressedImage` on
  `/camera/image_raw/compressed` every `RECORD_EVERY` camera frames
  (launch arg `record_every`, passed by `autoaim_autostart.sh`)
- once `camera_info` is stable, the script starts
  `ros2 bag record` on that topic plus `/camera_info` (intrinsics)
- bags land in `~/workspaces/isaac_ros-dev/bags/autostart/rec_YYYYmmdd_HHMMSS`
- the script verifies the recorder is still alive 3 s after starting it and
  retries up to 3 times with a fresh folder name (a name collision after a
  clock step used to kill the recorder instantly and silently)
- bags left without `metadata.yaml` by a hard power-off are reindexed
  automatically at the next boot (log: `/tmp/autoaim_bag_reindex.log`)
- when the folder exceeds `RECORD_MAX_TOTAL_GB` (default 20 GB) the OLDEST
  recordings are deleted automatically, so the disk can never fill up
- recorder log: `/tmp/autoaim_bag.log` inside the container

Tuning (top of `autoaim_autostart.sh`):

    RECORD_ENABLE=1          # 0 disables recording entirely
    RECORD_EVERY=2           # camera fps / N images per second (60/2 = 30 img/s)
    RECORD_MAX_TOTAL_GB=20   # total disk cap for bags/autostart

If the robot is powered off without a clean service stop, the last bag has no
`metadata.yaml`. It is reindexed automatically at the next boot; to read it on
another machine before that, run manually:

    ros2 bag reindex bags/autostart/rec_YYYYmmdd_HHMMSS

Extract the JPEGs from a bag into a folder of .jpg files (run inside the
container; the message payload is already a JPEG, no decoding needed):

```python
import os, sqlite3, sys
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

bag_db, out_dir = sys.argv[1], sys.argv[2]   # rec_.../rec_..._0.db3  out/
os.makedirs(out_dir, exist_ok=True)
db = sqlite3.connect(bag_db)
rows = db.execute(
    "SELECT m.timestamp, m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
    "WHERE t.name='/camera/image_raw/compressed'")
for i, (ts, raw) in enumerate(rows):
    msg = deserialize_message(raw, CompressedImage)
    with open(f"{out_dir}/{ts}_{i:06d}.jpg", "wb") as f:
        f.write(bytes(msg.data))
print("done:", i + 1, "images")
```

## 7. Useful checks on this device

Check service status:

    systemctl --no-pager -l status autoaim.service

Check RealSense USB on the host:

    lsusb | grep -i 8086

Check serial device on the host:

    ls -l /dev/ttyACM*

Enter the container manually with ROS already sourced:

    enter_autoaim_headless.sh

Inside the container, useful checks are:

    ros2 pkg prefix autoaim
    ros2 pkg prefix autoaim_realsense
    ros2 launch autoaim standard.launch.py camera:=realsense
    ros2 topic echo /camera_info --once
    ros2 topic hz /camera_info

## 8. What this profile intentionally does not do

This profile is not the sentry/ZED startup profile. It intentionally does not:

- wait for `nvargus-daemon.service`
- wait for `zed_x_daemon.service`
- require `DISPLAY=:1` or `QT_QPA_PLATFORM=xcb` for launch
- mount `/tmp/.X11-unix`
- launch `sentry.launch.py` or `hero.launch.py`
- pass `camera:=zed`
- use a second workspace mount (unlike hero/sentry)

If this folder is later reused for sentry/ZED, do not just change the launch
file. Reintroduce the ZED/Argus waits and any display/X11 requirements only if
that runtime path actually needs them.

## Install jetson-fan-max service

To keep the Jetson cool during operation, install and enable the fan max service:

```bash
sudo cp jetson-fan-max.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable jetson-fan-max.service
sudo systemctl start jetson-fan-max.service
```
