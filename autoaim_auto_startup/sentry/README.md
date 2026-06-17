# Sentry ZED headless container autostart

This README explains how to create the headless Docker container used for the
Sentry autoaim pipeline with the ZED camera, and how to install the systemd
autostart files.

## 1. Create the headless image

Commit the existing Isaac ROS dev container into a dedicated headless image:

```bash
docker commit isaac_ros_dev-aarch64-container autoaim_headless_image:latest
```

## 2. Create the headless container

Set `HOST_WS` to the real workspace path on the Jetson:

```bash
HOST_WS="/home/roboto/workspaces/isaac_ros-dev"  # change this based on the real path on the Jetson
```

Create the container:

```bash
docker create \
  --name autoaim_headless \
  --network host \
  --ipc host \
  --pid host \
  --privileged \
  --runtime nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=:1 \
  -e QT_QPA_PLATFORM=xcb \
  -v "$HOST_WS:/workspaces/isaac_ros-dev" \
  -v /dev:/dev \
  -v /tmp:/tmp \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /run/udev:/run/udev:ro \
  -v /etc/localtime:/etc/localtime:ro \
  --entrypoint /bin/bash \
  autoaim_headless_image:latest \
  -lc "sleep infinity"
```

## 3. Install the autostart files

To enable automatic startup of the full pipeline, copy the autostart files to
their system locations:

```text
autoaim_autostart.sh       -> /usr/local/bin/
autoaim.service            -> /etc/systemd/system/
enter_autoaim_headless.sh  -> /usr/local/bin/
```

Copy them with:

```bash
sudo cp autoaim_autostart.sh      /usr/local/bin/
sudo cp autoaim.service           /etc/systemd/system/
sudo cp enter_autoaim_headless.sh /usr/local/bin/
```

The service starts the pipeline at every reboot and keeps restarting it if the
`/camera_info` topic stops publishing, so the pipeline stays alive.

Check that the service starts correctly:

```bash
sudo systemctl start autoaim.service
```

## 4. Enable and check the service

Give execution permission, reload systemd, restart the service, and follow the
logs:

```bash
sudo chmod +x /usr/local/bin/autoaim_autostart.sh
sudo systemctl daemon-reload
sudo systemctl restart autoaim.service
sudo journalctl -fu autoaim.service
```

## 5. Manual debugging inside the container

Use this when you need to edit files, run `colcon build`, or relaunch the launch
file manually.

First stop the service. This kills only the ROS launch process inside the
container; the container itself stays alive:

```bash
sudo systemctl stop autoaim.service
```

Then enter the headless container:

```bash
/usr/local/bin/enter_autoaim_headless.sh
```

The enter script sets the video/display variables needed to access the container
and handle the ZED camera.
