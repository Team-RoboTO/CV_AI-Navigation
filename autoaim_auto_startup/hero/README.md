# Hero RealSense autoaim autostart on this Jetson

This folder is for this specific Jetson/robot profile:

- robot launch file: `hero.launch.py`
- camera: `realsense`
- health topic: `/camera_info`
- host workspace: `/home/roboto/workspaces/isaac_ros-dev`
- container name: `autoaim_headless`
- image name: `autoaim_headless_image:latest`

This setup is intentionally RealSense-only. It does not wait for ZED, Argus, or
`zed_x_daemon`, and it does not require X11/Wayland. The service can start in
true headless mode over SSH without a monitor attached.

## 1. Build the workspace

The workspace is expected at (in the normal, non headless container):

    /home/roboto/workspaces/isaac_ros-dev

Build as usual with symlink install, inside the normal, non headless container:

    cd /home/roboto/workspaces/isaac_ros-dev
    colcon build --symlink-install

The double mount in the container creation below is required because
`colcon --symlink-install` can create absolute symlinks pointing back to:

    /home/roboto/workspaces/isaac_ros-dev/build/...

Without the second mount, `install/setup.bash` can emit `not found` warnings and
ROS packages such as `autoaim` / `autoaim_realsense` may not resolve.

## 2. Create the headless image

Commit the existing Isaac ROS dev container into the autoaim image:

    docker commit isaac_ros_dev-aarch64-container autoaim_headless_image:latest

## 3. Recreate the headless container

Stop the service and remove any old container first. Docker mounts cannot be
changed on an existing container.

    sudo systemctl stop autoaim.service || true
    docker rm -f autoaim_headless || true

Create and start the Hero/RealSense headless container:

    HOST_WS="/home/roboto/workspaces/isaac_ros-dev"

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
      -v "$HOST_WS:/home/roboto/workspaces/isaac_ros-dev" \
      -v /dev:/dev \
      -v /tmp:/tmp \
      -v /run/udev:/run/udev:ro \
      -v /etc/localtime:/etc/localtime:ro \
      --entrypoint /bin/bash \
      autoaim_headless_image:latest \
      -lc "sleep infinity"

    docker start autoaim_headless

No ZED/X11 mounts are needed for this RealSense service profile.

## 4. Verify the container

Check both workspace mounts:

    docker inspect autoaim_headless \
      --format '{{range .Mounts}}{{println .Source "->" .Destination}}{{end}}' \
      | grep isaac_ros-dev

Expected output:

    /home/roboto/workspaces/isaac_ros-dev -> /workspaces/isaac_ros-dev
    /home/roboto/workspaces/isaac_ros-dev -> /home/roboto/workspaces/isaac_ros-dev

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

    ros2 launch autoaim hero.launch.py camera:=realsense

Expected success condition:

    [autoaim] OK: camera_info is stable

## 6. Useful checks on this device

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
    ros2 launch autoaim hero.launch.py camera:=realsense
    ros2 topic echo /camera_info --once
    ros2 topic hz /camera_info

## 7. What this profile intentionally does not do

This profile is not the sentry/ZED startup profile. It intentionally does not:

- wait for `nvargus-daemon.service`
- wait for `zed_x_daemon.service`
- mount `/tmp/.X11-unix`
- require `DISPLAY=:1`
- use `QT_QPA_PLATFORM=xcb`
- launch `sentry.launch.py`
- pass `camera:=zed`

If this folder is later reused for sentry/ZED, do not just change the launch
file. Reintroduce the ZED/Argus waits and any display/X11 requirements only if
that runtime path actually needs them.
