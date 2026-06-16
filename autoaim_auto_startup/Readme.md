Creation of the healess container for autostart

create the healeass container:

docker commit isaac_ros_dev-aarch64-container autoaim_headless_image:latest

create the container:

HOST_WS="/home/roboto/workspaces/isaac_ros-dev" -> change it based on the real path on the jetson

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


To allow the auto startup of the container isaac_ros_dev-aarch64, the startup of the all pipeline it's needed to put the 2 files in the following folder:
    autoaim_autostart.sh goes inside /usr/local/bin/
    autoaim.service goes inside /etc/systemd/system/
then check if it works with sudo systemctl start autoaim.service, in this way at each reboot the service starts automatically and run all the components (it also keep rerunning it if it sees that the camera_info topic is not publishing to ensure that the pipeline is working)


Give the permission and run the service to check if all it's working:
sudo chmod +x /usr/local/bin/autoaim_autostart.sh
sudo systemctl daemon-reload
sudo systemctl restart autoaim.service
sudo journalctl -fu autoaim.service


if is needed to fix inside the container the files, colcon build, and relaunching manually the launch file:

sudo systemctl stop autoaim_handless (it kills only the node inside, the container still alive)
and run
./usr/local/bin/enter_autoaim_headless.sh (it set all the video variables needed to go inside the container and handle the camera)

