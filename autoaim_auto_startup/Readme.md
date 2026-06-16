To allow the auto startup of the container isaac_ros_dev-aarch64, the startup of the all pipeline it's needed to put the 2 files in the following folder:
    autoaim_autostart.sh goes inside /usr/local/bin/
    autoaim.service goes inside /etc/systemd/system/
then check if it works with sudo systemctl start autoaim.service, in this way at each reboot the service starts automatically and run all the components (it also keep rerunning it if it sees that the camera_info topic is not publishing to ensure that the pipeline is working)
