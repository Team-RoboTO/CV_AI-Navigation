# Jetson Isaac ROS Dev Container Setup

This README explains how to install and replicate the Isaac ROS aarch64 development container on Jetson devices for the RoboTO auto-aim workspace.

Final target layout:

```text
Host:
  /home/roboto/workspaces/isaac_ros-dev

Container:
  /workspaces/isaac_ros-dev

Docker image:
  isaac_ros_dev-aarch64:latest

Docker container:
  isaac_ros_dev-aarch64-container

Container user:
  root

Container startup behavior:
  root starts directly in /workspaces/isaac_ros-dev
  ROS 2 Humble is sourced automatically
  install/setup.bash is sourced automatically if it exists
```

---

## 0. Assumptions

This setup assumes:

```text
Host user: roboto
Host OS: Ubuntu 22.04 on Jetson
JetPack: 6.2.x / L4T r36.4.x
ROS distro inside container: Humble
Workspace root: /home/roboto/workspaces/isaac_ros-dev
```

The workspace is mounted into the container as:

```text
/workspaces/isaac_ros-dev
```

---

## 1. Install host dependencies

Run on the Jetson host, not inside Docker.

```bash
sudo apt update
```

Install required tools:

```bash
sudo apt install -y \
  git \
  docker-ce \
  docker-ce-cli \
  containerd.io \
  docker-buildx-plugin \
  docker-compose-plugin \
  nvidia-container-toolkit \
  v4l-utils \
  usbutils \
  curl \
  wget
```

### Important Docker note

Do **not** install `docker.io` if the official Docker repository is already enabled.

If you see an error like:

```text
containerd.io : Conflicts: containerd
```

it means the system is mixing Ubuntu `docker.io` packages with Docker CE packages. Use Docker CE:

```bash
sudo apt install -y \
  docker-ce \
  docker-ce-cli \
  containerd.io \
  docker-buildx-plugin \
  docker-compose-plugin
```

---

## 2. Enable Docker for the current user

```bash
sudo usermod -aG docker $USER
newgrp docker
```

Check:

```bash
docker ps
```

If this still gives `permission denied`, reboot:

```bash
sudo reboot
```

After reboot:

```bash
docker ps
```

---

## 3. Configure NVIDIA Docker runtime and CDI

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo mkdir -p /etc/cdi
```

Generate NVIDIA CDI config:

```bash
if nvidia-ctk cdi generate --help | grep -q -- "--mode"; then
  sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml
else
  sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
fi
```

Restart Docker:

```bash
sudo systemctl restart docker
```

Verify CDI:

```bash
sudo nvidia-ctk cdi list
```

Expected output should include:

```text
nvidia.com/gpu=0
nvidia.com/gpu=all
```

---

## 4. Create the standard Isaac ROS workspace path

```bash
mkdir -p ~/workspaces
```

The final workspace should be:

```text
~/workspaces/isaac_ros-dev
```

For compatibility with older commands, create a `~/ros_ws` symlink:

```bash
ln -sfn ~/workspaces/isaac_ros-dev ~/ros_ws
```

---

## 5. Clone the RoboTO workspace repository

Use this if the repository contains both `src/` and files outside `src/`.

Replace:

```text
<REPO_URL>
<BRANCH_NAME>
```

with the correct repository URL and branch.

### Clean install

```bash
cd ~/workspaces

git clone -b <BRANCH_NAME> --single-branch <REPO_URL> isaac_ros-dev
```

Example:

```bash
cd ~/workspaces

git clone -b main --single-branch git@github.com:YOUR_TEAM/YOUR_REPO.git isaac_ros-dev
```

### If `~/workspaces/isaac_ros-dev` already exists

Clone into a temporary directory:

```bash
cd ~

git clone -b <BRANCH_NAME> --single-branch <REPO_URL> isaac_ros-dev-tmp
```

Copy into the workspace:

```bash
rsync -av \
  --exclude 'src/isaac_ros_common' \
  --exclude 'src/realsense-ros' \
  isaac_ros-dev-tmp/ ~/workspaces/isaac_ros-dev/
```

Remove the temporary clone:

```bash
rm -rf ~/isaac_ros-dev-tmp
```

Check:

```bash
cd ~/workspaces/isaac_ros-dev
git status
ls
ls src
```

---

## 6. Clone external dependencies

These are external dependencies, not RoboTO code.

```bash
cd ~/workspaces/isaac_ros-dev/src
```

Clone Isaac ROS Common:

```bash
if [ ! -d isaac_ros_common ]; then
  git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
fi
```

Clone RealSense ROS:

```bash
if [ ! -d realsense-ros ]; then
  git clone -b 4.51.1 https://github.com/IntelRealSense/realsense-ros.git
fi
```

Expected layout:

```text
~/workspaces/isaac_ros-dev/
  src/
    autoaim/
    isaac_ros_common/
    realsense-ros/
```

---

## 7. Install RealSense udev rules on host

This is required on the host so the container can access the D455.

```bash
cd /tmp

wget https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.56.3/config/99-realsense-libusb.rules

sudo mv 99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Check:

```bash
ls -l /etc/udev/rules.d/99-realsense-libusb.rules
```

If the D455 is connected:

```bash
lsusb | grep -i -E "intel|realsense"
```

---

## 8. Configure Isaac ROS image key

Use `ros2_humble.realsense` so the Docker image includes the RealSense layer.

This command works in both bash and fish:

```bash
printf 'CONFIG_IMAGE_KEY=ros2_humble.realsense\n' > ~/.isaac_ros_common-config
```

Copy the config into the Isaac ROS scripts directory:

```bash
cp ~/.isaac_ros_common-config \
  ~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts/.isaac_ros_common-config
```

Check:

```bash
cat ~/.isaac_ros_common-config
cat ~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts/.isaac_ros_common-config
```

Expected:

```text
CONFIG_IMAGE_KEY=ros2_humble.realsense
```

Do **not** add:

```text
CONFIG_CONTAINER_NAME_SUFFIX=normal
```

The final image and container should not contain `normal`.

---

## 9. Build the Isaac ROS image

Run:

```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev
```

This may take a long time the first time.

When the container opens, exit:

```bash
exit
```

Check the image:

```bash
docker images | grep isaac_ros_dev
```

Expected image:

```text
isaac_ros_dev-aarch64:latest
```

If you accidentally created:

```text
isaac_ros_dev-aarch64-normal:latest
```

but it has the same image ID as `isaac_ros_dev-aarch64:latest`, remove only the normal tag:

```bash
docker rmi isaac_ros_dev-aarch64-normal:latest
```

---

## 10. Create the persistent root container

Remove any old container:

```bash
docker rm -f isaac_ros_dev-aarch64-container 2>/dev/null || true
```

Create the root container:

```bash
docker run -dit \
  --name isaac_ros_dev-aarch64-container \
  --restart unless-stopped \
  --privileged \
  --network host \
  --ipc host \
  --runtime nvidia \
  --gpus all \
  --workdir /workspaces/isaac_ros-dev \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -v ~/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
  -v /etc/localtime:/etc/localtime:ro \
  isaac_ros_dev-aarch64:latest \
  bash -lc "tail -f /dev/null"
```

Check:

```bash
docker ps
```

Expected container:

```text
isaac_ros_dev-aarch64-container
```

---

## 11. Make root auto-enter the workspace and source ROS

Add auto-setup to `/root/.bashrc` inside the container:

```bash
docker exec -u root isaac_ros_dev-aarch64-container bash -lc 'grep -q AUTOAIM_WORKSPACE_AUTO_SETUP /root/.bashrc || printf "%s\n" "" "# AUTOAIM_WORKSPACE_AUTO_SETUP" "source /opt/ros/humble/setup.bash" "if [ -f /workspaces/isaac_ros-dev/install/setup.bash ]; then" "  source /workspaces/isaac_ros-dev/install/setup.bash" "fi" "cd /workspaces/isaac_ros-dev 2>/dev/null || true" "# END_AUTOAIM_WORKSPACE_AUTO_SETUP" >> /root/.bashrc'
```

Enter the container:

```bash
docker exec -it isaac_ros_dev-aarch64-container bash
```

Check:

```bash
whoami
pwd
```

Expected:

```text
root
/workspaces/isaac_ros-dev
```

---

## 12. Build the workspace inside the container

Inside the container:

```bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

Check:

```bash
ros2 pkg list | grep autoaim
```

If RealSense packages were cloned:

```bash
ros2 pkg list | grep realsense
```

---

## 13. Test RealSense camera

Connect the D455.

On the host:

```bash
lsusb | grep -i -E "intel|realsense"
```

Inside the container:

```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=848x480x60 \
  enable_depth:=false \
  enable_infra1:=false \
  enable_infra2:=false
```

In another container shell:

```bash
docker exec -it isaac_ros_dev-aarch64-container bash
```

Then:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /camera/camera/color/camera_info --once
```

---

## 14. Run autoaim

For the current Jetson 16 launch:

```bash
ros2 launch autoaim j16.launch.py
```

For the full path launch:

```bash
ros2 launch autoaim jetson16/aim.launch.py
```

For RealSense launch, only after the RealSense detector is installed in the `autoaim` package:

```bash
ros2 launch autoaim jetson16/realsense.launch.py
```

---

## 15. Container management

Start container:

```bash
docker start isaac_ros_dev-aarch64-container
```

Enter container:

```bash
docker exec -it isaac_ros_dev-aarch64-container bash
```

Stop container:

```bash
docker stop isaac_ros_dev-aarch64-container
```

Check containers:

```bash
docker ps
docker ps -a
```

Check images:

```bash
docker images | grep isaac_ros_dev
```

---

## 16. Auto-start at boot

The container was created with:

```text
--restart unless-stopped
```

so it should start automatically when Docker starts.

Enable Docker at boot:

```bash
sudo systemctl enable docker
```

Check:

```bash
sudo systemctl status docker --no-pager
```

Reboot test:

```bash
sudo reboot
```

After reboot:

```bash
docker ps
```

Expected:

```text
isaac_ros_dev-aarch64-container
```

Then enter:

```bash
docker exec -it isaac_ros_dev-aarch64-container bash
```

Expected:

```text
root@ubuntu:/workspaces/isaac_ros-dev#
```

---

## 17. Optional: create a headless container without rebuilding

This uses the same image and same workspace mount.

```bash
docker rm -f isaac_ros_dev-aarch64-headless-container 2>/dev/null || true

docker run -dit \
  --name isaac_ros_dev-aarch64-headless-container \
  --restart unless-stopped \
  --privileged \
  --network host \
  --ipc host \
  --runtime nvidia \
  --gpus all \
  --workdir /workspaces/isaac_ros-dev \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY= \
  -e QT_QPA_PLATFORM=offscreen \
  -v /dev:/dev \
  -v ~/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
  -v /etc/localtime:/etc/localtime:ro \
  isaac_ros_dev-aarch64:latest \
  bash -lc "tail -f /dev/null"
```

Add the same root auto-setup:

```bash
docker exec -u root isaac_ros_dev-aarch64-headless-container bash -lc 'grep -q AUTOAIM_WORKSPACE_AUTO_SETUP /root/.bashrc || printf "%s\n" "" "# AUTOAIM_WORKSPACE_AUTO_SETUP" "source /opt/ros/humble/setup.bash" "if [ -f /workspaces/isaac_ros-dev/install/setup.bash ]; then" "  source /workspaces/isaac_ros-dev/install/setup.bash" "fi" "cd /workspaces/isaac_ros-dev 2>/dev/null || true" "# END_AUTOAIM_WORKSPACE_AUTO_SETUP" >> /root/.bashrc'
```

Enter:

```bash
docker exec -it isaac_ros_dev-aarch64-headless-container bash
```

Expected:

```text
root@ubuntu:/workspaces/isaac_ros-dev#
```

---

## 18. Troubleshooting

### Docker permission denied

```bash
sudo usermod -aG docker $USER
newgrp docker
docker ps
```

If still broken:

```bash
sudo reboot
```

---

### CDI GPU error

If Docker fails with:

```text
unresolvable CDI devices nvidia.com/gpu=all
```

Regenerate CDI:

```bash
sudo mkdir -p /etc/cdi

if nvidia-ctk cdi generate --help | grep -q -- "--mode"; then
  sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml
else
  sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
fi

sudo systemctl restart docker
sudo nvidia-ctk cdi list
```

Expected:

```text
nvidia.com/gpu=0
nvidia.com/gpu=all
```

---

### Wrong workspace path

Correct host path:

```text
/home/roboto/workspaces/isaac_ros-dev
```

Correct container path:

```text
/workspaces/isaac_ros-dev
```

If needed, recreate compatibility symlink:

```bash
ln -sfn ~/workspaces/isaac_ros-dev ~/ros_ws
```

---

### Fish shell and heredoc errors

Fish may fail on commands like:

```bash
cat > file <<'EOF'
...
EOF
```

Use `printf` instead:

```bash
printf 'CONFIG_IMAGE_KEY=ros2_humble.realsense\n' > ~/.isaac_ros_common-config
```

---

### RealSense not detected

Check on host:

```bash
lsusb | grep -i -E "intel|realsense"
```

Check udev rule:

```bash
ls -l /etc/udev/rules.d/99-realsense-libusb.rules
```

If missing:

```bash
cd /tmp

wget https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.56.3/config/99-realsense-libusb.rules

sudo mv 99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug the D455.

---

### TensorRT engine compatibility

TensorRT `.engine` files are device-specific.

If an engine was built on another Jetson, rebuild it on the target Jetson from the `.onnx`.

Do not assume a Jetson 64 engine will deserialize on a Jetson 16.

---

## 19. Final expected state

Host:

```text
/home/roboto/workspaces/isaac_ros-dev
/home/roboto/ros_ws -> /home/roboto/workspaces/isaac_ros-dev
```

Docker:

```text
Image:
  isaac_ros_dev-aarch64:latest

Container:
  isaac_ros_dev-aarch64-container
```

Container entry:

```bash
docker start isaac_ros_dev-aarch64-container
docker exec -it isaac_ros_dev-aarch64-container bash
```

Expected prompt:

```text
root@ubuntu:/workspaces/isaac_ros-dev#
```

Expected ROS state:

```bash
ros2 pkg list | grep autoaim
```

Expected workspace:

```text
/workspaces/isaac_ros-dev
```
