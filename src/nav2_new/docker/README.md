# Docker — Multi-container setup for Nav + CV

## Recommended architecture

**Keep Nav and CV in SEPARATE containers.** Reasons:

| Concern | Same container | Separate (recommended) |
|---|---|---|
| Dependency isolation | CUDA + ROS Nav conflicts possible | Clean — each image has only what it needs |
| Rebuild time when changing CV | Rebuilds entire Nav stack too | Just the CV image |
| GPU resource conflict | Both compete for GPU | CV owns GPU; Nav runs on CPU only |
| Image size | 15–20 GB combined | ~4 GB Nav + ~8 GB CV |
| Restart one without the other | Full restart | Independent |

**Both containers share the host network** (`--net=host`) and the same `ROS_DOMAIN_ID`, so topics like `/micro_status`, `/cv/target`, and `/turret/cmd` flow between them transparently as if they were on a single host.

## Data flow

```
┌─────────────────────────────────┐     ┌─────────────────────────────────┐
│  CV container                    │     │  Nav container (this repo)       │
│                                  │     │                                  │
│  • camera driver                 │     │  • Livox driver                  │
│  • detection                     │     │  • FAST_LIO                      │
│  • shooter control               │     │  • Nav2 + AMCL                   │
│  • STM32 serial driver           │     │  • waypoint_manager              │
│                                  │     │  • turret_yaw_mux                │
│  publishes:                      │     │  subscribes:                     │
│   /micro_status    ────────────────────► /micro_status                    │
│   /cv/target       ────────────────────► /cv/target                       │
│                                  │     │                                  │
│  subscribes:                     │     │  publishes:                      │
│   /turret/cmd    ◄─────────────────────  /turret/cmd                      │
│                                  │     │   (via turret_yaw_mux)           │
└─────────────────────────────────┘     └─────────────────────────────────┘
                            │                         │
                            └─ ROS 2 DDS over host ──┘
                                (--net=host, same ROS_DOMAIN_ID)
```

## Setup

### 1. Build the Nav image

From the repo root:

```bash
docker build -t roboto/nav:latest -f docker/Dockerfile.jetson .
```

### 2. Pick a `ROS_DOMAIN_ID`

A number 0–232 that is NOT in use elsewhere on your subnet. Put it in both containers' run commands:

```bash
export ROS_DOMAIN_ID=42   # same in both containers!
```

### 3. Run Nav container

Use `scripts/run_nav_docker.sh` (provided) or manually:

```bash
xhost +local:docker

docker run -it --rm --privileged --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="ROS_DOMAIN_ID=42" \
  --env="RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/roboto_maps:/root/roboto_maps" \
  --device /dev/dri:/dev/dri \
  --name roboto_nav \
  roboto/nav:latest
```

### 4. Run CV container (yours, separately)

In the CV container's run command, add:

```bash
--env="ROS_DOMAIN_ID=42" \
--env="RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
--net=host \
```

### 5. Verify they can talk

Inside the Nav container:

```bash
ros2 topic list
# You should see topics from BOTH containers
```

If you only see Nav topics, the domain IDs don't match or one of them isn't using `--net=host`.

## Troubleshooting

**Topics not crossing between containers** → Both containers must use `--net=host` AND the same `ROS_DOMAIN_ID`. Check with `env | grep ROS_DOMAIN_ID` inside each.

**GUI not showing** (RViz) → On the host: `xhost +local:docker`. Inside the container: `echo $DISPLAY` should match host. If using SSH, add `ssh -X`.

**Livox driver "no permission" on the ethernet interface** → The driver needs to set the Jetson's IP on the LAN port. This requires `--privileged` and `--net=host`. You'll also need to tell the driver which interface to use — see the Livox network config section in the main README.

**Different RMW implementations in the two containers** → Force both to `rmw_fastrtps_cpp` (the default for Humble) or both to `rmw_cyclonedds_cpp`. Mixing DDS vendors is possible but flaky.
