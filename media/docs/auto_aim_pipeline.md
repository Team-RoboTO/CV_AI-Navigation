# Auto-aim Pipeline

High-level walkthrough of the data flow inside the `auto_aim` node. This
document is descriptive, not prescriptive: read it to understand what the
code does today.

## Subscribed topics

`auto_aim_node` consumes YOLO-pose armor keypoints exclusively. The legacy
bbox-only callback was removed in this branch — there is no `use_keypoints`
flag any more, and `auto_aim` does not subscribe to `/detector/armors`.
The detector still publishes that bbox topic for legacy debug consumers
(e.g. RViz/viewer), but it is not part of the runtime measurement chain.

```
/detector/armors_keypoints  auto_aim/ArmorKeypointArray  required
/camera_info                sensor_msgs/CameraInfo       required
/micro_imu                  std_msgs/Float32MultiArray   required, [yaw_rad, pitch_rad]
```

## Published topics

Detector outputs:

```
/detector/armors            vision_msgs/Detection2DArray
/detector/armors_keypoints  auto_aim/ArmorKeypointArray
/yolo/debug_image           sensor_msgs/Image              optional raw debug copy
```

Auto-aim outputs:

```
/tracker/cmd_gimbal   geometry_msgs/Twist          (firmware contract)
/cmd_vel_AI           geometry_msgs/Twist          (legacy compatibility)
/tracker/aim_pixels   geometry_msgs/Twist          (HUD overlay)
/tracker/marker       visualization_msgs/MarkerArray (RViz)
/auto_aim/debug       auto_aim/AutoAimDebug        (per-frame structured debug, P1+)
```

## Launch Bring-Up

`ros2 launch launch_pkg debug_targeting.launch.py` starts three runtime
pieces:

1. Intel RealSense color stream at `/camera/camera/color/image_raw` and
   camera info at `/camera/camera/color/camera_info`.
2. `auto_aim/scripts/yolo26_pose_realsense_node.py`, installed as the
   executable `auto_aim yolo26_pose_realsense_node.py`.
3. `auto_aim_node`, installed as `auto_aim auto_aim_node`.

The TensorRT engine belongs in `launch_pkg/resources`:

```
src/launch_pkg/resources/yolov26_keypoints.engine
```

After `colcon build --symlink-install --packages-select launch_pkg`, the
default launch argument resolves to:

```
install/launch_pkg/share/launch_pkg/resources/yolov26_keypoints.engine
```

`launch_pkg/package.xml` declares `auto_aim` as an `exec_depend` because this
launch file directly starts `auto_aim` executables.

For bench Jetsons without the gimbal microcontroller,
`debug_targeting.launch.py` also publishes a fake `/micro_imu` by default:

```
std_msgs/Float32MultiArray {data: [0.0, 0.0]}
```

That means the camera is treated as a fixed, level gimbal pose so the rest of
the pipeline can run. On the real robot, launch with
`use_fake_micro_imu:=false`; otherwise the fake publisher will fight the
firmware publisher.

## Per-frame data flow

```
+-----------------------------+
| RealSense color image       |
+--------------+--------------+
               |
               v
+-----------------------------+
| YOLOv26-pose detector       |
|   TensorRT + CuPy CUDA      |
|   class scores + 4 corners  |
+--------------+--------------+
               |
               | ArmorKeypointArray
               v
+-----------------------------+
| Keypoint gate               |
|   filter target classes     |
|   reject low-score corners  |
|   reject non-finite corners |
|   convex+winding sanity chk |
+--------------+--------------+
               |
               v
+-----------------------------+
| PnPSolver::solveKeypoints   |
|   TL,TR,BR,BL pixels (in)   |
|   IPPE PnP, small/large     |
|   keep min reproj err       |
|   reproj_err_norm = err /   |
|     keypoint AABB diagonal  |
+--------------+--------------+
               |  (rvec, tvec, is_large, reproj_err, image_points)
               v
+-----------------------------+
| FrameTransformer            |
|   camera optical -> odom    |
|   using imu_rotation +      |
|   imu_translation           |
|   yaw flip + obliquity clamp|
+--------------+--------------+
               |  ArmorDetection in odom
               v
+-----------------------------+
| Tracker (EKF, state machine)|
|   state: [xc vxc yc vyc za  |
|           vza yaw vyaw r]   |
|   LOST/DETECTING/TRACKING/  |
|   TEMP_LOST                 |
|   target switching          |
+--------------+--------------+
               |  EKF state
               v
+-----------------------------+
| AimPlanner                  |
|   predict 4 faces at        |
|   impact time               |
|   pick best by margin+vis   |
+--------------+--------------+
               |  selected face in odom
               v
+-----------------------------+
| BallisticsSolver            |
|   barrel-origin geometry    |
|   gravity-only iter solve   |
+--------------+--------------+
               |  abs_yaw, abs_pitch
               v
+-----------------------------+
| FireGate                    |
|   tracker state ok?         |
|   range ok?                 |
|   angular margin ok?        |
|   smoothing lag ok? (opt-in)|
|   reason enum on block      |
+--------------+--------------+
               |  fire/hold + reason
               v
+-----------------------------+
| CommandPublisher            |
|   bore-sight subtract       |
|   EMA smooth                |
|   sign flip                 |
|   publish Twist             |
|   publish legacy Twist      |
|   publish aim pixels        |
+--------------+--------------+
               |
               v
       /tracker/cmd_gimbal
       /cmd_vel_AI
       /tracker/aim_pixels
       /tracker/marker
       /auto_aim/debug
```

## Measurement-model contract

The pipeline has exactly one measurement model: YOLO-pose keypoints. The
`ArmorKeypoint.msg` schema fixes the keypoint index order:

```
keypoints_xy = [TLx, TLy, TRx, TRy, BRx, BRy, BLx, BLy]   image pixels
```

`PnPSolver::init()` builds the matching armor-frame object points in the
exact same order:

```
TL = (0, +hy, +hz)    image-left,  image-up
TR = (0, -hy, +hz)    image-right, image-up
BR = (0, -hy, -hz)    image-right, image-down
BL = (0, +hy, -hz)    image-left,  image-down
```

with the X axis as plate normal (out of the plate toward the viewer),
Y as image-left when face-on, Z as image-up. PnP correspondence is
index-by-index — the i-th 2D keypoint maps to the i-th 3D point above.
**There is no runtime reordering.** Operators must verify on a known
frame that keypoint 0 in `kp_image_points` is in the upper-left quadrant
of the armor and that the indices walk clockwise. See
`docs/calibration_guide.md` for the verification overlay.

Detector keypoints are expected in original image pixel coordinates after
letterbox unpadding/rescaling. The detector should not clip keypoints before
publishing, because clipping can make PnP solve a distorted quadrilateral.
`auto_aim_node` rejects out-of-image keypoints as `KP_OUT_OF_IMAGE`.

The C++ side defends against ordering mistakes only with a convex+winding
check (`enable_keypoint_geometry_check`, default ON): a self-intersecting
or non-convex quad is rejected as `KP_INVALID_GEOMETRY`. That stops bow-tie
configurations but **does not detect a 90° rotation of the index labels**
— that case must be caught visually before competition.

## EKF state and observation

State vector (9 elements):

```
x_(0) = xc        robot center x   [m, odom]
x_(1) = vxc       robot center vx  [m/s]
x_(2) = yc        robot center y   [m]
x_(3) = vyc       robot center vy  [m/s]
x_(4) = za        armor height     [m]
x_(5) = vza       armor vz         [m/s]
x_(6) = yaw       armor face yaw   [rad]
x_(7) = vyaw      yaw rate         [rad/s]
x_(8) = r         armor radius     [m]
```

Observation (4 elements):

```
z(0) = xa = xc - r * cos(yaw)
z(1) = ya = yc - r * sin(yaw)
z(2) = za
z(3) = yaw
```

The "robot center" (xc, yc) is the spinning-top center, and the armor sits
on a circle of radius `r` around it. The four faces are at
`yaw, yaw + pi/2, yaw + pi, yaw + 3pi/2`. Pairs of faces (front/back,
left/right) can have different radii (`radius_`, `other_radius_`) and a
height offset `dz_` for the alternate pair.

## Aim planning

`AimPlanner::computeAim()` runs two passes:

1. Pick a rough flight time using `pred_lead_extra + 1.5 / bullet_speed`,
   where `pred_lead_extra` is either `cfg.time_bias` (legacy) or the value
   passed via the `override_pred_lead_s` argument (P7+).
2. For each of four faces, compute future face position at impact time,
   the barrel-to-face geometry, the ballistic pitch (gravity-only), and
   a margin against the angular-window threshold. Pick the face with the
   largest margin (tie-break: largest visibility = `cos(obliquity)`).
3. Re-run with the best face's flight time. The `pred_lead_extra` term is
   carried across both iterations — earlier code reset it to `time_bias`
   on iter 1, silently dropping any measured-latency contribution.
4. The committed `pred_t` is logged on `/auto_aim/debug.pred_t_total_s`
   broken into components (`pred_lead_measured_s`, `pred_lead_gimbal_s`,
   `pred_lead_time_bias_s`, `pred_lead_ema_s`, `pred_flight_time_s`).

After P7, `cfg.time_bias` is replaced by measured pipeline latency
(`latency_estimate_ema_s + gimbal_response_delay_s`) when
`use_measured_latency=true`. With `enable_ema_delay_compensation=true` the
planner additionally adds the steady-state EMA smoother delay
`dt * (1 - cmd_smooth_alpha) / cmd_smooth_alpha` (clamped to
`ema_delay_max_s`) to compensate for the lag introduced by the published
command's exponential moving average.

## Timing model

Every detection callback the node logs a strict timing chain on
`/auto_aim/debug`:

```
image_stamp ──capture_to_process_s──▶ proc_start
proc_start  ──process_s─────────────▶ proc_end
proc_end    ──process_to_publish_s──▶ publish
total_s     = image_stamp -> publish
estimate_ema_s = EMA(total_s) used as the next-frame measured latency
```

The EKF state is anchored at `image_stamp`. `computeAim()` predicts
forward by `pred_t_total_s`, which is the time we expect between the
image being captured and the bullet reaching the target:

```
pred_t_total_s = pred_lead_measured_s            (use_measured_latency)
              + pred_lead_gimbal_s              (use_measured_latency)
              + pred_lead_time_bias_s           (legacy fallback)
              + pred_lead_ema_s                 (enable_ema_delay_compensation)
              + pred_flight_time_s              (always)
```

When all flags are off, this collapses to the legacy
`cfg.time_bias + flight_time` for backward compatibility.

### Why no EKF rewind/replay yet

The EKF currently advances state to the latest detection's image stamp
synchronously, so prediction lead has to cover the *entire* gap between
the capture and the bullet impact. A rewind/replay scheme would keep a
short ring buffer of EKF states, apply each new measurement at its
captured-time anchor (rewinding then re-predicting), and finally roll
the state forward to `now + barrel_response`. That removes the need for
a measured-latency lead but has hard requirements:

* a state history ring sized to cover the worst-case detection delay,
* an inverse predict step (we currently do not have one),
* a replay-safe match-association path (the current Mahalanobis gate
  uses the latest covariance — replay would need the historical one),
* explicit handling of the radius EMA, target switching, and yaw unwrap
  during rewind so the state is identical regardless of out-of-order
  measurement arrival.

See `docs/design_notes/ekf_rewind_replay.md` for the full design sketch.
This step is intentionally *not* implemented; the current code just
makes the timestamp semantics observable so a future rewind change has
ground truth to test against.

## Fire gate

Fire is granted only if:

* Tracker is in `TRACKING`.
* Range is within `[min_fire_dist, max_fire_dist]`.
* Selected face's angular margin is non-negative.
* Selected face center is inside the camera-frame angular window.
* (Diagnostic, off by default) Smoothed command is close enough to the
  commanded ballistic ray. EMA smoothing on a moving target leaves a
  near-constant phase lag, so this gate would block fire indefinitely
  while tracking — keep it disabled in competition. Enable only for
  acquisition behaviour or static-rig QA via
  `fire.enable_smoothing_gate=true`.
* (P9+) Anti-gyro timing residual is within tolerance, when enabled.

A short hysteresis keeps fire enabled across small margin dips so the
firmware does not see fire flicker on noisy frames.

## Reference frames

* **camera optical**: standard ROS 2 convention, +Z forward, +X right, +Y down.
* **gimbal/body**: `+X` forward, `+Y` left, `+Z` up.
* **odom (micro reference)**: same orientation as gimbal at startup, fixed in
  the world. `imu_rotation_` is the rotation that takes gimbal-yaw/pitch into
  the camera optical frame. `imu_translation_` lifts the camera to the gimbal
  height.

The conversion in `microImuCallback`:

```
q_gimbal = setRPY(0, pitch, yaw)   // body -> world from current yaw/pitch
q_conv   = setRPY(-pi/2, 0, -pi/2) // body -> camera optical (constant)
imu_rotation_     = q_gimbal * q_conv
imu_translation_  = (0, 0, gimbal_height)
```

`p_odom = imu_rotation_ * p_camera + imu_translation_`.

## Why we do **not** use TF

The microcontroller publishes its own absolute yaw/pitch on `/micro_imu`
and the auto-aim node publishes commands in that same reference. Routing
through `/tf` would add transport jitter and a frame configuration that
must stay identical to the firmware view. The current code keeps the
camera-to-odom transform in-process from `/micro_imu`. TF can be added as
a debug visualization later, but never as the source of truth for the
runtime command path.
