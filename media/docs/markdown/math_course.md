# Mathematics Course

## What The Math Is For

The auto-aim system answers one question many times per second:

> Given a camera image, the current gimbal yaw and pitch, and a moving target,
> what absolute yaw and pitch should the gimbal move to, and is it safe to fire?

The repository solves that question as a chain of smaller math problems:

1. A detector reports a 2D rectangle around an armor plate.
2. PnP estimates where that plate is in 3D camera coordinates.
3. A frame transform moves the 3D point into the robot's odom frame.
4. An Extended Kalman Filter estimates the enemy robot center, velocity, yaw,
   yaw rate, and armor radius.
5. The aim planner predicts which of four armor faces should be hit at bullet
   impact time.
6. The ballistic solver computes the pitch needed to compensate gravity.
7. The fire gate checks whether the command is accurate enough to shoot.

Each step has its own units. Mixing units is one of the fastest ways to break
this project. In this package, distances are meters unless a comment says
millimeters, angles are radians unless a parameter name ends with `_deg`, and
image coordinates are pixels.

## Math Notation For Beginners

A scalar is one number, such as `3.0` meters. A vector is a list of numbers,
such as a 3D position:

```text
p = [x, y, z]^T
```

A matrix is a rectangular table of numbers. Multiplying a matrix by a vector
can rotate, scale, or mix the vector components. A rotation matrix `R` and a
translation vector `t` transform a point from one frame to another:

```text
p_world = R p_camera + t
```

The code often uses the same idea with quaternions instead of explicit rotation
matrices. A quaternion is another way to store a 3D rotation. You do not need
to manually multiply quaternions at first; you need to know that
`tf2::quatRotate(q, p)` rotates point `p` by rotation `q`.

## Coordinate Frames

A coordinate frame defines what the axes mean. The same physical armor can
have different coordinates in different frames.

| Frame | Meaning in this repository |
| --- | --- |
| Camera optical | ROS camera convention: `+Z` forward from the camera, `+X` right in the image, `+Y` down in the image. PnP outputs points here. |
| Gimbal body | Mechanical convention: `+X` forward, `+Y` left, `+Z` up. The barrel offset is measured in this frame. |
| Odom / micro reference | Fixed startup reference used by the microcontroller. The node publishes absolute yaw and pitch commands in this frame after sign conversion. |

In `src/src/frame_transformer.cpp`, the current IMU yaw and pitch build a
rotation:

```text
q_camera_to_odom = q_gimbal q_body_to_camera
```

Then a PnP point is transformed as:

```text
p_odom = q_camera_to_odom p_camera + [0, 0, h_gimbal]^T
```

The code form is:

```cpp
tf2::Vector3 p_odom = tf2::quatRotate(rotation_, p_cam) + translation_;
```

### Yaw And Pitch

Yaw turns left/right around the vertical axis. Pitch turns up/down. The command
published on `/tracker/cmd_gimbal` is not a small change. It is an absolute
destination:

```text
command = absolute yaw target in radians
command = absolute pitch target in radians
```

The parameters `gimbal.yaw_sign` and `gimbal.pitch_sign` convert between the
code's internal convention and the microcontroller's convention. They must be
exactly `+1` or `-1`.

## Camera Projection

A camera maps a 3D point to a 2D pixel. Ignoring distortion for the moment:

```text
u = fx X / Z + cx
v = fy Y / Z + cy
```

Here `(X, Y, Z)` is a point in camera optical coordinates, `(u, v)` is the
pixel, `fx` and `fy` are focal lengths in pixels, and `(cx, cy)` is the optical
center in pixels. These values come from `/camera_info`.

The code also uses the inverse idea for the HUD overlay. If the aim ray has a
small yaw and pitch in the camera frame, the pixel is:

```text
u = cx + fx tan(theta_yaw)
v = cy - fy tan(theta_pitch)
```

This appears in `src/src/auto_aim_node.cpp` when publishing
`/tracker/aim_pixels`.

## PnP: From 2D Corners To 3D Pose

PnP means "Perspective-n-Point". It estimates the camera-frame pose of a known
3D object from corresponding 3D object points and 2D image points.

The current `debug_targeting.launch.py` path gets exact armor corner estimates
from the YOLOv26-pose detector as TL, TR, BR, BL keypoints. Those four image
points go directly into `PnPSolver::solveKeypoints`.

The older bbox fallback does not get exact armor corners. It gets a bounding
box: center `(cx, cy)`, width `w`, and height `h`. The PnP solver builds a
synthetic rectangle inside that box:

```text
x_left  = cx - light_ratio * w / 2
x_right = cx + light_ratio * w / 2
y_top   = cy - light_ratio * h / 2
y_bottom= cy + light_ratio * h / 2
```

The four image points are:

```text
(x_left, y_bottom)
(x_left, y_top)
(x_right, y_top)
(x_right, y_bottom)
```

The 3D object model is a flat armor plate. The solver tries both dimensions:

- small armor: `140 mm` by `125 mm`
- large armor: `235 mm` by `127 mm`

It keeps the model with lower reprojection error. Reprojection error means:

1. Estimate the pose.
2. Project the known 3D corners back into the image.
3. Measure the average pixel distance between projected corners and input
   corners.

The code rejects a PnP result when the mean reprojection error is too large.
The debug topic also reports a normalized error:

```text
e_norm = e_pixels / sqrt(w^2 + h^2)
```

This makes errors easier to compare across targets of different pixel size.

## Armor Yaw And Obliquity

The PnP solver estimates both position and orientation. Orientation can flip
for flat rectangles because two poses can project similarly into the image. The
frame transformer protects the tracker from a bad plate normal.

Let the transformed armor center be `(x, y, z)` in odom. The bearing from the
robot to the armor is:

```text
theta_bearing = atan2(y, x)
```

If the PnP yaw points the wrong way, the code adds `pi`. If the yaw is still
too far from the bearing by more than `max_oblique_deg`, the code clamps the
yaw to the bearing. This means the position can still be used even when the
plate orientation is unreliable.

## Tracker State

The tracker models the enemy robot as a rotating body with armor plates on a
circle around the robot center. The EKF state vector has nine values:

```text
x = [xc, vxc, yc, vyc, za, vza, yaw, vyaw, r]^T
```

| Symbol | Meaning | Unit |
| --- | --- | --- |
| `xc, yc` | enemy robot center position in odom | m |
| `vxc, vyc` | enemy robot center horizontal velocity | m/s |
| `za` | armor height | m |
| `vza` | armor vertical velocity | m/s |
| `yaw` | visible armor face yaw | rad |
| `vyaw` | armor yaw rate | rad/s |
| `r` | radius from enemy center to armor plate | m |

The measurement vector is:

```text
z = [xa, ya, za, yaw]^T
```

where `(xa, ya, za)` is the detected armor center in odom.

## Measurement Model

The tracker assumes the armor is offset from the enemy center by radius `r`:

```text
xa = xc - r cos(yaw)
ya = yc - r sin(yaw)
za = za
yaw = yaw
```

In EKF notation this is:

```text
z_pred = h(x)
```

The innovation is the difference between what was measured and what the
current state predicted:

```text
y = z - h(x)
```

A small innovation means the new detection agrees with the tracker. A large
innovation means the detection may be noisy, a different target, or a different
armor face after the enemy rotated.

## Why The Tracker Is An EKF

A normal Kalman filter works when the equations are linear. This tracker has
`sin(yaw)` and `cos(yaw)`, so the measurement model is nonlinear. An Extended
Kalman Filter solves this by linearizing the nonlinear equation near the
current estimate.

The linearization is the Jacobian matrix `H`:

```text
H = dh/dx
```

For the first two measurement rows:

```text
d xa / d xc  = 1
d xa / d yaw = r sin(yaw)
d xa / d r   = -cos(yaw)

d ya / d yc  = 1
d ya / d yaw = -r cos(yaw)
d ya / d r   = -sin(yaw)
```

These are exactly the non-zero entries built in `src/src/tracker.cpp` inside
`Tracker::ekfUpdate`.

## Prediction Step

Between frames, the target is predicted with a damped constant-velocity model.
For a position and velocity pair:

```text
p_new = p + b v dt
v_new = b v
```

The damping coefficient `b` is computed from `alpha_pos`, `alpha_yaw`,
`alpha_coast`, the frame time `dt`, and `ref_freq`. This keeps damping
approximately consistent even if the detector rate changes.

The covariance matrix `P` stores uncertainty. During prediction it becomes:

```text
P_new = F P F^T + Q
```

`F` is the Jacobian of the motion model. `Q` is process noise. Larger `q_pos`
or `q_yaw` makes the tracker respond faster but jitter more. Smaller values
make it smoother but slower.

## Update Step

When a detection is associated with the current track, the Kalman update is:

```text
S = H P H^T + R
K = P H^T S^-1
x_new = x + K y
```

`R` is measurement noise. In this repository, `R` grows with target distance.
The code also increases yaw and position noise when the armor is viewed
obliquely, because a nearly edge-on plate gives weaker orientation information.

The covariance update uses Joseph form:

```text
P_new = (I - K H) P (I - K H)^T + K R K^T
```

Joseph form costs a little more computation but is more numerically stable.

## Mahalanobis Distance

The tracker must decide whether a detection belongs to the current target. It
uses Mahalanobis distance:

```text
d^2 = y^T S^-1 y
```

This is better than plain Euclidean distance because it accounts for
uncertainty. A detection far from a very uncertain prediction may still be
reasonable. A detection close to a very certain prediction can be suspicious if
the covariance says it should not move there.

The parameter `maha_threshold` is the gate. If the distance is above this
threshold, the detection is not associated with the current track.

## Tracker State Machine

The tracker has four states:

| State | Meaning |
| --- | --- |
| `LOST` | No target is trusted. The next valid detection initializes a new track. |
| `DETECTING` | A possible target has been seen but needs consecutive matches before commands are trusted. |
| `TRACKING` | The target is locked. The node may publish nonzero yaw/pitch and the fire gate may allow shooting. |
| `TEMP_LOST` | The tracker had a target but missed recent detections. It coasts briefly before becoming `LOST`. |

The purpose of this state machine is to avoid commanding the gimbal from a
single accidental detection.

## Armor Face Switching

When an enemy spins, the visible armor face changes. The tracker detects a
large yaw jump. A jump near `90 deg` means the visible face changed to the
alternate pair. The code swaps `radius_` and `other_radius_`, updates the
height offset `dz_`, and then performs a normal EKF update from the new face.

This is why the tracker state includes the enemy center and radius instead of
only tracking the visible armor point. The center lets the code predict all
four plates.

## Aim Planning

The aim planner predicts four possible future armor faces:

```text
yaw_i(t) = yaw + vyaw * t + i*pi/2,  i in {0,1,2,3}
```

The enemy center is also predicted:

```text
xc(t) = xc + vxc * t
yc(t) = yc + vyc * t
zc(t) = za + vza * t
```

For face `i`, the face center is:

```text
xi = xc(t) - ri cos(yaw_i(t))
yi = yc(t) - ri sin(yaw_i(t))
zi = zc(t) + dzi
```

The planner evaluates all four faces and chooses the best one. The first pass
uses a rough prediction time:

```text
t_pred = time_bias + 1.5 / bullet_speed
```

or measured latency when enabled. The second pass reuses the best face's
computed bullet flight time.

## Barrel Origin And Parallax

The aim vector starts at the barrel, not at the camera. This matters at close
range. A `10 cm` camera-to-barrel offset at `1 m` is a large angular error.

The barrel offset is measured in the gimbal body frame:

```text
o_barrel = [ox, oy, oz]^T
```

The code rotates horizontal offset by current yaw:

```text
bx = ox cos(theta) - oy sin(theta)
by = ox sin(theta) + oy cos(theta)
bz = h_gimbal + oz
```

Then the aim vector is face position minus barrel position.

## Ballistics

Without gravity, pitch would be:

```text
pitch = atan2(dz, ground_distance)
```

where `ground_distance` is horizontal distance and `dz` is vertical target
difference. Gravity makes the bullet drop during flight, so the barrel must
aim higher.

The solver iterates:

```text
vx = bullet_speed * cos(pitch)
t = ground_distance / vx
pitch = atan2(dz + 0.5 * gravity * t^2, ground_distance)
```

After five iterations, it returns pitch and flight time. It rejects impossible
geometry such as nearly vertical shots or a pitch magnitude above about `1.2`
radians.

## Fire Margin

For each predicted face, the planner computes an angular window:

```text
window = angular_window * min(window_ref_dist / max(range, 0.5), 2.0)
```

The error is:

```text
error = abs(shortest_angular_distance(bearing, face_yaw))
```

The margin is:

```text
margin = window - error
```

If the margin is positive, the selected face is aligned enough according to
the planner. The fire gate still performs more checks before the actual fire
flag is published.

## Anti-Gyro Timing

When anti-gyro mode is enabled and the enemy yaw rate is high, face alignment
timing matters. The residual is:

```text
residual = shortest_angular_distance(face_yaw, bearing) / vyaw
```

Its unit is seconds. A residual near zero means the bullet impact time and the
face alignment time match. The fire gate can block shots when
`abs(residual)` is larger than `fire.anti_gyro_max_residual`.

## Command Smoothing

The node smooths yaw and pitch commands using an exponential moving average.
Pitch uses:

```text
pitch_smooth = alpha * pitch_target + (1 - alpha) * pitch_old
```

Yaw uses the shortest angular distance so that a target crossing from `+pi` to
`-pi` does not command a full revolution:

```text
yaw_smooth = normalize(yaw_old + alpha * shortest(yaw_old, yaw_target))
```

`cmd_smooth_alpha = 1` means no smoothing. Smaller values smooth more but
increase lag. The fire gate blocks shooting when smoothing lag is too large.

## Fire Gate Logic

The fire gate is intentionally conservative. It allows fire only when all
required checks pass:

1. Tracker is in `TRACKING`.
2. Aim target is valid.
3. Ballistic solution is valid.
4. Distance is within `min_fire_dist` and `max_fire_dist`.
5. Optional stale-measurement and anti-gyro checks pass.
6. Planner margin is non-negative.
7. Camera-frame aim angle is inside `angular_window`.
8. Smoothed command is close enough to the unsmoothed target.

If a margin-like check fails briefly after a shot was allowed, hysteresis can
keep fire enabled across small angular drift. Hysteresis is dropped as soon as
a hard check fails.

## Debugging With The Math

When the robot misses or jitters, debug in the same order as the math pipeline:

1. Detection: Are bounding boxes stable in pixels?
2. PnP: Is reprojection error low and depth stable?
3. Transform: Is odom position stable when the robot is still?
4. EKF: Are innovation and Mahalanobis distance reasonable?
5. Aim: Is the selected face stable and the predicted target plausible?
6. Command: Is smoothing lag large?
7. Fire gate: Which blocker reason dominates?

Do not start by changing EKF noise if PnP depth is already noisy. Do not tune
ballistics if the measured bullet speed is unknown. Fix the earliest wrong
stage.

## Practice Problems

1. The YOLOv26-pose detector reports keypoints TL `(460,260)`, TR `(540,260)`,
   BR `(542,340)`, BL `(458,340)`. List them in the order expected by
   `solveKeypoints`.
2. A point has camera coordinates `(0.2, -0.1, 3.0)` m. With `fx = fy = 700`,
   `cx = 480`, `cy = 300`, compute the projected pixel.
3. If `bullet_speed` is accidentally set too high, will long-range shots tend
   to hit high or low? Explain using the gravity equation.
4. A target has `shortest(bearing, face_yaw) = 0.06 rad` and the scaled window
   is `0.05 rad`. What is the margin? Can the planner allow fire?
5. Why is Mahalanobis distance better than plain distance for target
   association?
