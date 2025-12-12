# Armor Tracker

## Overview
The `armor_tracker` package implements a sophisticated Extended Kalman Filter (EKF) ensuring robust tracking of RoboMaster enemy armors. Unlike simple coordinate tracking, this system models the target as a rigid body (a "Spinning Top"), allowing it to maintain lock even when armors rotate out of view.

## Core Logic: The "Spinning Top" Model
Standard trackers fail when an armor rotates behind the robot. This tracker solves this by modeling the **entire robot structure**, not just the visible plate.

### State Vector
The EKF maintains a 9-dimensional state vector:
$$ X = [x_c, v_{xc}, y_c, v_{yc}, z_a, v_{za}, \psi, v_\psi, r]^T $$
Where:
- $(x_c, y_c)$: Position of the robot's **center** (not the armor).
- $(v_{xc}, v_{yc})$: Velocity of the center.
- $z_a$: Height of the armor (assumed constant relative to center).
- $\psi$ (Yaw): The rotation angle of the robot.
- $v_\psi$ (Yaw rate): How fast the robot is spinning.
- $r$: Radius (Distance from center to armor).

### Workflow
1.  **Input**: Receives 3D positions of detected armors from the vision system.
2.  **Predict**: Uses the kinematic model to predict where the armors will be (accounting for rotation).
3.  **Update**:
    -   If an armor is matched, update the state using the EKF.
    -   If the armor matches a known ID but the yaw has jumped (e.g., switched from front armor to side armor), the system detects an **Armor Jump** and adjusts the yaw phase automatically.

## Usage
### Subscribed Topics
-   `/detections_output` (`auto_aim_interfaces/Armors`): The 3D position of armors from the detector.
-   `/camera_info`: Camera intrinsic parameters.

### Published Topics
-   `/tracker/target` (`auto_aim_interfaces/Target`): High-level tracking data (position, velocity, radius, yaw) sent to the trajectory solver.

### Parameters
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `max_match_distance` | `0.15` | Max distance (m) to match a detection to prediction. |
| `max_match_yaw_diff` | `1.0` | Max yaw difference (rad) for matching. |
| `ekf.sigma2_q_xyz` | `20.0` | Process noise for position. |
| `ekf.sigma2_q_yaw` | `100.0` | Process noise for yaw (spin). |
| `ekf.sigma2_q_r` | `800.0` | Process noise for radius. |
