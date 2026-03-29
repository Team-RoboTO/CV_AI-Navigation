# Trajectory Solver

## Overview
The `rm_trajectory` package is responsible for ballistics solving. It calculates the necessary gimbal pitch and yaw angles to hit a moving target, accounting for gravity, air resistance, and system latency. It supports multi-target tracking, indirect firing (aiming ahead of spinning targets), and hysteresis-based fire gating.

## Core Logic: Iterative Solver
The solver uses a numerical iteration approach to find the firing solution.

### Physics Model
The flight of the projectile is modeled with:
-   **Gravity**: $g = 9.8 m/s^2$
-   **Air Resistance (Two Modes)**: 
    -   *Linear Drag* (Strada A): Models drag proportional to velocity ($a = -k_l \cdot v$). Good for slow speeds.
    -   *Quadratic Drag* (Strada B): Models drag proportional to velocity squared ($a = -k_q \cdot v^2$). Required for realistic drop over distance at high muzzle velocities.

### Targeting Modes
-   **Direct Aiming**: Tracks and shoots at the currently visible armor plate.
-   **Indirect Aiming**: When the enemy robot spins rapidly, the system aims at a fixed position ahead in time, predicting exactly when the armor plate will rotate into the crosshairs. Includes *Hysteresis* to prevent jittering when switching modes.

### Target Validation
For a target to be fired upon, it must pass a strict gate:
-   **Ballistic Valid**: The mathematical equations successfully find a real-world firing solution without hitting limits.
-   **Reachable**: The calculated pitch angle does not exceed the mechanical limits of the robot's physical turret.

### Latency Compensation
To hit moving targets, the system predicts the target's future position based on total delay using a `transport_delay` compensation:
$$ t_{prediction} = t_{flight} + t_{bias} + t_{transport\_delay} $$

### Algorithm
1.  **Initial Guess**: $t = dist / v_{bullet}$
2.  **Iterate** (max 12 times):
    a.  Predict target position at $t_{prediction}$.
    b.  Calculate required Pitch ($	heta$) using parabolic approximation with selected aerodynamic drag compensation.
    c.  Update flight time $t$ by evaluating the true horizontal distance.
    d.  If converged (change in time is microscopic), break.

## Usage
### Subscribed Topics
-   `/tracker/targets` (`auto_aim_interfaces/Targets`): Array of multiple target states from the EKF tracker.
-   `/micro_pose` / `/imu/data`: Odometry topics to compute the current physical pitch and yaw limits.

### Published Topics
-   `/tracker/cmd_gimbal` (`auto_aim_interfaces/GimbalCmd`): Pitch/Yaw absolute commands.
-   `/tracker/cmd_vel` (`geometry_msgs/Twist`): Fallback compatibility commands.
-   `/trajectory/marker` (`visualization_msgs/Marker`): Impact point visualization for RViz.

### Key Parameters
| Parameter | Description |
| :--- | :--- |
| `bullet_speed` | Muzzle velocity (m/s). |
| `use_quadratic_drag` | Boolean toggle between Linear and Quadratic drag models. |
| `linear_drag_coeff` | Air resistance coefficient for linear mode. |
| `quadratic_drag_coeff` | Air resistance coefficient for quadratic mode. |
| `time_bias` | Additional fixed hardware latency compensation (ms). |
| `indirect_vyaw_threshold` | Spin speed (rad/s) needed to trigger indirect shooting mode. |
