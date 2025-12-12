# Trajectory Solver

## Overview
The `rm_trajectory` package is responsible for ballistics solving. It calculates the necessary gimbal pitch and yaw angles to hit a moving target, accounting for gravity, air resistance, and system latency.

## Core Logic: Iterative Solver
The solver uses a numerical iteration approach to find the firing solution.

### Physics Model
The flight of the projectile is modeled with:
-   **Gravity**: $g = 9.8 m/s^2$
-   **Air Resistance**: Modeled as linear drag $a = -k \cdot v$

### Latency Compensation
To hit moving targets, the system predicts the target's future position based on the total delay:
$$ t_{prediction} = t_{flight} + t_{bias} $$
Where:
-   $t_{flight}$: Estimated time for the bullet to reach the target.
-   $t_{bias}$: System latency (Image capture $\to$ Processing $\to$ CAN Bus command).

### Algorithm
1.  **Initial Guess**: $t = dist / v_{bullet}$
2.  **Iterate** (max 10 times):
    a.  Predict target position at $t + t_{bias}$.
    b.  Calculate required Pitch ($\theta$) using parabolic approximation with drag compensation.
    c.  Recalculate path length through the arc.
    d.  Update flight time $t$.
    e.  If converged, break.

## Usage
### Subscribed Topics
-   `/tracker/target` (`auto_aim_interfaces/Target`): Target state from the EKF tracker.

### Published Topics
-   `/tracker/cmd_gimbal` (`auto_aim_interfaces/GimbalCmd`): Pitch/Yaw commands for the turret.
-   `/tracker/cmd_vel` (`geometry_msgs/Twist`): Angular velocities (for debugging).
-   `/trajectory/marker` (`visualization_msgs/Marker`): Impact point visualization.

### Parameters
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `bullet_speed` | `25.0` | Muzzle velocity (m/s). |
| `gravity` | `9.8` | Gravity constant (adjust if needed, usually 9.8). |
| `k` | `0.01` | Air resistance coefficient. |
| `time_bias` | `0.05` | Latency compensation in seconds (50ms). |
