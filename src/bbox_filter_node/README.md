# bbox_filter_node
package name: filter
node name: talker

## Shell command to run nodes from *launch file*
```sh
cd <workspace>
source install/setup.bash
ros2 launch filter __launch.py
```

## Shell commands to run talker on linux

Shell code to run talker:
```sh
conda activate <my_env>
cd <workspace>
source /opt/ros/humble/setup.bash 
source install/setup.bash
ros2 run filter talker
```

Shell code to play the bag (to be run in a different terminal):
```sh
conda activate <my_env>
cd <workspace>
source /opt/ros/humble/setup.bash 
source install/setup.bash
ros2 bag info ros2 bag info rosbag2_2024_02_29-16_19_44/rosbag2_2024_02_29-16_19_44.db3 
ros2 bag play ros2 bag info rosbag2_2024_02_29-16_19_44 
```


Creating .yaml file from rosbag:
```sh
ros2 bag reindex ros_bag (folder)
```
Running TF to enable the movement of Path object:
```sh
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 1.5707963267948966 world P
```

## Bash code and service
The service is located in `/lib/systemd/system/docker_service.service`

Bash script is located in `/home/roboto/workspaces/isaac_ros-dev/src/bbox_filter_node/start_docker_and_nodes.sh`


# shoot_prediction_without_drag Node Documentation

## Description

The shoot_prediction_without_drag node implements a system to predict and calculate the projectile trajectory based on data provided by the IMU and configuration parameters.

## Parameters
The node uses three parameters configurable via launch file (already defined by default) that correspond to the offsets along the axes between the camera and the reference point:
- camera_x_displacement, default: 0.17 m 
- camera_y_displacement, default: 0.00 m 
- camera_z_displacement, default: 0.065 m 

## Topics
### Input
1. /detections_output/optimal_target   
Type: vision_msgs/msg/Detection2D
2. /imu/filtered  
Type: sensor_msgs/msg/Imu
3. /micro_pose  
Type: geometry_msgs/msg/PoseStamped
### Output
1. /predicted_shoot  
Type: nav_msgs/msg/Path
2. /cmd_vel  
Type: geometry_msgs/msg/Twist

## Functioning
### Initialization
The node declares the camera offset parameters, creates the publishers and subscribers, and instantiates an object of the PredictionWithout class to calculate the projectile trajectory.

### Sensor Callbacks
- imu_callback  
This method reads the data coming from the IMU and updates the system angular velocities to readjust the system orientation and rotation.

- micro_callback  
This method receives position data from the /micro_pose topic. The theta_0 parameter is updated with the X coordinate of the current position.

### Detection Processing
detections_callback :
- Process the target position relative to the camera.
- Calculate the trajectory and angular references using the PredictionWithout class.
- Publish the results to the /cmd_vel topic and update the predicted path to the /predicted_shoot topic.

### Trajectory Publishing
The trajectory points are stored in a circular queue with up to 30 points.

# prediction_without library documentation

## Description

The PredictionWithout class implements a prediction method to determine the launch angles of a projectile to intercept a moving target. It uses numerical optimization to minimize the distance between the projectile and the target at the moment of impact.

## Initialization
```sh
PredictionWithout(v_p, lag=0.02, refs=5)
```

### Parameters
- v_p (float): projectile velocity.
- lag (float, default 0.02): system latency time.
- refs (int, default 5): number of references for calculating averaged angles.

### Attributes
- g (float): gravitational acceleration (9.81 m/s²).
- x_prev, y_prev, z_prev (float or None): last known coordinates of the target.
- ts_prev (float or None): previous timestamp.
- vax, vay, vaz (float): instantaneous velocities in three dimensions.
- weig_avg (numpy array): matrix for weighted averaging of velocities.
- weig_avg_t, weig_avg_p (numpy array): vectors for weighted averaging of angles.

## Methods
```sh
target_linear_position(x0, y0, z0, vx, vy, vz, t)
```
Calculates the future position of the target assuming linear motion.

### Parameters
- x0, y0, z0 (float): initial position of the target.
- vx, vy, vz (float): target velocity.
- t (float): prediction time.

### Returns
numpy.array: estimated position of the target.

```sh
projectile_position_without(theta, psi, t, theta_0=None)
```
Calculates the position of the projectile as a function of launch angles and time.

### Parameters
- theta, psi (float): launch angles in spherical coordinates.
- t (float): elapsed time since launch.
- theta_0 (float, optional): rotation angle to apply.

### Returns
numpy.array: projectile position.

```sh
objective_function_without(x0, y0, z0, vx, vy, vz, lag_time, params)
```
Objective function to minimize: distance between the projectile and the target at the moment of impact.

### Parameters
- x0, y0, z0 (float): initial position of the target.
- vx, vy, vz (float): target velocity.
- lag_time (float): system latency time.
- params (tuple): values of theta, psi, and t_hit.

### Returns
float: distance between the projectile and the target.

```sh
prediction_without(x0, y0, z0, wz, ts)
```
Estimates the optimal angles for the shot and the impact time.

### Parameters
- x0, y0, z0 (float): current position of the target.
- wz (float): angular velocity of the target.
- ts (float): current timestamp.


### Returns
tuple (float, float, float, OptimizeResult): angles theta, psi, impact time t_hit and optimization result.
