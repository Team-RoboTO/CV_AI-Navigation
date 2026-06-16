"""
slam.launch.py – Build a 2D map with SLAM Toolbox in head-referenced mode.

The LiDAR+IMU on the head are treated as the navigation reference.\nRotate the head if needed: in this package that is the robot reference frame for SLAM.\nWhen done, save the map:
  ros2 run nav2_new save_map --ros-args -p name:=lab_map

Usage:
  ros2 launch nav2_new slam.launch.py
  ros2 launch nav2_new slam.launch.py mount:=yaw180
  ros2 launch nav2_new slam.launch.py rviz:=false
  ros2 launch nav2_new slam.launch.py lidar_z:=0.8
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_new')
    default_slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    mount_arg = DeclareLaunchArgument('mount', default_value='normal')
    params_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=default_slam_params)
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false')
    lidar_z_arg = DeclareLaunchArgument('lidar_z', default_value='0.6')

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sensors.launch.py')),
        launch_arguments={
            'mount': LaunchConfiguration('mount'),
            'rviz': 'false',
            'lidar_z': LaunchConfiguration('lidar_z'),
        }.items()
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[LaunchConfiguration('slam_params_file'),
                    {'use_sim_time': False}],
        output='screen'
    )
    slam_delayed = TimerAction(period=5.0, actions=[slam_toolbox_node])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'slam.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        mount_arg, params_arg, rviz_arg, lidar_z_arg,
        sensors, slam_delayed, rviz,
    ])
