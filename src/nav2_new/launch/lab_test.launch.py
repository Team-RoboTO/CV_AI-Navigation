"""
lab_test.launch.py – Sensors + Nav2 on a pre-built map + RViz for interactive testing.

In RViz:
  1. Click "2D Pose Estimate" → click the robot's actual position on the map
  2. Click "2D Goal Pose"     → click where you want the robot to go

Usage:
  ros2 launch nav2_new lab_test.launch.py map:=/path/to/lab_map.yaml
  ros2 launch nav2_new lab_test.launch.py map:=/path/to/lab_map.yaml mount:=yaw180
  ros2 launch nav2_new lab_test.launch.py map:=/path/to/lab_map.yaml lidar_z:=0.8
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_new')

    map_arg = DeclareLaunchArgument('map', description='Full path to the map yaml')
    mount_arg = DeclareLaunchArgument('mount', default_value='normal')
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

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation.launch.py')),
        launch_arguments={'map': LaunchConfiguration('map')}.items()
    )
    nav_delayed = TimerAction(period=5.0, actions=[navigation])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'navigation.rviz')],
        output='screen'
    )

    return LaunchDescription([
        map_arg, mount_arg, lidar_z_arg,
        sensors, nav_delayed, rviz,
    ])
