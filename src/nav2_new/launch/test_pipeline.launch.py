"""
test_pipeline.launch.py – End-to-end integration test WITHOUT real hardware.

Launches:
  • Full navigation pipeline (sensors + Nav2 + waypoint_manager + mux + safety)
  • fake_micro_status — simulates STM32 telemetry on /micro_status
  • fake_cv_publisher — simulates CV target on /cv/target
  • micro_status_parser + game_state_manager (real game FSM)

Useful for:
  • Verifying strategy transitions on HP change without the real micro
  • Testing turret mux handoff (sweep vs. chassis-forward)
  • Developing / debugging new strategies offline

Usage:
  ros2 launch nav2_new test_pipeline.launch.py map:=/path/to/lab_map.yaml

Then from other terminals:
  ros2 topic pub --once /fake_micro/start_match std_msgs/Bool "data: true"
  ros2 topic pub --once /fake_micro/set_hp std_msgs/Int32 "data: 20"
  ros2 topic pub --once /fake_micro/center_captured std_msgs/Bool "data: true"
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

    args = [
        DeclareLaunchArgument('map', description='Full path to map yaml'),
        DeclareLaunchArgument('team', default_value='red'),
        DeclareLaunchArgument('mount', default_value='flip'),
        DeclareLaunchArgument('cv_pattern', default_value='sweep'),
    ]

    competition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'competition.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'team': LaunchConfiguration('team'),
            'mount': LaunchConfiguration('mount'),
        }.items()
    )

    fake_micro = Node(
        package='nav2_new',
        executable='fake_micro_status',
        name='fake_micro_status',
        parameters=[{'team': LaunchConfiguration('team')}],
        output='screen',
    )

    fake_cv = Node(
        package='nav2_new',
        executable='fake_cv_publisher',
        name='fake_cv_publisher',
        parameters=[{'pattern': LaunchConfiguration('cv_pattern')}],
        output='screen',
    )

    # Start fakes a bit after nav so it doesn't miss the init handshake
    fakes_delayed = TimerAction(period=12.0, actions=[fake_micro, fake_cv])

    return LaunchDescription(args + [competition, fakes_delayed])
