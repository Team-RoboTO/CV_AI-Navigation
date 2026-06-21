"""
competition_match.launch.py

Full navigation-side match pipeline.

Start order:
  1. sensors.launch.py immediately
  2. micro_status_adapter and game_status_reporter early
  3. navigation.launch.py after 15 seconds, so Livox/IMU/FAST-LIO are warm
  4. initial pose after Nav2/AMCL are active enough to receive /initialpose
  5. waypoint_manager + game_state_manager + turret_idle_target_publisher

CV container must run:
  - turret_yaw_mux
  - serial_bridge
and must see /tf, /tf_static, /cmd_vel_NAV, /turret/idle_target.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_new')

    map_arg = DeclareLaunchArgument('map', default_value='/root/nav2_ws/maps/arena_final.yaml')
    team_arg = DeclareLaunchArgument('team', default_value='red')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    use_composition_arg = DeclareLaunchArgument('use_composition', default_value='False')
    mount_arg = DeclareLaunchArgument('mount', default_value='normal')
    lidar_z_arg = DeclareLaunchArgument('lidar_z', default_value='0.6')
    start_sensors_arg = DeclareLaunchArgument('start_sensors', default_value='true')
    start_rviz_arg = DeclareLaunchArgument('rviz', default_value='false')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
    )
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join(pkg_share, 'config', 'arena_waypoints.yaml'),
    )
    match_params_file_arg = DeclareLaunchArgument(
        'match_params_file',
        default_value=os.path.join(pkg_share, 'config', 'match_manager_params.yaml'),
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sensors.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_sensors')),
        launch_arguments={
            'mount': LaunchConfiguration('mount'),
            'lidar_z': LaunchConfiguration('lidar_z'),
            'rviz': 'false',
        }.items(),
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'use_composition': LaunchConfiguration('use_composition'),
        }.items(),
    )

    common_params = [
        LaunchConfiguration('match_params_file'),
        {
            'team': LaunchConfiguration('team'),
            'default_team': LaunchConfiguration('team'),
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
    ]

    micro_status_adapter = Node(
        package='nav2_new', executable='micro_status_adapter', name='micro_status_adapter',
        output='screen', parameters=common_params,
    )

    game_status_reporter = Node(
        package='nav2_new', executable='game_status_reporter', name='game_status_reporter',
        output='screen', parameters=common_params,
    )

    nav_match_reset = Node(
        package='nav2_new', executable='nav_match_reset', name='nav_match_reset',
        output='screen', parameters=common_params,
    )

    set_initial_pose = Node(
        package='nav2_new', executable='set_initial_pose', name='set_initial_pose',
        output='screen', parameters=common_params,
    )

    turret_idle_target = Node(
        package='nav2_new', executable='turret_idle_target_publisher', name='turret_idle_target_publisher',
        output='screen', parameters=common_params,
    )

    waypoint_manager = Node(
        package='nav2_new', executable='waypoint_manager',
        output='screen', parameters=common_params,
    )

    game_state_manager = Node(
        package='nav2_new', executable='game_state_manager', name='game_state_manager',
        output='screen', parameters=common_params,
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        map_arg, team_arg, use_sim_time_arg, use_composition_arg,
        mount_arg, lidar_z_arg, start_sensors_arg, start_rviz_arg,
        params_file_arg, waypoints_file_arg, match_params_file_arg,

        # 0s: sensors first.
        sensors,

        # 2s: status adapter can already listen to /micro_status; game status can monitor startup.
        TimerAction(period=3.0, actions=[micro_status_adapter]),

        # 15s: start Nav2 only after Livox/IMU/FAST-LIO have had time to stabilize.
        TimerAction(period=15.0, actions=[nav]),

        # 21s: reset helper after Nav2 services exist.
        TimerAction(period=21.0, actions=[nav_match_reset]),

        # 24s: seed AMCL initial pose after map_server/amcl are alive.
        TimerAction(period=26.0, actions=[set_initial_pose]),

        # 28s: start match logic and waypoint execution.
        TimerAction(period=30.0, actions=[turret_idle_target, waypoint_manager, game_state_manager]),

        rviz,
    ])
