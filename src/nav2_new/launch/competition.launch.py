"""
competition.launch.py – Full RoboMaster competition pipeline.

Pipeline:
  sensors → Nav2 (combat params) → set_initial_pose
    ↓
  micro_status_parser (JSON /micro_status → individual topics)
    ↓
  game_state_manager (FSM: waiting → rushing → holding / retreating)
    ↓  /strategy
  waypoint_manager (executes the strategy via Nav2 Simple Commander)
    ↓  /cmd_vel_nav
  cmd_vel_safety  →  /cmd_vel  →  motor driver
  turret_yaw_mux  →  /turret/cmd

Usage:
  # With real micro publishing /micro_status
  ros2 launch nav2_new competition.launch.py map:=/root/roboto_maps/arena.yaml

  # Override the default team (used until /team arrives from micro_status)
  ros2 launch nav2_new competition.launch.py team:=blue

  # Disable combat-mode tight costmap and use lab params
  ros2 launch nav2_new competition.launch.py \\
      params_file:=/path/to/nav2_params.yaml
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
    default_map = os.path.join(pkg_share, 'maps', 'arena_map.yaml')
    default_waypoints = os.path.join(pkg_share, 'config', 'arena_waypoints.yaml')
    default_turret = os.path.join(pkg_share, 'config', 'turret_mux.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params_combat.yaml')

    args = [
        DeclareLaunchArgument('team', default_value='red',
                              description='Default team (overridden by /team topic)'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('mount', default_value='flip'),
        DeclareLaunchArgument('lidar_z', default_value='0.8',
                              description='LiDAR height above floor in meters'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 params — use nav2_params_combat.yaml in matches'),
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints),
        DeclareLaunchArgument('turret_config', default_value=default_turret),
        DeclareLaunchArgument('enable_turret_mux', default_value='false',
                              description='Default: false. The CV container owns turret yaw/pitch; Nav does not contribute. Set true only if you want Nav to publish /turret/cmd during idle.'),
        DeclareLaunchArgument('enable_cmd_vel_safety', default_value='false',
                              description='Optional safety watchdog. Your serial bridge already runs at 100 Hz so this is usually not needed.'),
        DeclareLaunchArgument('enable_game_state_fsm', default_value='true',
                              description='If false, waypoint_manager idle-waits for manual /strategy commands'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    # ── 1) Sensors ────────────────────────────────────────────────────────
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sensors.launch.py')),
        launch_arguments={
            'mount': LaunchConfiguration('mount'),
            'rviz': 'false',
            'lidar_z': LaunchConfiguration('lidar_z'),
        }.items()
    )

    # ── 2) Nav2 stack ────────────────────────────────────────────────────
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )
    nav_delayed = TimerAction(period=5.0, actions=[navigation])

    # ── 3) Seed AMCL at spawn ─────────────────────────────────────────────
    set_pose = TimerAction(
        period=10.0,
        actions=[Node(
            package='nav2_new',
            executable='set_initial_pose',
            name='set_initial_pose',
            parameters=[{
                'team': LaunchConfiguration('team'),
                'waypoints_file': LaunchConfiguration('waypoints_file'),
            }],
            output='screen')]
    )

    # ── 4) Waypoint manager ──────────────────────────────────────────────
    waypoint_mgr = TimerAction(
        period=15.0,
        actions=[Node(
            package='nav2_new',
            executable='waypoint_manager',
            name='waypoint_manager',
            parameters=[{
                'team': LaunchConfiguration('team'),
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'initial_strategy': 'wait_at_spawn',
            }],
            output='screen')]
    )

    # ── 5) Micro status parser ──────────────────────────────────────────
    # ── 5) Micro status adapter ────────────────────────────────────────
    #     Your serial bridge publishes /micro_status as Float32MultiArray.
    #     The adapter unpacks indices → /team, /match_started, /health,
    #     /center_captured which game_state_manager subscribes to.
    micro_adapter = Node(
        package='nav2_new',
        executable='micro_status_adapter',
        name='micro_status_adapter',
        output='screen',
    )

    # ── 6) Game state manager ──────────────────────────────────────────
    game_fsm = Node(
        package='nav2_new',
        executable='game_state_manager',
        name='game_state_manager',
        parameters=[{'waypoints_file': LaunchConfiguration('waypoints_file')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_game_state_fsm')),
    )

    # ── 7) Turret mux ──────────────────────────────────────────────────
    turret_mux = Node(
        package='nav2_new',
        executable='turret_yaw_mux',
        name='turret_yaw_mux',
        parameters=[LaunchConfiguration('turret_config')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_turret_mux')),
    )

    # ── 8) cmd_vel safety ──────────────────────────────────────────────
    cmd_vel_safety = Node(
        package='nav2_new',
        executable='cmd_vel_safety',
        name='cmd_vel_safety',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_cmd_vel_safety')),
    )

    # ── 9) RViz ────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'navigation.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [
        sensors, nav_delayed, set_pose, waypoint_mgr,
        micro_adapter, game_fsm,
        turret_mux, cmd_vel_safety, rviz,
    ])
