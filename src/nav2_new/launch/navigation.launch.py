"""
navigation.launch.py – Nav2 stack with AMCL localization on a pre-built map.

IMPORTANT — topic naming:
  Nav2's default output topic is `/cmd_vel`. In this project we remap it
  to `/cmd_vel_NAV` to coexist with `/cmd_vel_AI` (from the CV container).
  A downstream combiner/micro-driver reads both and decides which to forward
  to the motors.

  The remapping is done at the composition container level below so ALL
  Nav2 internal nodes publish to /cmd_vel_NAV. If you want the raw default
  behavior, set cmd_vel_topic:=/cmd_vel.

Usage:
  ros2 launch nav2_new navigation.launch.py map:=/path/to/map.yaml
  ros2 launch nav2_new navigation.launch.py map:=... cmd_vel_topic:=/cmd_vel
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_new')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_share, 'maps', 'arena_map.yaml')

    map_arg = DeclareLaunchArgument('map', default_value=default_map)
    params_arg = DeclareLaunchArgument('params_file', default_value=default_params)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    use_composition_arg = DeclareLaunchArgument('use_composition', default_value='True')
    use_respawn_arg = DeclareLaunchArgument('use_respawn', default_value='False')
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info')

    cmd_vel_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel_NAV',
        description='Final motor-facing output topic after XY-only filtering')
    nav2_raw_cmd_arg = DeclareLaunchArgument(
        'nav2_raw_cmd_vel_topic', default_value='/cmd_vel_nav_raw',
        description='Velocity-smoother output before XY-only/rotation filtering')
    controller_cmd_arg = DeclareLaunchArgument(
        'controller_cmd_vel_topic', default_value='/cmd_vel_nav',
        description='Raw controller_server output before velocity_smoother')
    cmd_rotate_arg = DeclareLaunchArgument(
        'cmd_vel_rotate_yaw_deg', default_value='0.0',
        description='Rotate Nav2 x/y command into robot/barrel frame before /cmd_vel_NAV')

    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    nav2_raw_cmd_vel_topic = LaunchConfiguration('nav2_raw_cmd_vel_topic')
    controller_cmd_vel_topic = LaunchConfiguration('controller_cmd_vel_topic')
    cmd_vel_rotate_yaw_deg = LaunchConfiguration('cmd_vel_rotate_yaw_deg')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml,
    }
    configured_params = ParameterFile(
        RewrittenYaml(source_file=params_file, root_key='',
                      param_rewrites=param_substitutions, convert_types=True),
        allow_substs=True)

    # Nav2 still produces its usual cmd_vel, but we first route it to a raw topic
    # and then a small filter node strips angular.z so only x/y translation reaches
    # the motor-facing cmd_vel topic.
    nav2_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', controller_cmd_vel_topic),
        ('/cmd_vel_smoothed', nav2_raw_cmd_vel_topic),
    ]

    nav2_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params, {'autostart': autostart}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=nav2_remappings,
        output='screen',
    )


    xy_only_filter = Node(
        package='nav2_new',
        executable='cmd_vel_xy_only',
        name='cmd_vel_xy_only',
        parameters=[{
            'input_topic': controller_cmd_vel_topic,
            'output_topic': cmd_vel_topic,
            'rotate_yaw_deg': cmd_vel_rotate_yaw_deg,
        }],
        output='screen',
)

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': LaunchConfiguration('use_composition'),
            'use_respawn': LaunchConfiguration('use_respawn'),
            'container_name': 'nav2_container',
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': LaunchConfiguration('use_composition'),
            'use_respawn': LaunchConfiguration('use_respawn'),
            'container_name': 'nav2_container',
        }.items()
    )

    return LaunchDescription([
        map_arg, params_arg, use_sim_time_arg, autostart_arg,
        use_composition_arg, use_respawn_arg, log_level_arg, cmd_vel_arg, nav2_raw_cmd_arg, controller_cmd_arg, cmd_rotate_arg,
        nav2_container, xy_only_filter, localization, navigation,
    ])
