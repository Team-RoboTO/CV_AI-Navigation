"""
sensors.launch.py – Livox Mid-360 head-referenced pipeline.

Default mode:
- FAST-LIO uses raw /livox/lidar
- Livox filter is disabled unless use_livox_filter:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


MOUNT_QUATS = {
    'flip': (1.0, 0.0, 0.0, 0.0),
    'yaw180': (0.0, 0.0, 1.0, 0.0),
    'normal': (0.0, 0.0, 0.0, 1.0),
}


def launch_setup(context, *args, **kwargs):
    mount = LaunchConfiguration('mount').perform(context)
    if mount not in MOUNT_QUATS:
        raise RuntimeError(f"Invalid mount='{mount}'. Use one of {list(MOUNT_QUATS)}")

    qx, qy, qz, qw = MOUNT_QUATS[mount]
    iqx, iqy, iqz, iqw = -qx, -qy, -qz, qw

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame',
        ],
        output='screen',
    )

    tf_relay = Node(
        package='nav2_new',
        executable='tf_frame_relay',
        name='tf_frame_relay',
        parameters=[{
            'body_to_base_qx': iqx,
            'body_to_base_qy': iqy,
            'body_to_base_qz': iqz,
            'body_to_base_qw': iqw,
            'planarize': True,
            'force_z_zero': True,
            'invert_yaw': False,
            'invert_x': False,
            'invert_y': False,
        }],
        output='screen',
    )

    return [lidar_tf, tf_relay]


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_new')

    mount_arg = DeclareLaunchArgument('mount', default_value='normal')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false')

    use_livox_filter_arg = DeclareLaunchArgument(
        'use_livox_filter',
        default_value='false',
        description='Enable Livox CustomMsg filtering before FAST-LIO'
    )

    filter_min_range_arg = DeclareLaunchArgument('filter_min_range', default_value='0.10')
    filter_max_range_arg = DeclareLaunchArgument('filter_max_range', default_value='4.0')
    filter_min_z_arg = DeclareLaunchArgument('filter_min_z', default_value='-3.00')
    filter_max_z_arg = DeclareLaunchArgument('filter_max_z', default_value='3.00')
    filter_drop_tags_arg = DeclareLaunchArgument('filter_drop_nonzero_tags', default_value='false')

    lidar_x_arg = DeclareLaunchArgument('lidar_x', default_value='0.0')
    lidar_y_arg = DeclareLaunchArgument('lidar_y', default_value='0.0')
    lidar_z_arg = DeclareLaunchArgument('lidar_z', default_value='0.65')

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py',
            ])
        ])
    )

    pointcloud_converter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_converter'),
                'launch',
                'launch.py',
            ])
        ])
    )

    livox_filter = Node(
        package='nav2_new',
        executable='livox_custom_filter',
        name='livox_custom_filter',
        condition=IfCondition(LaunchConfiguration('use_livox_filter')),
        parameters=[{
            'input_topic': '/livox/lidar',
            'output_topic': '/livox/lidar_filtered',
            'min_range': LaunchConfiguration('filter_min_range'),
            'max_range': LaunchConfiguration('filter_max_range'),
            'min_z': LaunchConfiguration('filter_min_z'),
            'max_z': LaunchConfiguration('filter_max_z'),
            'drop_nonzero_tags': LaunchConfiguration('filter_drop_nonzero_tags'),
            'log_every_n_clouds': 30,
        }],
        output='screen',
    )

    odom_to_camera_init_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_camera_init',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'odom',
            '--child-frame-id', 'camera_init',
        ],
        output='screen',
    )

    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_footprint',
        ],
        output='screen',
    )

    scan_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_scan',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_scan',
        ],
        output='screen',
    )

    robot_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_robot_base',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '-0.7071068', '--qw', '0.7071068',
            '--frame-id', 'base_link',
            '--child-frame-id', 'robot_base',
        ],
        output='screen',
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.5,
            'min_height': -0.55,
            'max_height': -0.15,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.015,
            'scan_time': 0.1,
            'range_min': 0.20,
            'range_max': 4.0,
            'inf_epsilon': 1.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/livox/lidar_pc2'),
            ('scan', '/scan'),
        ],
        output='screen',
    )

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'launch',
                'mapping.launch.py',
            ])
        ]),
        launch_arguments={
            'config_path': os.path.join(pkg_share, 'config'),
            'config_file': 'mid360.yaml',
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )
    scan_memory_filter = Node(
        package='nav2_new',
        executable='scan_memory_filter',
        name='scan_memory_filter',
        parameters=[{
            'input_topic': '/scan',
            'output_topic': '/scan_nav',
            'persistence_sec': 0.15,
            'min_range': 0.20,
            'max_range': 4.0,
            # Gate the scan while the turret/head is moving or CV is active.
            # This prevents a head-mounted Livox from drawing fake obstacles in Nav2.
            'gate_enabled': True,
            'gate_topic': '/turret/cmd',
            'cv_mode_threshold': 0.5,
            'gate_hold_sec': 0.40,
            'gate_on_turret_motion': True,
            'turret_motion_yaw_delta': 0.03,
            'publish_clear_scan_when_gated': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        mount_arg,
        rviz_arg,
        use_livox_filter_arg,
        filter_min_range_arg,
        filter_max_range_arg,
        filter_min_z_arg,
        filter_max_z_arg,
        filter_drop_tags_arg,
        lidar_x_arg,
        lidar_y_arg,
        lidar_z_arg,
        OpaqueFunction(function=launch_setup),
        livox_driver,
        pointcloud_converter,
        livox_filter,
        odom_to_camera_init_tf,
        scan_tf,
        base_footprint_tf,
        robot_base_tf,
        pointcloud_to_laserscan,
        fast_lio,
        scan_memory_filter,
    ])