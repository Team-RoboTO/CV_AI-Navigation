
"""
sensors.launch.py – Livox Mid-360 head-referenced pipeline.

Optimized mode:
- FAST-LIO uses raw /livox/lidar
- Nav2/AMCL receive /scan directly from /livox/lidar through livox_to_scan_node
- Heavy chain /livox/lidar -> PointCloud2 -> pointcloud_to_laserscan is disabled
- Livox filter is disabled unless use_livox_filter:=true

Important:
- The Livox flip is assumed to be handled inside the driver.
- Keep mount:=normal unless you are explicitly testing a TF-only mount change.
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
    "flip": (1.0, 0.0, 0.0, 0.0),
    "yaw180": (0.0, 0.0, 1.0, 0.0),
    "normal": (0.0, 0.0, 0.0, 1.0),
}


def launch_setup(context, *args, **kwargs):
    mount = LaunchConfiguration("mount").perform(context)
    if mount not in MOUNT_QUATS:
        raise RuntimeError(f"Invalid mount='{mount}'. Use one of {list(MOUNT_QUATS)}")

    qx, qy, qz, qw = MOUNT_QUATS[mount]
    iqx, iqy, iqz, iqw = -qx, -qy, -qz, qw

    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_livox_frame",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", str(qx), "--qy", str(qy), "--qz", str(qz), "--qw", str(qw),
            "--frame-id", "base_link",
            "--child-frame-id", "livox_frame",
        ],
        output="screen",
    )

    tf_relay = Node(
        package="nav2_new",
        executable="tf_frame_relay",
        name="tf_frame_relay",
        parameters=[{
            "body_to_base_qx": iqx,
            "body_to_base_qy": iqy,
            "body_to_base_qz": iqz,
            "body_to_base_qw": iqw,
            "planarize": True,
            "force_z_zero": True,
            "invert_yaw": False,
            "invert_x": False,
            "invert_y": False,
        }],
        output="screen",
    )

    return [lidar_tf, tf_relay]


def generate_launch_description():
    pkg_share = get_package_share_directory("nav2_new")

    mount_arg = DeclareLaunchArgument("mount", default_value="normal")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="false")

    use_livox_filter_arg = DeclareLaunchArgument(
        "use_livox_filter",
        default_value="false",
        description="Enable Livox CustomMsg filtering before FAST-LIO"
    )

    filter_min_range_arg = DeclareLaunchArgument("filter_min_range", default_value="0.10")
    filter_max_range_arg = DeclareLaunchArgument("filter_max_range", default_value="4.0")
    filter_min_z_arg = DeclareLaunchArgument("filter_min_z", default_value="-3.00")
    filter_max_z_arg = DeclareLaunchArgument("filter_max_z", default_value="3.00")
    filter_drop_tags_arg = DeclareLaunchArgument("filter_drop_nonzero_tags", default_value="false")

    lidar_x_arg = DeclareLaunchArgument("lidar_x", default_value="0.0")
    lidar_y_arg = DeclareLaunchArgument("lidar_y", default_value="0.0")
    lidar_z_arg = DeclareLaunchArgument("lidar_z", default_value="0.65")

    # Direct LaserScan parameters.
    # These replace:
    #   /livox/lidar -> livox_custom_to_pc2 -> /livox/lidar_pc2
    #   /livox/lidar_pc2 -> pointcloud_to_laserscan -> /scan
    scan_publish_rate_arg = DeclareLaunchArgument("scan_publish_rate_hz", default_value="6.0")
    scan_keep_every_n_arg = DeclareLaunchArgument("scan_keep_every_n", default_value="2")
    scan_range_min_arg = DeclareLaunchArgument("scan_range_min", default_value="0.20")
    scan_range_max_arg = DeclareLaunchArgument("scan_range_max", default_value="4.0")
    scan_z_min_arg = DeclareLaunchArgument("scan_z_min", default_value="-0.55")
    scan_z_max_arg = DeclareLaunchArgument("scan_z_max", default_value="-0.15")
    scan_angle_increment_arg = DeclareLaunchArgument("scan_angle_increment", default_value="0.015")

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("livox_ros_driver2"),
                "launch_ROS2",
                "msg_MID360_launch.py",
            ])
        ])
    )

    livox_filter = Node(
        package="nav2_new",
        executable="livox_custom_filter",
        name="livox_custom_filter",
        condition=IfCondition(LaunchConfiguration("use_livox_filter")),
        parameters=[{
            "input_topic": "/livox/lidar",
            "output_topic": "/livox/lidar_filtered",
            "min_range": LaunchConfiguration("filter_min_range"),
            "max_range": LaunchConfiguration("filter_max_range"),
            "min_z": LaunchConfiguration("filter_min_z"),
            "max_z": LaunchConfiguration("filter_max_z"),
            "drop_nonzero_tags": LaunchConfiguration("filter_drop_tags"),
            "log_every_n_clouds": 30,
        }],
        output="screen",
    )

    odom_to_camera_init_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_camera_init",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "odom",
            "--child-frame-id", "camera_init",
        ],
        output="screen",
    )

    base_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_footprint",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "base_link",
            "--child-frame-id", "base_footprint",
        ],
        output="screen",
    )

    # Kept for compatibility. The optimized /scan normally keeps the Livox frame
    # unless the livox_to_scan_node parameter frame_id is set.
    scan_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_scan",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "base_link",
            "--child-frame-id", "base_scan",
        ],
        output="screen",
    )

    robot_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_robot_base",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "-0.7071068", "--qw", "0.7071068",
            "--frame-id", "base_link",
            "--child-frame-id", "robot_base",
        ],
        output="screen",
    )

    # Optimized direct conversion:
    #   /livox/lidar -> /scan
    #
    # frame_id="" means: preserve the original Livox frame from the message.
    # This is safer than pretending the scan is already in base_link.
    #
    # z_min/z_max are chosen to match your previous pointcloud_to_laserscan slice:
    #   min_height: -0.55
    #   max_height: -0.15
    livox_to_scan = Node(
        package="livox_converter",
        executable="livox_to_scan_node",
        name="livox_to_scan",
        parameters=[{
            "input_topic": "/livox/lidar",
            "output_topic": "/scan",

            # Empty string = keep original message frame.
            # If you later want to force a frame, use "livox_frame" or "base_link".
            "frame_id": "",

            # CPU controls.
            "publish_rate_hz": LaunchConfiguration("scan_publish_rate_hz"),
            "keep_every_n": LaunchConfiguration("scan_keep_every_n"),
            "qos_depth": 1,

            # LaserScan geometry.
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": LaunchConfiguration("scan_angle_increment"),
            "scan_time": 0.10,

            # Range and height filters.
            "range_min": LaunchConfiguration("scan_range_min"),
            "range_max": LaunchConfiguration("scan_range_max"),
            "z_min": LaunchConfiguration("scan_z_min"),
            "z_max": LaunchConfiguration("scan_z_max"),

            # Optional diagnostics inside optimized node.
            "log_every_n_clouds": 100,
        }],
        output="screen",
    )

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("fast_lio"),
                "launch",
                "mapping.launch.py",
            ])
        ]),
        launch_arguments={
            "config_path": os.path.join(pkg_share, "config"),
            "config_file": "mid360.yaml",
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    scan_memory_filter = Node(
        package="nav2_new",
        executable="scan_memory_filter",
        name="scan_memory_filter",
        parameters=[{
            "input_topic": "/scan",
            "output_topic": "/scan_nav",

            # Low persistence: less ghosting during long loop navigation.
            "persistence_sec": 0.05,

            # Keep local obstacle scan close and stable.
            "min_range": 0.30,
            "max_range": LaunchConfiguration("scan_range_max"),

            # Keep gate disabled. Gate via /turret/cmd previously caused local
            # costmap to lose obstacles when CV/turret was active.
            "gate_enabled": False,
            "gate_topic": "/turret/cmd",
            "cv_mode_threshold": 0.5,
            "gate_hold_sec": 0.40,
            "gate_on_turret_motion": True,
            "turret_motion_yaw_delta": 0.03,
            "publish_clear_scan_when_gated": False,
        }],
        output="screen",
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

        scan_publish_rate_arg,
        scan_keep_every_n_arg,
        scan_range_min_arg,
        scan_range_max_arg,
        scan_z_min_arg,
        scan_z_max_arg,
        scan_angle_increment_arg,

        OpaqueFunction(function=launch_setup),

        livox_driver,

        # Optional raw Livox filter, disabled by default.
        livox_filter,

        odom_to_camera_init_tf,
        scan_tf,
        base_footprint_tf,
        robot_base_tf,

        # Replaces pointcloud_converter + pointcloud_to_laserscan.
        livox_to_scan,

        fast_lio,
        scan_memory_filter,
    ])