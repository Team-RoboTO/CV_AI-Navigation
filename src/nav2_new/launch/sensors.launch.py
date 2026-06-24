
"""
sensors.launch.py – Livox Mid-360 head-referenced pipeline.

V21 principle:
- Keep the original behaviour that worked well while moving: FAST-LIO pose is primary for odom->base_link.
- Use micro vx/vy only as a stationary gate: if chassis is not translating, freeze x/y exactly but keep yaw from FAST-LIO.
- This avoids fake translation during head rotation without integrating vx/vy in the wrong frame.

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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
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
            "input_topic": "/tf_fastlio",
            "source_parent_frame": "camera_init",
            "source_child_frame": "body",
            "target_parent_frame": "odom",
            "target_child_frame": "base_link",
            "body_to_base_qx": iqx,
            "body_to_base_qy": iqy,
            "body_to_base_qz": iqz,
            "body_to_base_qw": iqw,
            "planarize": True,
            "force_z_zero": True,
            "invert_yaw": False,
            "invert_x": False,
            "invert_y": False,
            # --- lever-arm (calibrare al passo 2d) --- p0 (-0.005, -0.017) p1()
            "lever_arm_enable": True,
            "lever_arm_x": -0.069,
            "lever_arm_y": -0.0195,
            "lever_arm_z": 0.0,
            "use_micro_stationary_gate": LaunchConfiguration("use_micro_stationary_gate"),
            "micro_status_topic": "/micro_status",
            "micro_vx_index": 2,
            "micro_vy_index": 3,
            "micro_vx_sign": 1.0,
            "micro_vy_sign": LaunchConfiguration("micro_vy_sign"),
            "stationary_vxy_threshold": LaunchConfiguration("stationary_vxy_threshold"),
            "micro_timeout_sec": LaunchConfiguration("micro_timeout_sec"),
            "log_stationary_gate": LaunchConfiguration("log_stationary_gate"),
        }],
        output="screen",
    )
    amcl_scan_stabilizer = Node(
        package="nav2_new", executable="amcl_scan_stabilizer",
        name="amcl_scan_stabilizer", output="screen",
        parameters=[{
            "scan_in": "/scan",
            "scan_out": "/scan_amcl",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "stable_frame": "base_stable",
        }],
    )
    amcl_motion_gate = Node(
        package="nav2_new", executable="amcl_motion_gate",
        name="amcl_motion_gate", output="screen",
        parameters=[{
            "micro_status_topic": "/micro_status",
            "amcl_node_name": "amcl",
            "vx_index": 2, "vy_index": 3,
            "vx_sign": 1.0, "vy_sign": -1.0,
            "move_threshold_mps": 0.12,
            "freeze_after_sec": 0.5,
        }],
    )

    return [lidar_tf, tf_relay, amcl_scan_stabilizer, amcl_motion_gate]


def generate_launch_description():
    pkg_share = get_package_share_directory("nav2_new")

    mount_arg = DeclareLaunchArgument("mount", default_value="normal")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="false")

    use_micro_stationary_gate_arg = DeclareLaunchArgument("use_micro_stationary_gate", default_value="true")
    stationary_vxy_threshold_arg = DeclareLaunchArgument("stationary_vxy_threshold", default_value="0.12")
    micro_timeout_sec_arg = DeclareLaunchArgument("micro_timeout_sec", default_value="0.30")
    micro_vy_sign_arg = DeclareLaunchArgument("micro_vy_sign", default_value="-1.0")
    log_stationary_gate_arg = DeclareLaunchArgument("log_stationary_gate", default_value="true")
    publish_fastlio_debug_tf_arg = DeclareLaunchArgument(
        "publish_fastlio_debug_tf",
        default_value="false",
        description="Publish odom->camera_init so raw FAST-LIO body appears in RViz. Keep false in match to avoid ghost TF."
    )

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
    scan_publish_rate_arg = DeclareLaunchArgument("scan_publish_rate_hz", default_value="8.0")
    scan_keep_every_n_arg = DeclareLaunchArgument("scan_keep_every_n", default_value="1")
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
        condition=IfCondition(LaunchConfiguration("publish_fastlio_debug_tf")),
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

    fast_lio = GroupAction([
        # Keep raw FAST-LIO camera_init->body off the public /tf tree.
        # tf_frame_relay consumes it privately on /tf_fastlio and publishes the guarded odom->base_link.
        SetRemap(src="/tf", dst="/tf_fastlio"),
        IncludeLaunchDescription(
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
    ])

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
        use_micro_stationary_gate_arg,
        stationary_vxy_threshold_arg,
        micro_timeout_sec_arg,
        micro_vy_sign_arg,
        log_stationary_gate_arg,
        publish_fastlio_debug_tf_arg,

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