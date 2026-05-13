import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("auto_aim")
    launch_pkg_share = get_package_share_directory("launch_pkg")
    default_params = os.path.join(pkg_share, "config", "params_realsense_16.yaml")
    default_engine = os.path.join(
        launch_pkg_share, "resources", "yolov26_keypoints.engine"
    )

    engine_path = LaunchConfiguration("engine_path")
    params_file = LaunchConfiguration("params_file")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    use_fake_micro_imu = LaunchConfiguration("use_fake_micro_imu")
    fake_imu_yaw_rad = LaunchConfiguration("fake_imu_yaw_rad")
    fake_imu_pitch_rad = LaunchConfiguration("fake_imu_pitch_rad")
    fake_imu_rate_hz = LaunchConfiguration("fake_imu_rate_hz")
    angular_window = LaunchConfiguration("angular_window")
    fire_alignment_source = LaunchConfiguration("fire_alignment_source")
    pnp_force_armor_size = LaunchConfiguration("pnp_force_armor_size")
    pnp_size_switch_margin_px = LaunchConfiguration("pnp_size_switch_margin_px")
    max_armor_z = LaunchConfiguration("max_armor_z")
    detector_threshold = LaunchConfiguration("detector_threshold")
    publish_debug_every = LaunchConfiguration("publish_debug_every")
    detector_debug_scores = LaunchConfiguration("detector_debug_scores")

    realsense_node = ComposableNode(
        name="camera",
        namespace="camera",
        package="realsense2_camera",
        plugin="realsense2_camera::RealSenseNodeFactory",
        parameters=[{
            "enable_color": True,
            "enable_depth": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": True,
            "enable_accel": True,
            "unite_imu_method": 2,
            "publish_tf": False,
            "rgb_camera.profile": "640x480x60",
            "pointcloud.enable": False,
            "align_depth.enable": False,
        }],
    )

    realsense_container = ComposableNodeContainer(
        name="realsense_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[realsense_node],
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    yolo26_detector = Node(
        package="auto_aim",
        executable="yolo26_pose_realsense_node.py",
        name="yolo26_pose_realsense",
        parameters=[{
            "engine_path": engine_path,
            "image_topic": image_topic,
            "frame_id": "camera_color_optical_frame",
            "threshold": ParameterValue(detector_threshold, value_type=float),
            "nms_iou": 0.20,
            "keypoint_score_threshold": 0.05,
            "publish_debug_every": ParameterValue(publish_debug_every, value_type=int),
            "debug_scores": ParameterValue(detector_debug_scores, value_type=bool),
            "clip_keypoints_to_image": False,
        }],
        output="screen",
        emulate_tty=True,
    )

    auto_aim_node = Node(
        package="auto_aim",
        executable="auto_aim_node",
        name="auto_aim",
        parameters=[
            params_file,
            {
                # Keypoint topic + thresholds come from params_file; this dict
                # only overrides the per-launch pieces.
                "keypoint_topic": "/detector/armors_keypoints",
                # Do not override target_classes or tracker state-machine
                # parameters here. The aggressive old debug values made the
                # tracker jump between false/nearer detections and hid real
                # association failures. Use the YAML as the source of truth.
                "angular_window": ParameterValue(angular_window, value_type=float),
                "fire.alignment_source": fire_alignment_source,
                "pnp.force_armor_size": pnp_force_armor_size,
                "pnp.size_switch_margin_px": ParameterValue(
                    pnp_size_switch_margin_px, value_type=float
                ),
                "max_armor_z": ParameterValue(max_armor_z, value_type=float),
            },
        ],
        remappings=[
            ("/camera_info", camera_info_topic),
        ],
        output="screen",
        emulate_tty=True,
    )

    fake_micro_imu = Node(
        package="auto_aim",
        executable="fake_micro_imu_node.py",
        name="fake_micro_imu",
        parameters=[{
            # Static test pose only. It never derives orientation from video
            # frames; yaw/pitch are fixed launch arguments.
            "mode": "static",
            "yaw_rad": ParameterValue(fake_imu_yaw_rad, value_type=float),
            "pitch_rad": ParameterValue(fake_imu_pitch_rad, value_type=float),
            "rate_hz": ParameterValue(fake_imu_rate_hz, value_type=float),
        }],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_fake_micro_imu),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "engine_path",
            default_value=default_engine,
            description=(
                "TensorRT engine installed from "
                "launch_pkg/resources/yolov26_keypoints.engine."
            ),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Auto-aim parameter YAML for the Jetson 16 RealSense robot.",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/camera/color/image_raw",
            description="RealSense color image topic consumed by the YOLOv26 detector.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera/color/camera_info",
            description="RealSense CameraInfo topic remapped into auto_aim /camera_info.",
        ),
        DeclareLaunchArgument(
            "use_fake_micro_imu",
            default_value="true",
            description=(
                "Run static yaw=0 pitch=0 /micro_imu for bench/video tests. "
                "Set false on the robot with real microcontroller feedback."
            ),
        ),
        DeclareLaunchArgument(
            "fake_imu_rate_hz",
            default_value="120.0",
            description="Static fake IMU publish rate.",
        ),
        DeclareLaunchArgument(
            "fake_imu_yaw_rad",
            default_value="0.0",
            description="Static fake IMU yaw [rad] for bench transform tests.",
        ),
        DeclareLaunchArgument(
            "fake_imu_pitch_rad",
            default_value="0.0",
            description="Static fake IMU pitch [rad] for mount-pitch diagnosis.",
        ),
        DeclareLaunchArgument(
            "angular_window",
            default_value="0.09",
            description=(
                "Fire/off-axis angular window [rad]. For unwarped video "
                "debug only, try 0.25-0.40."
            ),
        ),
        DeclareLaunchArgument(
            "fire_alignment_source",
            default_value="disabled",
            description=(
                "Fire alignment gate source: camera_angle, relative_error, or "
                "disabled. Debug launch defaults disabled for static fake IMU."
            ),
        ),
        DeclareLaunchArgument(
            "pnp_force_armor_size",
            default_value="auto",
            description="PnP armor size override: auto, small, or large.",
        ),
        DeclareLaunchArgument(
            "pnp_size_switch_margin_px",
            default_value="0.50",
            description=(
                "When PnP size auto margin is below this, keep previous size."
            ),
        ),
        DeclareLaunchArgument(
            "max_armor_z",
            default_value="1.2",
            description=(
                "Max absolute armor z [m] after fake/real IMU transform. "
                "Raise only to diagnose mount-pitch errors."
            ),
        ),
        DeclareLaunchArgument(
            "detector_threshold",
            default_value="0.20",
            description=(
                "YOLOv26 confidence threshold. For bench diagnosis, try 0.10 "
                "if max_conf is below 0.20."
            ),
        ),
        DeclareLaunchArgument(
            "publish_debug_every",
            default_value="0",
            description=(
                "Publish /yolo/debug_image every N detector frames. "
                "0 disables the image copy."
            ),
        ),
        DeclareLaunchArgument(
            "detector_debug_scores",
            default_value="false",
            description="Log detector score summaries every 60 frames.",
        ),
        realsense_container,
        fake_micro_imu,
        TimerAction(period=1.0, actions=[yolo26_detector]),
        TimerAction(period=2.0, actions=[auto_aim_node]),
    ])
