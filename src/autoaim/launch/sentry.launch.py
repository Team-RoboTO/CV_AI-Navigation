from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MODEL_FILE = "yolov26_keypoints.engine"


def find_models_dir(pkg_share: Path) -> Path:
    for path in (pkg_share, *pkg_share.parents):
        candidate = path / "models"
        if candidate.is_dir():
            return candidate
    return pkg_share / "models"


def autoaim_params():
    return [
        # YOLO26 labels: "0" blue, "1" grey, "2" red. Grey is ignored by autoaim_node.
        {"target_classes": ["0"]},
        {"use_keypoints": True},
        {"keypoint_topic": "/detector/armors_keypoints"},
        {"min_keypoint_score": 0.0},
        {"max_reproj_error": 25.0},
        {"light_ratio": 0.85},
        {"max_armor_distance": 6.0},
        {"max_armor_z": 4.0},
        {"confirm_frames": 2},
        {"lost_timeout": 0.50},
        {"q_pos": 30.0},
        {"q_yaw": 40.0},
        {"q_r": 1e-6},
        {"r_pos_base": 0.05},
        {"r_pos_slope": 0.04},
        {"r_yaw_base": 0.05},
        {"r_yaw_slope": 0.005},
        {"max_oblique_deg": 65.0},
        {"alpha_pos": 0.98},
        {"alpha_yaw": 1.00},
        {"alpha_coast": 0.98},
        {"initial_radius": 0.24},
        {"radius_ema_alpha": 0.05},
        {"initial_dz": 0.05},
        {"bullet_speed": 25.0},
        {"gravity": 9.8},
        {"gimbal_height": 0.420},
        {"barrel_offset_x": 0.0},
        {"barrel_offset_y": -0.06},
        {"barrel_offset_z": -0.03},
        {"angular_window": 1.0},
        {"window_ref_dist": 1.0},
        {"min_fire_dist": 0.2},
        {"max_fire_dist": 6.0},
        {"time_bias": 0.005},
        {"ref_freq": 60.0},
        {"yaw_jump_thresh": 0.55},
        {"use_vyaw_from_timing": True},
        {"vyaw_timing_min_dt": 0.050},
        {"vyaw_timing_max_dt": 0.500},
        {"vyaw_fire_threshold": 5.0},
        {"max_match_dist": 0.8},
        {"maha_threshold": 16.9},
        {"switch_range_ratio": 0.85},
        {"switch_cooldown": 10},
        {"same_target_identity_dist": 1.0},
        {"cmd_smooth_alpha": 1.00},
        {"cmd_deadband_yaw": 0.005},
        {"cmd_deadband_pitch": 0.005},
        {"cmd_rate_limit_yaw": 0.0},
        {"cmd_rate_limit_pitch": 0.0},
        {"fire_lock_yaw": 0.05},
        {"fire_lock_pitch": 0.04},
        {"micro_pitch_feedback_opposite_sign": True},
        {"micro_pitch_lock_opposite_sign": False},
        {"cmd_hold_time": 0.25},
        {"cmd_max_delta_yaw": 0.80},
        {"cmd_max_delta_pitch": 0.80},
        {"require_aim_inside_frame": False},
        {"use_ego_motion_compensation": True},
        {"ego_velocity_available": False},
        {"ego_velocity_body_frame": True},
        {"ego_velocity_scale_x": 1.0},
        {"ego_velocity_scale_y": 1.0},
        {"ego_velocity_max": 3.0},
        {"ego_position_max_drift": 0.0},
        {"chassis_heading_index": -1},
        {"gimbal.yaw_sign": 1.0},
        {"gimbal.pitch_sign": -1.0},
    ]


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("autoaim"))
    default_engine = (
        find_models_dir(pkg_share)
        / "jetson64"
        / MODEL_FILE
    )

    engine_path = LaunchConfiguration("engine_path")
    serial_port = LaunchConfiguration("serial_port")

    serial_params = [
        {"serial_port": serial_port},
        {"serial_baudrate": 500000},
        {"serial_tx_hz": 100.0},
        {"serial_reconnect_interval": 2.0},
        {"serial_rx_timeout": 3.0},
    ]

    viewer_params = [
        {"micro_pitch_feedback_opposite_sign": False},
    ]

    detector_params = [
        {"engine_path": engine_path},
        {"threshold": 0.15},
        {"nms_iou": 0.2},
        {"publish_debug_every": 4},
        {"debug_scores": True},
    ]

    return LaunchDescription([
        DeclareLaunchArgument("engine_path", default_value=str(default_engine)),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        Node(
            package="autoaim",
            executable="serial_bridge.py",
            name="micro_communications_node",
            parameters=serial_params,
            output="screen",
        ),
        Node(
            package="autoaim",
            executable="autoaim_node",
            name="autoaim",
            parameters=autoaim_params(),
            output="screen",
        ),
        Node(
            package="autoaim",
            executable="viewer_node.py",
            name="autoaim_viewer",
            parameters=viewer_params,
            output="screen",
        ),
        Node(
            package="autoaim",
            executable="zed_detector.py",
            name="zed_detector",
            parameters=detector_params,
            output="screen",
        ),
    ])
