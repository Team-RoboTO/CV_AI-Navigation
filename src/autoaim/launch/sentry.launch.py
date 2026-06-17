# sudo systemctl stop autoaim.service

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# Default TensorRT engine path. The model is kept outside the package
# (e.g. on the Isaac ROS dev volume) so it is NOT rebuilt/installed with
# colcon. Override per-launch with engine_path:=... or the env var.
DEFAULT_ENGINE = "/workspaces/isaac_ros-dev/AI-models/yolov26_keypoints.engine"

# Change this to "zed" or "realsense" when this robot's default camera changes.
DEFAULT_CAMERA = "zed"


def autoaim_params():
    return [
        # YOLO26 labels: 0=blue armor, 1=grey armor (ignored), 2=red armor.
        # True: read our team color from /micro_status[target_color_status_index]
        #       and pick the enemy class automatically via micro_color_target_classes.
        # False: always shoot the fixed class list in target_classes below.
        {"target_classes_from_micro_status": True},

        # micro_color_target_classes[i] = YOLO class to shoot when micro sends i.
        #   i=0 (we are red)  -> shoot blue -> "0"
        #   i=1 (we are blue) -> shoot red  -> "2"
        {"target_classes": ["0"]},
        {"target_color_status_index": 4},
        {"micro_color_target_classes": ["0", "2"]},

        {"use_keypoints": True},
        {"keypoint_topic": "/detector/armors_keypoints"},

        {"min_keypoint_score": 0.15},
        {"max_reproj_error": 25.0},

        {"angle_sync_enable": True},
        {"light_ratio": 0.85},
        {"max_armor_distance": 6.0},
        {"max_armor_z": 4.0},
        {"confirm_frames": 2},
        {"lost_timeout": 0.50},

        {"q_pos": 100.0},
        {"q_yaw": 20.0},
        {"q_r": 1e-6},
        {"r_pos_base": 0.05},
        {"r_pos_slope": 0.04},
        {"r_yaw_base": 0.05},
        {"r_yaw_slope": 0.005},
        {"max_oblique_deg": 65.0},

        {"alpha_pos": 0.995},
        {"alpha_yaw": 1.00},
        {"alpha_coast": 0.98},

        {"initial_radius": 0.24},
        {"radius_ema_alpha": 0.05},
        {"initial_dz": 0.05},

        {"bullet_speed": 20.0},
        {"gravity": 9.8},
        {"gimbal_height": 0.420},

        {"barrel_offset_x": 0.0},
        {"barrel_offset_y": 0.08},
        {"barrel_offset_z": -0.15},

        {"angular_window": 1.0},
        {"window_ref_dist": 3.0},
        {"min_fire_dist": 0.2},
        {"max_fire_dist": 6.0},

        {"use_measured_latency": True},
        {"actuation_latency": 0.020},
        {"time_bias": 0.045},

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
        {"fire_lock_yaw": 0.5},
        {"fire_lock_pitch": 0.5},

        {"micro_pitch_feedback_opposite_sign": True},
        {"micro_pitch_lock_opposite_sign": True},

        {"cmd_hold_time": 0.25},
        {"cmd_max_delta_yaw": 1.0},
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


def turret_mux_params():
    return [
        # CV final command, already produced by autoaim.
        {"cv_cmd_topic": "/cmd_vel_AI"},

        # Real detection gate. Do NOT use /cmd_vel_AI freshness because
        # autoaim keeps publishing the last yaw/pitch after target loss.
        {"detection_topic": "/detector/armors"},
        {"detection_timeout": 0.30},
        {"cv_cmd_timeout": 0.50},

        {"micro_status_topic": "/micro_status"},
        {"idle_target_topic": "/turret/idle_target"},
        {"turret_cmd_topic": "/turret/cmd"},

        # The CV container must see /tf and /tf_static from navigation.
        # The mux uses TF map -> base_link to aim at the idle waypoint.
        {"map_frame": "map"},
        {"base_frame": "base_link"},

        {"enable_cv_pipeline": True},
        {"enable_idle_pipeline": True},

        {"publish_rate": 50.0},

        # If idle yaw goes in the wrong direction, set -1.0.
        {"yaw_sign": 1.0},

        # Offset between micro yaw zero and barrel/LiDAR forward direction.
        {"yaw_zero_offset": 0.0},

        # Smooth idle/nav yaw. 0.7 rad/s ≈ 40 deg/s.
        {"idle_yaw_rate_limit": 0.7},

        # When target is lost, hold the pitch reported by the micro.
        {"idle_pitch_mode": "hold_last_micro"},
        {"idle_pitch_static": 0.0},

        # In /micro_status: RX[0] = yaw, RX[1] = pitch.
        {"micro_yaw_index": 0},
        {"micro_pitch_index": 1},

        # Normally idle target comes from navigation.
        {"use_default_idle_target": False},
        {"default_idle_target_x": 0.0},
        {"default_idle_target_y": 0.0},
    ]


def generate_launch_description():
    default_engine = os.environ.get("AUTOAIM_ENGINE_PATH", DEFAULT_ENGINE)

    engine_path = LaunchConfiguration("engine_path")
    serial_port = LaunchConfiguration("serial_port")

    serial_params = [
        {"serial_port": serial_port},
        {"serial_baudrate": 500000},
        {"serial_tx_hz": 100.0},
        {"serial_reconnect_interval": 2.0},
        {"serial_rx_timeout": 3.0},

        # If /turret/cmd becomes stale, shoot is forced to 0.
        # Yaw/pitch are still resent to hold.
        {"cmd_timeout": 0.3},

        # Header+CRC8 framing. Requires matching micro firmware.
        {"use_framed_protocol": False},

        # New architecture:
        # turret_yaw_mux publishes /turret/cmd,
        # navigation publishes /cmd_vel_NAV.
        {"turret_cmd_topic": "/turret/cmd"},
        {"nav_cmd_topic": "/cmd_vel_NAV"},
        {"micro_status_topic": "/micro_status"},

        # Runtime switches.
        {"enable_nav_pipeline": True},
        {"enable_turret_pipeline": True},
        {"hold_last_turret_when_disabled": True},
    ]

    viewer_params = [
        {"micro_pitch_feedback_opposite_sign": True},
    ]

    camera = LaunchConfiguration("camera")
    zed_condition = IfCondition(PythonExpression(["'", camera, "' == 'zed'"]))
    realsense_condition = IfCondition(PythonExpression(["'", camera, "' == 'realsense'"]))

    pkg_share = get_package_share_directory("autoaim")
    zed_config = os.path.join(pkg_share, "config", "zed.yaml")
    realsense_config = os.path.join(pkg_share, "config", "realsense.yaml")

    engine_override = {"engine_path": engine_path}

    return LaunchDescription([
        DeclareLaunchArgument("engine_path", default_value=str(default_engine)),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),

        DeclareLaunchArgument(
            "camera", default_value=DEFAULT_CAMERA,
            description='Active camera detector: "realsense" or "zed".'),

        # Mux must start before serial_bridge.
        Node(
            package="autoaim",
            executable="turret_yaw_mux",
            name="turret_yaw_mux",
            parameters=turret_mux_params(),
            output="screen",
        ),

        Node(
            package="autoaim",
            executable="serial_bridge",
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
            parameters=[zed_config, engine_override],
            output="screen",
            condition=zed_condition,
        ),

        Node(
            package="autoaim_realsense",
            executable="realsense_detector",
            name="realsense_detector",
            parameters=[realsense_config, engine_override],
            output="screen",
            condition=realsense_condition,
        ),
    ])
