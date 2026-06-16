#sudo systemctl stop autoaim.service


import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Default TensorRT engine path. The model is kept outside the package
# (e.g. on the Isaac ROS dev volume) so it is NOT rebuilt/installed with
# colcon. Override per-launch with engine_path:=... or the env var.
ENGINE_DEFAULT = "/workspaces/isaac_ros-dev/AI-models/yolov26_keypoints.engine"


def autoaim_params():
    return [
        # YOLO26 labels: "0" blue, "1" grey, "2" red. Grey is ignored by autoaim_node.
        {"target_classes": ["0"]},
        {"use_keypoints": True},
        {"keypoint_topic": "/detector/armors_keypoints"},
        # 0.0 accepted garbage keypoints -> PnP jitter -> fire-lock dropouts.
        {"min_keypoint_score": 0.15},
        {"max_reproj_error": 25.0},
        # Use the gimbal angles AT THE IMAGE TIMESTAMP for the camera->odom
        # projection (ring buffer + interpolation) instead of the latest angles.
        # Fixes the "tracker is slow / lags behind" feel whenever the head moves.
        # zed_detector.py now stamps with sl.TIME_REFERENCE.IMAGE (capture time).
        {"angle_sync_enable": True},
        {"light_ratio": 0.85},
        {"max_armor_distance": 6.0},
        {"max_armor_z": 4.0},
        {"confirm_frames": 2},
        {"lost_timeout": 0.50},
        # 30/40 were very high — probably raised to fight the lag caused by the
        # missing image<->angle sync (now fixed). High q makes the state track
        # measurement noise -> jittery commands -> fire-lock dropouts.
        # Start lower; raise again only if tracking feels sluggish AFTER the fix.
        {"q_pos": 100.0},
        {"q_yaw": 20.0},
        {"q_r": 1e-6},
        {"r_pos_base": 0.05},
        {"r_pos_slope": 0.04},
        {"r_yaw_base": 0.05},
        {"r_yaw_slope": 0.005},
        {"max_oblique_deg": 65.0},
        # 0.98 at ~100 Hz decays the velocity estimate to ~13%/s — a constant
        # drag that under-leads translating targets. Let q_pos handle the noise.
        {"alpha_pos": 0.995},
        {"alpha_yaw": 1.00},
        {"alpha_coast": 0.98},
        {"initial_radius": 0.24},
        {"radius_ema_alpha": 0.05},
        {"initial_dz": 0.05},
        {"bullet_speed": 20.0},
        {"gravity": 9.8},
        {"gimbal_height": 0.420},
        # MEASURE FROM THE ACTIVE LENS, NOT THE HOUSING CENTER — and note this
        # is a ZED X MINI: stereo baseline 50 mm, so the lens is only ~2.5 cm
        # from the housing center (NOT ~6 cm — that figure is for ZED 2/X).
        # The camera is upside down with SDK FLIP_MODE.ON (see zed_detector.py),
        # which also SWAPS which physical sensor produces the "left image".
        # Find the active lens empirically: cover one lens with a finger -> the
        # one that blacks out the detector feed is the active one. Then:
        #   barrel_offset_y = (muzzle) - (active lens), along robot-LEFT [m]
        #                     (positive = muzzle left of lens)
        #   barrel_offset_z = (muzzle height) - (lens height) [m]
        #                     camera 3 cm ABOVE barrel -> -0.03 
        {"barrel_offset_x": 0.0},
        {"barrel_offset_y": 0.08},     # <- MEASURE (lens-cover trick, see CHANGES.md), !!! WAS 0.15 FOR STANDARD SENTRY
        {"barrel_offset_z": -0.15},
        # 1.0 rad @ ref 1 m -> ~0.33 rad window at 3 m: OK for tuning, loose for
        # a match. Against a fast spinner, timed shots need ~0.10-0.18 rad with
        # window_ref_dist ~3.0. Tighten once the static calibration is done.
        {"angular_window": 1.0},
        {"window_ref_dist": 3.0},
        {"min_fire_dist": 0.2},
        {"max_fire_dist": 6.0},
        # 0.005 is essentially ZERO latency compensation. It must cover the FULL
        # capture->muzzle latency (capture+inference+transport+serial+gimbal),
        # typically 40-80 ms for this pipeline. Now that the detector stamps with
        # capture time you can MEASURE it: log (now - header.stamp) at command
        # publish and add ~15 ms. Too small -> shots trail a mover; too large -> lead.
        # Measured-latency horizon: the node measures (now − capture stamp)
        # per frame and adds actuation_latency = serial TX + gimbal settle +
        # muzzle exit (calibrate: shoot a mover, adjust in 5 ms steps; the node
        # logs the measured pipeline part every 2 s).
        {"use_measured_latency": True},
        {"actuation_latency": 0.020},
        # Fallback fixed bias, used ONLY if use_measured_latency is False.
        {"time_bias": 0.045},
        # Must equal the REAL detection rate: ros2 topic hz /detector/armors_keypoints
        # (the ZED grabs at 120 fps — if inference keeps up this should be ~120).
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
        # ⚠ THESE TWO MUST BE EQUAL. Both describe the same physical fact —
        # whether the micro's pitch FEEDBACK has the opposite sign of its pitch
        # COMMAND. pitch_sign appears SQUARED in the loop, so it cannot absorb
        # the difference (the old comment claiming independence was wrong).
        # With True/False, exactly one of two failures is guaranteed:
        #   - geometry pitch mirrored -> systematic vertical miss (shoots low/high), or
        #   - pitch lock error = 2x command -> fire only when commanded pitch ~ 0
        #     ("shoots only sometimes").
        # Determine the truth empirically: echo /micro_status — field [1] is the
        # pitch feedback, field [11] is the pitch command echo. With the gimbal
        # settled on a target: feedback ≈ -command -> set BOTH True;
        # feedback ≈ +command -> set BOTH False.
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


def generate_launch_description():
    # TensorRT engine lives OUTSIDE the package (not installed into the
    # package share). Override order: --launch arg engine_path  >  env
    # AUTOAIM_ENGINE_PATH  >  this default.
    default_engine = os.environ.get("AUTOAIM_ENGINE_PATH", ENGINE_DEFAULT)

    engine_path = LaunchConfiguration("engine_path")
    serial_port = LaunchConfiguration("serial_port")

    serial_params = [
        {"serial_port": serial_port},
        {"serial_baudrate": 500000},
        {"serial_tx_hz": 100.0},
        {"serial_reconnect_interval": 2.0},
        {"serial_rx_timeout": 3.0},
        # SAFETY: if /cmd_vel_AI goes stale for longer than this (autoaim node
        # crashed), the shoot flag sent to the micro is forced to 0. Without it
        # the bridge re-sends the last shoot=1 at 100 Hz forever.
        {"cmd_timeout": 0.3},
        # Header+CRC8 framing (self-resyncing, rejects corrupted packets).
        # REQUIRES matching micro firmware — see CHANGES.md. Leave False until then.
        {"use_framed_protocol": False},
    ]

    viewer_params = [
        # Must match micro_pitch_lock_opposite_sign in the autoaim params above,
        # otherwise the pitch-error numbers in the debug overlay are sign-flipped.
        {"micro_pitch_feedback_opposite_sign": True},
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
            # C++ serial bridge (port of serial_bridge.py — same params/protocol).
            # To fall back to the Python version: executable="serial_bridge.py".
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
            parameters=detector_params,
            output="screen",
        ),
    ])
