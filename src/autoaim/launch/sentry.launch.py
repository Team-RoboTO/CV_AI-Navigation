import os
from pathlib import Path

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

# -- Debug / logging policy: safe-by-default competition mode -----------------
# Keep debug publishers and ROS log output off for normal launches. These values
# are launch-file configuration, not command-line launch arguments. Functional
# topics are never gated here: /detector/armors, /detector/armors_keypoints,
# /cmd_vel_AI, serial TX, and fire control stay active.
DEBUG_SETTINGS = {
    "viewer_node_enable": True,           # Starts autoaim_viewer and /tracker/debug_image only when True.
    "detector_publish_debug_every": 0,     # /yolo/debug_image cadence; 0 means no debug-image publisher, 4 is useful on bench.
    "detector_publish_json": True,        # /detector/armors_keypoints_json; debug echo only, tracker never reads it.
    "autoaim_publish_markers": True,      # /tracker/marker RViz MarkerArray; skips MarkerArray construction when False.
    "autoaim_publish_aim_pixels": True,   # /tracker/aim_pixels viewer aid; command math still runs for safety checks.
    "ros_log_sinks_enable": False,         # Master sink switch; False disables stdout, rosout, and external-lib ROS log output, including fatal output.
    "log_detector_enable": False,          # Detector log category; False passes enable_ros_logs=False and uses fatal as a backup ROS level.
    "log_autoaim_enable": False,           # Autoaim/tracker log category; False skips AA_RCLCPP_* wrappers before ROS logging.
    "log_serial_enable": False,            # Serial bridge log category; False skips ROS log calls; TX/RX still runs.
    "log_viewer_enable": False,            # Viewer log category; relevant only when viewer_node_enable is True.
    "log_turret_mux_enable": False,        # Sentry-only turret_yaw_mux log category.
    "log_level_detector": "warn",         # Used only when log_detector_enable=True; restores normal WARN/ERROR/FATAL visibility.
    "log_level_autoaim": "warn",          # Used only when log_autoaim_enable=True; INFO/DEBUG remain hidden unless set lower.
    "log_level_serial": "warn",           # Used only when log_serial_enable=True; matches previous WARN-default behavior.
    "log_level_viewer": "warn",           # Used only when log_viewer_enable=True.
    "log_level_turret_mux": "warn",       # Used only when log_turret_mux_enable=True.
}


def ros_log_args(enable_key, level_key):
    # Runtime ROS logging control. The node parameter enable_ros_logs lets C++
    # AA_RCLCPP_* wrappers and Python guard clauses return before ROS logging;
    # log-level and sink arguments are the backup layer when logs are enabled.
    level = DEBUG_SETTINGS[level_key] if DEBUG_SETTINGS[enable_key] else "fatal"
    args = ["--ros-args", "--log-level", level]
    if not DEBUG_SETTINGS["ros_log_sinks_enable"]:
        args += [
            "--disable-stdout-logs",
            "--disable-rosout-logs",
            "--disable-external-lib-logs",
        ]
    return args


def python_log_params(enable_key):
    return [
        # Python nodes read enable_ros_logs before active get_logger() calls so
        # f-string construction and Python logging calls are skipped when off.
        {"enable_ros_logs": DEBUG_SETTINGS[enable_key]},
    ]

def autoaim_debug_params():
    return [
        # Debug node parameters are sourced from DEBUG_SETTINGS above so the
        # safe-by-default policy is centralized. /cmd_vel_AI is published
        # unconditionally by autoaim_node and is never controlled here.
        {"debug_publish_markers": DEBUG_SETTINGS["autoaim_publish_markers"]},
        {"debug_publish_aim_pixels": DEBUG_SETTINGS["autoaim_publish_aim_pixels"]},
        # C++ autoaim_node uses this to skip AA_RCLCPP_* wrappers entirely when
        # the category is off; command/tracking/fire paths remain unchanged.
        {"enable_ros_logs": DEBUG_SETTINGS["log_autoaim_enable"]},
    ]


def detector_debug_params():
    return [
        # Listed after the sensor YAML in each detector Node, so these launch
        # defaults win over persistent YAML defaults. The bbox/keypoint detector
        # topics are functional outputs and are never controlled here.
        {"publish_debug_every": DEBUG_SETTINGS["detector_publish_debug_every"]},
        {"publish_json": DEBUG_SETTINGS["detector_publish_json"]},
        # Used by both detector implementations: Python skips get_logger() calls,
        # C++ skips AA_RCLCPP_* wrappers before entering RCLCPP macros.
        {"enable_ros_logs": DEBUG_SETTINGS["log_detector_enable"]},
    ]


def autoaim_params():
    return [
        # YOLO26 labels: 0=blue armor, 1=grey armor (ignored), 2=red armor.
        # True: read our team color from /micro_status[target_color_status_index]
        #       and pick the enemy class automatically via micro_color_target_classes.
        # False: always shoot the fixed class list in target_classes below.
        {"target_classes_from_micro_status": True},  # ON/OFF auto color from micro

        # micro_color_target_classes[i] = YOLO class to shoot when micro sends i.
        #   i=0 (we are red)  -> shoot blue -> "0"
        #   i=1 (we are blue) -> shoot red  -> "2"
        {"target_classes": ["0"]},  # used only if target_classes_from_micro_status=False
        {"target_color_status_index": 4},
        {"micro_color_target_classes": ["0", "2"]},
        {"use_keypoints": True},
        {"keypoint_topic": "/detector/armors_keypoints"},

        # 0.0 accepted garbage keypoints -> PnP jitter -> fire-lock dropouts.
        {"min_keypoint_score": 0.3},
        {"max_reproj_error": 25.0},

        # Use the gimbal angles AT THE IMAGE TIMESTAMP for the camera->odom
        # projection (ring buffer + interpolation) instead of the latest angles.
        # Fixes the "tracker is slow / lags behind" feel whenever the head moves.
        # zed_detector.py now stamps with sl.TIME_REFERENCE.IMAGE (capture time).
        {"angle_sync_enable": True},
        {"light_ratio": 0.85},
        {"max_armor_distance": 6.0},
        {"max_armor_z": 4.0},
        {"confirm_frames": 1},
        {"lost_timeout": 0.75},

        # 30/40 were very high — probably raised to fight the lag caused by the
        # missing image<->angle sync (now fixed). High q makes the state track
        # measurement noise -> jittery commands -> fire-lock dropouts.
        # Start lower; raise again only if tracking feels sluggish AFTER the fix.
        {"q_pos": 80.0},
        # q_z DECOUPLED from q_pos: the height channel must NOT inherit the big
        # q_pos used to chase movers, or PnP z-jitter becomes phantom vertical
        # velocity and the pitch oscillates. Keep small.
        {"q_z": 2.0},
        {"q_yaw": 12.0},
        {"q_r": 1e-6},
        {"r_pos_base": 0.05},
        {"r_pos_slope": 0.04},
        {"r_yaw_base": 0.05},
        {"r_yaw_slope": 0.005},
        {"max_oblique_deg": 65.0},

        {"pitch_offset_deg": -2.0},
        {"yaw_offset_deg": 4.5},

        # 0.98 at ~100 Hz decays the velocity estimate to ~13%/s — a constant
        # drag that under-leads translating targets. Let q_pos handle the noise.
        {"alpha_pos": 0.995},
        {"alpha_yaw": 1.00},
        {"alpha_coast": 0.98},

        {"initial_radius": 0.24},
        {"radius_ema_alpha": 0.05},
        {"initial_dz": 0.05},

        {"bullet_speed": 24.0},
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
        #                     Sentry-calibrated shared value: camera above barrel.
        {"barrel_offset_x": 0.0},
        {"barrel_offset_y": -0.03},     # <- MEASURE (lens-cover trick, see INSTRUCTIONS.md)
        {"barrel_offset_z": -0.16},

        # 1.0 rad @ ref 1 m -> ~0.33 rad window at 3 m: OK for tuning, loose for
        # a match. Against a fast spinner, timed shots need ~0.10-0.18 rad with
        # window_ref_dist ~3.0. Tighten once the static calibration is done.
        {"angular_window": 0.65},
        {"window_ref_dist": 3.0},
        {"min_fire_dist": 0.2},
        {"max_fire_dist": 6.0},
        # Obliquity fire gate, range-independent. 0.0 = OFF (current behavior).
        # Set ~0.6 to stop firing on foreshortened plates during slow spin
        # (the main cause of low hit-rate there). vis = cos(angle-off-frontal).
        {"fire_min_vis": 0.6},

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
        {"actuation_latency": 0.060},
        # Fallback fixed bias, used ONLY if use_measured_latency is False.
        {"time_bias": 0.045},

        # Must equal the REAL detection rate: ros2 topic hz /detector/armors_keypoints
        # (the ZED grabs at 120 fps — if inference keeps up this should be ~120).
        {"ref_freq": 60.0},
        {"yaw_jump_thresh": 0.55},
        {"use_vyaw_from_timing": True},
        {"vyaw_timing_min_dt": 0.050},
        {"vyaw_timing_max_dt": 0.9},
        {"vyaw_fire_threshold": 5.0},

        {"max_match_dist": 0.8},
        {"maha_threshold": 16.9},
        {"switch_range_ratio": 0.85},
        {"switch_cooldown": 10},
        {"same_target_identity_dist": 1.0},

        {"cmd_smooth_alpha": 0.80},
        # YAW stays at 1.0 (snappy, no lag for movers). PITCH can be smoothed
        # independently: lower toward ~0.6 ONLY if pitch is still nervous after
        # q_z + rate-limit + deadband. 1.0 = no smoothing (no added lag).
        {"cmd_smooth_alpha_pitch": 0.75},
        {"cmd_deadband_yaw": 0.02},
        {"cmd_deadband_pitch": 0.015},
        {"cmd_rate_limit_yaw": 5.0},
        # Pitch rate limit: ~2.0 rad/s (~114 deg/s). Real plate-height changes
        # are far slower than this, so it removes jitter without adding lag.
        # YAW stays 0.0 (unlimited) so fast movers are tracked snappily.
        {"cmd_rate_limit_pitch": 2.0},
        {"fire_lock_yaw": 0.10},
        {"fire_lock_pitch": 0.07},

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
        {"micro_pitch_lock_opposite_sign": False},

        {"cmd_hold_time": 0.25},
        {"cmd_max_delta_yaw": 0.80},
        {"cmd_max_delta_pitch": 0.0},
        {"require_aim_inside_frame": False},

        {"use_ego_motion_compensation": True},
        {"ego_velocity_available": True},
        {"ego_velocity_body_frame": True},
        {"ego_velocity_scale_x": 1.0},
        {"ego_velocity_scale_y": 1.0},
        {"ego_velocity_max": 3.0},
        {"ego_velocity_deadband": 0.20},
        {"ego_velocity_lpf_alpha": 0.25},
        {"ego_position_max_drift": 0.0},
        {"chassis_heading_index": -1},
        {"gimbal.yaw_sign": 1.0},
        {"gimbal.pitch_sign": -1.0},
    ]

def turret_mux_params():
    return [
        {"cv_cmd_topic": "/cmd_vel_AI"},
        {"detection_topic": "/detector/armors"},
        {"micro_status_topic": "/micro_status"},
        {"output_topic": "/turret/cmd"},

        {"detection_timeout": 0.50},
        {"cv_cmd_timeout": 0.50},
        {"min_detection_score": 0.12},

        # When CV is lost, aim at the navigation-provided map point.
        {"use_idle_target_when_no_detection": True},
        {"idle_target_topic": "/turret/idle_target"},
        {"map_frame": "map"},
        {"base_frame": "base_link"},
        {"tf_timeout": 0.05},

        # Tune sign/zero against your micro convention.
        {"yaw_sign": 1.0},
        {"yaw_zero_offset": 0.0},
        {"idle_yaw_rate_limit": 4.0},

        {"idle_recompute_period": 0.02},
        {"idle_yaw_deadband": 0.05},

        # Pitch: hold the current pitch by default. Use "static" or "target" if needed.
        {"idle_pitch_mode": "hold_last_micro"},
        {"idle_pitch_static": 0.0},
        {"idle_target_z": 0.42},
        {"gimbal_height": 0.42},

        # Fallback if /turret/idle_target or TF is unavailable.
        {"freeze_when_no_detection": True},
        # C++ turret_yaw_mux uses this to skip AA_RCLCPP_* wrappers entirely
        # when the mux log category is off. /turret/cmd arbitration is unchanged.
        {"enable_ros_logs": DEBUG_SETTINGS["log_turret_mux_enable"]},
    ]


def generate_launch_description():
    # TensorRT engine lives OUTSIDE the package (not installed into the
    # package share). Override order: --launch arg engine_path  >  env
    # AUTOAIM_ENGINE_PATH  >  this default.
    default_engine = os.environ.get("AUTOAIM_ENGINE_PATH", DEFAULT_ENGINE)

    engine_path = LaunchConfiguration("engine_path")
    serial_port = LaunchConfiguration("serial_port")


    serial_params = [
        {"shooting_active": True},
        {"rotating_chassis": True},

        {"serial_port": serial_port},
        {"serial_baudrate": 500000},
        {"serial_tx_hz": 100.0},
        {"serial_reconnect_interval": 2.0},
        {"serial_rx_timeout": 3.0},
        # Sentry routes autoaim through turret_yaw_mux so navigation can provide
        # idle aiming when CV is lost.
        {"turret_cmd_topic": "/turret/cmd"},
        {"nav_cmd_topic": "/cmd_vel_NAV"},
        {"micro_status_topic": "/micro_status"},
        {"enable_nav_pipeline": True},
        {"enable_turret_pipeline": True},
        {"hold_last_turret_when_disabled": True},
        # SAFETY: if /cmd_vel_AI goes stale for longer than this (autoaim node
        # crashed), the shoot flag sent to the micro is forced to 0. Without it
        # the bridge re-sends the last shoot=1 at 100 Hz forever.
        {"cmd_timeout": 0.3},
        # Header+CRC8 framing (self-resyncing, rejects corrupted packets).
        # REQUIRES matching micro firmware. Leave False until then.
        {"use_framed_protocol": False},
        # Shared by the C++ bridge and Python fallback. False skips active
        # serial log calls only; TX/RX, watchdogs, and micro_status continue.
        {"enable_ros_logs": DEBUG_SETTINGS["log_serial_enable"]},
    ]

    viewer_params = [
        # Must match micro_pitch_lock_opposite_sign in the autoaim params above,
        # otherwise the pitch-error numbers in the debug overlay are sign-flipped.
        {"micro_pitch_feedback_opposite_sign": False},
        {"fire_lock_yaw": 0.04},
        {"fire_lock_pitch": 0.05},
    ]

    # Camera selection: set DEFAULT_CAMERA above, or override with camera:=zed
    # / camera:=realsense at launch time. The detector is the ONLY
    # camera-specific node — autoaim/tracker/serial/viewer are
    # unchanged and never learn which camera is active (same topics either way).
    camera = LaunchConfiguration("camera")
    zed_condition = IfCondition(PythonExpression(["'", camera, "' == 'zed'"]))
    realsense_condition = IfCondition(PythonExpression(["'", camera, "' == 'realsense'"]))

    pkg_share = get_package_share_directory("autoaim")
    zed_config = os.path.join(pkg_share, "config", "zed.yaml")
    realsense_config = os.path.join(pkg_share, "config", "realsense.yaml")

    # The engine_path launch arg/env overrides whatever the sensor YAML sets.
    engine_override = {"engine_path": engine_path}

    return LaunchDescription([
        DeclareLaunchArgument("engine_path", default_value=str(default_engine)),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),

        # Sentry defaults to DEFAULT_CAMERA above.
        DeclareLaunchArgument(
            "camera", default_value=DEFAULT_CAMERA,
            description='Active camera detector: "realsense" or "zed".'),

        Node(
            package="autoaim",
            # C++ serial bridge. Fallback: executable="serial_bridge.py".
            executable="serial_bridge",
            name="micro_communications_node",
            parameters=serial_params,
            output="screen",
            arguments=ros_log_args("log_serial_enable", "log_level_serial"),
        ),

        Node(
            package="autoaim",
            executable="autoaim_node",
            name="autoaim",
            parameters=autoaim_params() + autoaim_debug_params(),
            output="screen",
            arguments=ros_log_args("log_autoaim_enable", "log_level_autoaim"),
        ),
        Node(
            package="autoaim",
            executable="turret_yaw_mux",
            name="turret_yaw_mux",
            parameters=turret_mux_params(),
            output="screen",
            arguments=ros_log_args("log_turret_mux_enable", "log_level_turret_mux"),
        ),
        Node(
            package="autoaim",
            executable="viewer_node.py",
            name="autoaim_viewer",
            parameters=viewer_params + python_log_params("log_viewer_enable"),
            output="screen",
            arguments=ros_log_args("log_viewer_enable", "log_level_viewer"),
            condition=IfCondition("true" if DEBUG_SETTINGS["viewer_node_enable"] else "false"),
        ),

        # ZED detector (camera:=zed). Camera/runtime config in config/zed.yaml.
        Node(
            package="autoaim",
            executable="zed_detector.py",
            name="zed_detector",
            parameters=[zed_config, engine_override] + detector_debug_params(),
            output="screen",
            arguments=ros_log_args("log_detector_enable", "log_level_detector"),
            condition=zed_condition,
        ),

        # RealSense detector (camera:=realsense). Config in config/realsense.yaml.
        # Same topics/messages as the ZED path.
        Node(
            package="autoaim_realsense",
            executable="realsense_detector",
            name="realsense_detector",
            parameters=[realsense_config, engine_override] + detector_debug_params(),
            output="screen",
            arguments=ros_log_args("log_detector_enable", "log_level_detector"),
            condition=realsense_condition,
        ),
    ])