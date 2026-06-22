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
DEFAULT_CAMERA = "realsense"

# Main debug switches for the standard launch.
publish_debug_state = True
publish_other_debug_messages = False

def autoaim_params():
    return [
        {"publish_debug_state": publish_debug_state},
        {"publish_other_debug_messages": publish_other_debug_messages},

        # YOLO26 labels: 0=blue armor, 1=grey armor (ignored), 2=red armor.
        # True: read our team color from /micro_status[target_color_status_index]
        #       and pick the enemy class automatically via micro_color_target_classes.
        # False: always shoot the fixed class list in target_classes below.
        {"target_classes_from_micro_status": False},  # ON/OFF auto color from micro

        # micro_color_target_classes[i] = YOLO class to shoot when micro sends i.
        #   i=0 (we are red)  -> shoot blue -> "0"
        #   i=1 (we are blue) -> shoot red  -> "2"
        {"target_classes": ["0"]},  # used only if target_classes_from_micro_status=False
        {"target_color_status_index": 4},
        {"micro_color_target_classes": ["0", "2"]},
        {"use_keypoints": True},
        {"keypoint_topic": "/detector/armors_keypoints"},

        # 0.0 accepted garbage keypoints -> PnP jitter -> fire-lock dropouts.
        {"min_keypoint_score": 0.20},
        {"max_reproj_error": 30.0},

        # Use the gimbal angles AT THE IMAGE TIMESTAMP for the camera->odom
        # projection (ring buffer + interpolation) instead of the latest angles.
        # Fixes the "tracker is slow / lags behind" feel whenever the head moves.
        # zed_detector.py now stamps with sl.TIME_REFERENCE.IMAGE (capture time).
        {"angle_sync_enable": True},
        {"light_ratio": 0.85},
        {"max_armor_distance": 6.0},
        {"max_armor_z": 4.0},
        {"confirm_frames": 2},
        {"lost_timeout": 1.00},
        {"track_grace_misses": 2},
        # Grace handles single-frame dropouts. Keep TEMP_LOST firing disabled
        # unless testing shows the bounded coast-fire path is still needed.
        {"fire_in_temp_lost": False},
        {"temp_lost_fire_max": 3},

        # STATIC process noise. The adaptive/NIS q_pos controller was REMOVED:
        # it adapted only the horizontal center, but a spinner's center is ~still,
        # so it reacted to yaw/PnP model error and overshot the lead.
        #   q_pos [(m/s^2)^2]: center accel variance. sqrt(5)=2.2 m/s^2 RMS —
        #     enough lead for a translating standard chassis without chasing noise.
        #   q_yaw [(rad/s^2)^2]: spin accel variance. LOWERED 20 -> 7: 20 let vyaw
        #     absorb near-frontal PnP-yaw noise as PHANTOM spin on a static target
        #     (vyaw wandered to -2.8 rpm while the enemy stood still), which pushed
        #     the fire margin over its edge → fire/no-fire flicker (WORKLOG §4).
        #     The face-jump-timing gate, not q_yaw, does the fast spin-up
        #     convergence, so 7 keeps real spinners while killing the drift.
        {"q_pos": 10.0},
        {"q_yaw": 7.0},
        {"q_r": 1e-6},
        # Position measurement noise is ANISOTROPIC (WORKLOG §4b):
        #   r_pos_*      = RADIAL (depth / along line of sight) — PnP's weak axis,
        #                  large and range-growing.
        #   r_pos_tang_* = TANGENTIAL (lateral / bearing) — pixel-precise, small.
        # Splitting them lets the gimbal FOLLOW a moving enemy tightly without
        # depth noise destabilising the track. Lower r_pos_tang_* = tighter, more
        # responsive lateral follow (watch for jitter); raise toward r_pos_* to
        # recover the old isotropic over-smoothed behaviour.
        {"r_pos_base": 0.05},
        {"r_pos_slope": 0.04},
        {"r_pos_tang_base": 0.025},
        {"r_pos_tang_slope": 0.010},
        {"r_yaw_base": 0.05},
        {"r_yaw_slope": 0.005},
        {"max_oblique_deg": 75.0},

        # 0.98 at ~100 Hz decays the velocity estimate to ~13%/s — a constant
        # drag that under-leads translating targets. Let q_pos handle the noise.
        {"alpha_pos": 0.995},
        # alpha_yaw = 1.0 → NO spin damping. A 小陀螺 spins at a roughly constant
        # rate; damping vyaw between updates only makes the spin estimate decay
        # and the 4-face phase prediction lag behind. 0.995 quietly bled ~26%/s.
        {"alpha_yaw": 1.0},
        {"alpha_coast": 0.98},

        {"initial_radius": 0.24},
        {"radius_ema_alpha": 0.05},
        {"initial_dz": 0.0},
        {"adapt_dz_enable": False},

        {"bullet_speed": 25.0},
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
        {"barrel_offset_y": 0.02},     # <- MEASURE (lens-cover trick, see INSTRUCTIONS.md)
        {"barrel_offset_z": -0.05},

        # FIRE WINDOW (facing tolerance on the selected plate).
        # margin = win - |phase_error|,  win = angular_window * min(ref/range, 1.0).
        # The min() is now capped at 1.0 (was 2.0): the window only SHRINKS far
        # away, it no longer DOUBLES up close — that close-range expansion let the
        # shot fire on a plate up to ~46° off-facing (the "hits the wheels / 45°
        # to the side" bug). Budget: facing tol (~0.10) + vyaw_unc*horizon
        # (~0.05-0.18) ≈ 0.15-0.28 rad. 0.35 (20°) at/under 3 m is a safe start;
        # now that vyaw is gated you can tighten toward 0.25 for tighter groups.
        {"angular_window": 0.35},
        {"window_ref_dist": 3.0},
        {"min_fire_dist": 0.2},
        {"max_fire_dist": 6.0},
        # ── STATIC FIRE-FLICKER FIXES (WORKLOG §4) ──
        # The strict facing window above is a SPINNER timing gate. A non-spinning,
        # slightly-canted plate sat right on its edge while perfectly aimed, so
        # noise toggled fire on/off. These three relax/stabilise the static path
        # WITHOUT widening the spinner window (do NOT raise angular_window):
        #   static_facing_max: when |vyaw| < face_lookahead_min_vyaw, fire a LOCKED
        #     plate up to this facing angle (0.6 rad ≈ 34°) instead of the strict
        #     margin. Bullet-safe: still needs command_locked + range.
        {"static_facing_max": 0.6},
        #   yaw_facing_obs_floor: U-shaped yaw trust — distrust frontal PnP yaw
        #     (poorly observable) so it cannot drive phantom spin. Lower = stricter.
        {"yaw_facing_obs_floor": 0.05},
        #   ippe_alt_penalty: Mahalanobis hysteresis so the IPPE alternate yaw
        #     solution cannot flip frame-to-frame near face-on (the spike trigger).
        {"ippe_alt_penalty": 1.0},
        # Face lookahead is aim planning, not shot permission. The fire window
        # still decides when a shot is safe; lookahead pre-aims an incoming face
        # so the gimbal is settled when that face enters the unchanged window.
        {"face_lookahead_enable": True},
        {"face_lookahead_min_vyaw": 0.35},
        {"face_lookahead_horizon": 0.35},
        {"face_switch_hysteresis": 0.08},
        {"min_face_hold_time": 0.10},

        # PREDICTION HORIZON = measured(now − capture stamp) + actuation_latency.
        # actuation_latency is ONLY the post-command fixed delay (serial TX +
        # gimbal settle + muzzle exit). It must NOT include capture→aim — that is
        # measured per frame. The old 0.060 double-counted the pipeline, and at
        # high spin a too-long horizon LEADS the shot into the gap between plates:
        #   phase error = vyaw * Δt_horizon_error.
        #   At 300 RPM (31.4 rad/s) every 10 ms of horizon = 0.31 rad ≈ 18° of
        #   spin. So horizon accuracy of a few ms matters more than anything else
        #   on a spinner. Calibrate on a mover in 5 ms steps; the node logs the
        #   measured pipeline part every 2 s.
        {"use_measured_latency": True},
        {"actuation_latency": 0.030},
        # LEGACY fallback fixed horizon, used ONLY if use_measured_latency=False.
        # Then it must cover the FULL capture→muzzle latency (40-80 ms typical).
        {"LEGACY_time_bias": 0.045},

        # Must equal the REAL detection rate: ros2 topic hz /detector/armors_keypoints
        # (the ZED grabs at 120 fps — if inference keeps up this should be ~120).
        {"ref_freq": 60.0},
        # yaw_jump_thresh 1.20 rad (69°): a real face switch is ~90° (1.57 rad);
        # a 300 RPM spinner moves 31.4/60 = 0.52 rad between 60 Hz frames, so 1.20
        # sits cleanly between "normal frame" and "face switch".
        {"yaw_jump_thresh": 1.20},
        {"use_vyaw_from_timing": True},
        {"vyaw_timing_min_dt": 0.025},
        {"vyaw_timing_max_dt": 0.700},
        {"vyaw_conf_p_max": 2.0},
        # ── Anti-spurious-jump gate (stops a PnP flip / bad association from
        # locking a WRONG vyaw → 45° side shots). A jump only drives the spin
        # estimator when PnP reproj < vyaw_timing_max_reproj AND two consecutive
        # same-direction estimates agree within ±vyaw_timing_consistency.
        {"vyaw_timing_max_reproj": 8.0},     # [px] raise if good spins are ignored
        {"vyaw_timing_consistency": 0.35},   # ±35%; lower = stricter spin lock

        {"max_match_dist": 1.1},
        # maha_threshold: 4-DOF chi-square gate. 99% = 13.3, 99.9% = 18.5. 25 is
        # intentionally loose (R is obliquity-inflated, so visible-but-oblique
        # plates still pass and TRACKING does not flicker to TEMP_LOST). Lower
        # toward ~16 only if false detections are being associated.
        {"maha_threshold": 25.0},
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
        {"fire_lock_k_yaw": 0.0675},
        {"fire_lock_k_pitch": 0.0625},
        {"fire_lock_min": 0.015},
        {"fire_lock_max_yaw": 0.12},
        {"fire_lock_max_pitch": 0.10},

        # Pitch signs have two different jobs:
        # - feedback_opposite corrects geometry/PnP from raw micro feedback.
        # - lock_opposite converts raw feedback to command-echo convention.
        # On this firmware the measured stable target log is:
        #   micro pitch feedback ~= command echo, so lock_opposite is False.
        # Geometry still needs the feedback inversion with gimbal.pitch_sign=-1.
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
    # TensorRT engine lives OUTSIDE the package (not installed into the
    # package share). Override order: --launch arg engine_path  >  env
    # AUTOAIM_ENGINE_PATH  >  this default.
    default_engine = os.environ.get("AUTOAIM_ENGINE_PATH", DEFAULT_ENGINE)

    engine_path = LaunchConfiguration("engine_path")
    serial_port = LaunchConfiguration("serial_port")

    serial_params = [
        {"shooting_active": False},
        {"rotating_chassis": False},

        {"serial_port": serial_port},
        {"serial_baudrate": 500000},
        {"serial_tx_hz": 100.0},
        {"serial_reconnect_interval": 2.0},
        {"serial_rx_timeout": 3.0},
        # Standard has no turret_yaw_mux/navigation pipeline in this launch.
        # Feed autoaim directly into the bridge's turret-command slot and force
        # nav TX fields to zero from launch params, without special bridge code.
        {"turret_cmd_topic": "/cmd_vel_AI"},
        {"nav_cmd_topic": "/cmd_vel_NAV"},
        {"micro_status_topic": "/micro_status"},
        {"enable_nav_pipeline": False},
        {"enable_turret_pipeline": True},
        {"hold_last_turret_when_disabled": True},
        # SAFETY: if /cmd_vel_AI goes stale for longer than this (autoaim node
        # crashed), the shoot flag sent to the micro is forced to 0. Without it
        # the bridge re-sends the last shoot=1 at 100 Hz forever.
        {"cmd_timeout": 0.3},
        # Header+CRC8 framing (self-resyncing, rejects corrupted packets).
        # REQUIRES matching micro firmware. Leave False until then.
        {"use_framed_protocol": False},
    ]

    viewer_params = [
        # Must match micro_pitch_lock_opposite_sign in the autoaim params above,
        # otherwise the pitch-error numbers in the debug overlay are sign-flipped.
        {"micro_pitch_feedback_opposite_sign": False},
        {"use_debug_state": publish_debug_state},
        {"debug_state_topic": "/debug_state"},
        {"fire_lock_yaw": 0.05},
        {"fire_lock_pitch": 0.04},
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

        # Standard defaults to DEFAULT_CAMERA above.
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

        # ZED detector (camera:=zed). Camera/runtime config in config/zed.yaml.
        Node(
            package="autoaim",
            executable="zed_detector.py",
            name="zed_detector",
            parameters=[zed_config, engine_override],
            output="screen",
            condition=zed_condition,
        ),

        # RealSense detector (camera:=realsense). Config in config/realsense.yaml.
        # Same topics/messages as the ZED path.
        Node(
            package="autoaim_realsense",
            executable="realsense_detector",
            name="realsense_detector",
            parameters=[realsense_config, engine_override],
            output="screen",
            condition=realsense_condition,
        ),
    ])
