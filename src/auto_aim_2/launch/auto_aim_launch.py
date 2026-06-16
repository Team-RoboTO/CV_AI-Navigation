from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    auto_aim_params = [

        # ── DETECTOR INPUT ──────────────────────────────────────────────────
        {'target_classes': ['0']},
        {'use_keypoints': True},
        {'keypoint_topic': '/detector/armors_keypoints'},
        {'min_keypoint_score': 0.0},
        {'max_reproj_error': 25.0},

        # ── DETECTION FILTERS ───────────────────────────────────────────────
        {'light_ratio': 0.85},
        {'max_armor_distance': 6.0},
        {'max_armor_z': 4.0},

        # ── TRACKER STATE MACHINE ────────────────────────────────────────────
        {'confirm_frames': 2},
        {'lost_timeout': 0.50},

        # ── EKF PROCESS NOISE ────────────────────────────────────────────────
        {'q_pos': 30.0},
        {'q_yaw': 40.0},
        {'q_r': 1e-6},

        # ── EKF MEASUREMENT NOISE ────────────────────────────────────────────
        {'r_pos_base': 0.05},
        {'r_pos_slope': 0.04},
        {'r_yaw_base': 0.05},
        {'r_yaw_slope': 0.005},
        {'max_oblique_deg': 65.0},

        # ── VELOCITY DAMPING ─────────────────────────────────────────────────
        {'alpha_pos': 0.98},
        {'alpha_yaw': 1.00},
        {'alpha_coast': 0.98},

        # ── ARMOR GEOMETRY ───────────────────────────────────────────────────
        {'initial_radius': 0.24},
        {'radius_ema_alpha': 0.05},
        # initial_dz [m]: height step between armor pairs (faces 1,3 vs 0,2).
        # If shots consistently miss the lower armor pair, flip sign to -0.05.
        {'initial_dz': 0.05},

        # ── BALLISTICS ───────────────────────────────────────────────────────
        {'bullet_speed': 25.0},
        {'gravity': 9.8},
        {'gimbal_height': 0.420},

        # ── BARREL OFFSET ────────────────────────────────────────────────────
        {'barrel_offset_x': 0.0},
        {'barrel_offset_y': 0.07},
        {'barrel_offset_z': -0.15},

        # ── FIRE GATE ────────────────────────────────────────────────────────
        {'angular_window': 1.0},
        {'window_ref_dist': 1.0},
        {'min_fire_dist': 0.2},
        {'max_fire_dist': 6.0},

        # ── TIMING / PREDICTION ──────────────────────────────────────────────
        # time_bias is critical for spinning targets.
        # At 300 RPM: 1ms error = 1.8 deg of rotation error.
        # If shots land behind the plate: reduce. In front: increase.
        {'time_bias': 0.04},
        {'ref_freq': 70.0},

        # ── FACE JUMP DETECTION ──────────────────────────────────────────────
        # yaw_jump_thresh [rad]: minimum yaw change to trigger face jump.
        # 0.70 rad = 40 deg. Prevents oblique PnP noise from triggering
        # false jumps at slow rotation speeds.
        {'yaw_jump_thresh': 0.55},

        # ── SPIN RATE ESTIMATION ─────────────────────────────────────────────
        {'use_vyaw_from_timing': True},
        # vyaw_timing_min_dt: ignore jumps faster than this [s].
        # 80ms = protects against false jumps from PnP noise.
        {'vyaw_timing_min_dt': 0.050},
        # vyaw_timing_max_dt: ignore jumps slower than this [s].
        # 400ms = active down to ~75 RPM (pi/2 / 0.4 = 3.9 rad/s).
        {'vyaw_timing_max_dt': 0.500},

        # ── SINGLE-FACE VS FOUR-FACE MODE ───────────────────────────────────
        # vyaw_fire_threshold [rad/s]: below this spin rate, only aim at the
        # currently visible face. Above it, use full 4-face spinning prediction.
        # This is the key fix for "goes crazy during slow rotation":
        # when |vyaw| < threshold the four phantom faces are NOT predicted,
        # so the aim stays locked on the visible plate.
        # 3.0 rad/s ≈ 29 RPM — anything slower is treated as stationary.
        # If your enemy spins at a minimum of 100 RPM (10.5 rad/s), you can
        # raise this to 8.0 to get even cleaner single-face tracking at low speed.
        {'vyaw_fire_threshold': 8.0},

        # ── MATCH GATES ──────────────────────────────────────────────────────
        {'max_match_dist': 0.8},
        {'maha_threshold': 16.9},

        # ── TARGET SWITCHING ────────────────────────────────────────────────
        {'switch_range_ratio': 0.85},
        {'switch_cooldown': 10},
        {'same_target_identity_dist': 1.0},

        # ── COMMAND SMOOTHING / LIMITS ───────────────────────────────────────
        {'cmd_smooth_alpha': 1.00},
        {'cmd_deadband_yaw': 0.005},
        {'cmd_deadband_pitch': 0.005},
        {'cmd_rate_limit_yaw': 0.0},
        {'cmd_rate_limit_pitch': 0.0},
        {'fire_lock_yaw': 0.06},
        {'fire_lock_pitch': 0.05},
        {'micro_pitch_feedback_opposite_sign': True},
        {'micro_pitch_lock_opposite_sign': False},

        # ── COMMAND SAFETY ───────────────────────────────────────────────────
        {'cmd_hold_time': 0.25},
        {'cmd_max_delta_yaw': 0.80},
        {'cmd_max_delta_pitch': 0.80},
        {'require_aim_inside_frame': False},

        # ── EGO-MOTION COMPENSATION ──────────────────────────────────────────
        {'use_ego_motion_compensation': True},
        {'ego_velocity_available': False},
        {'ego_velocity_body_frame': True},
        {'ego_velocity_scale_x': 1.0},
        {'ego_velocity_scale_y': 1.0},
        {'ego_velocity_max': 3.0},
        {'ego_position_max_drift': 0.0},
        {'chassis_heading_index': -1},

        # ── GIMBAL SIGNS ─────────────────────────────────────────────────────
        {'gimbal.yaw_sign': 1.0},
        {'gimbal.pitch_sign': -1.0},
    ]

    serial_params = [
        {'serial_port': '/dev/ttyACM0'},
        {'serial_baudrate': 500000},
        {'serial_tx_hz': 100.0},
        {'serial_reconnect_interval': 2.0},
        {'serial_rx_timeout': 3.0},
    ]

    viewer_params = [
        {'micro_pitch_feedback_opposite_sign': False},
    ]

    return LaunchDescription([
        Node(
            package='auto_aim_2',
            executable='serial_new_communication_USB_C_micro_imu_v9.py',
            name='micro_communications_node',
            parameters=serial_params,
            output='screen',
        ),
        Node(
            package='auto_aim_2',
            executable='auto_aim_2_node',
            name='auto_aim_2',
            parameters=auto_aim_params,
            output='screen',
        ),
        #Node(
        #    package='auto_aim_2',
        #    executable='viewer_node.py',
        #    name='auto_aim_viewer',
        #    parameters=viewer_params,
        #    output='screen',
        #),
        Node(
            package='auto_aim_2',
            executable='zed_yolo26_pose_keypoints_node_auto_aim2.py',
            name='zed_camera',
            parameters=viewer_params,
            output='screen',
        ),
    ])
