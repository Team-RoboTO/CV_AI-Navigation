from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Node-only targeting profile for the Jetson-64/ZED setup.
    # ZED wrapper, detector, and the lower-computer /micro_imu bridge
    # are expected to be launched externally by robot bringup.
    detector_topic = LaunchConfiguration('detector_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    micro_imu_topic = LaunchConfiguration('micro_imu_topic')
    twist_output_topic = LaunchConfiguration('twist_output_topic')

    auto_aim_node = Node(
        package='auto_aim_targeting',
        executable='auto_aim_targeting_node',
        name='auto_aim_targeting',
        remappings=[
            ('/detector/armors', detector_topic),
            ('/camera_info', camera_info_topic),
            ('/micro_imu', micro_imu_topic),
            ('/cmd_vel', twist_output_topic),
        ],
        parameters=[
            {'target_frame': 'odom'},
            {'target_classes': ['0']},  # 0 for BLUE, 3 for RED

            # Detection / PnP
            {'light_ratio': 0.85},
            {'bbox_padding_y': 0.0},
            {'max_armor_distance': 8.0},
            {'pnp.max_reprojection_error': 10.0},
            {'pnp.correct_yaw': True},
            {'pnp.yaw_correction_max_oblique_deg': 65.0},

            # Tracker state machine
            {'tracker.max_trackers': 1},
            {'tracker.tracking_thres': 3},
            {'tracker.lost_time_thres': 0.4},
            {'tracker.max_match_distance': 1.5},
            {'tracker.max_track_range': 8.0},
            {'tracker.new_tracker_min_dist': 0.5},
            {'tracker.new_tracker_assumed_radius': 0.24},

            # Geometry / EKF
            {'tracker.initial_r1': 0.24},
            {'tracker.initial_r2': 0.24},
            {'tracker.r_kf_Q': 3.3e-8},
            {'tracker.r_kf_R': 4.0e-4},
            {'tracker.r_kf_P_init': 6.4e-3},
            {'tracker.r_adapt_max_dist': 4.0},
            {'tracker.dz_adapt_alpha': 0.05},
            {'tracker.r_yaw_uncertainty_scale': 50.0},
            {'tracker.v_yaw_max': 25.0},
            {'tracker.maha_match_threshold': 13.3},
            {'tracker.maha_jump_threshold': 20.0},
            {'ekf.sigma2_q_xyz': 5.0},
            {'ekf.sigma2_q_yaw': 10.0},
            {'ekf.sigma2_q_r': 1.0e-6},
            {'ekf.xyz_damping_alpha': 0.90},
            {'ekf.yaw_damping_alpha': 0.90},
            {'ekf.coast_damping_factor': 0.70},
            {'ekf.damping_innov_threshold': 0.10},
            {'ekf.yaw_innov_threshold': 0.15},
            {'ekf.ref_frequency': 30.0},
            {'ekf.accel_ema_alpha': 0.30},
            {'ekf.r_xyz_base': 0.05},
            {'ekf.r_xyz_slope': 0.04},
            {'ekf.r_yaw_base': 0.05},
            {'ekf.r_yaw_slope': 0.005},
            {'ekf.r_yaw_angle_power': 4.0},
            {'ekf.max_yaw_oblique_deg': 65.0},
            {'ekf.secondary_face_fusion': False},
            {'ekf.secondary_r_inflation': 2.0},
            {'ekf.secondary_maha_threshold': 13.3},

            # Pose / extrinsics. These preserve the refactor's height-only
            # camera transform; calibrate camera_offset_* before changing.
            {'pose_source': 'micro_imu'},
            {'micro_pose_timeout': 0.15},
            {'tracker.self_orientation_timeout': 0.2},
            {'gimbal.height': 0.325},
            {'camera_offset_x': 0.0},
            {'camera_offset_z': 0.0},
            {'gimbal.yaw_sign': 1.0},
            {'gimbal.pitch_sign': 1.0},

            # Ballistics / barrel offset
            {'bullet_speed': 25.0},
            {'gravity': 9.8},
            {'linear_drag_coeff': 0.01},
            {'quadratic_drag_coeff': 0.01},
            {'use_quadratic_drag': False},
            {'gimbal_pitch_min': -1.2},
            {'gimbal_pitch_max': 1.2},
            {'barrel_offset_x': 0.0},
            {'barrel_offset_y': 0.0},
            {'barrel_offset_z': -0.10},

            # Fire / timing
            {'time_bias': 0.05},
            {'time_bias_alpha': 0.35},
            {'gimbal_response_delay': 0.0},
            {'min_fire_dist': 0.3},
            {'max_fire_dist': 8.0},
            {'angular_window': 0.09},
            {'angular_window_ref_dist': 3.0},
            {'max_measurement_age': 0.30},
            {'detector_stall_timeout': 0.20},
            {'max_gimbal_yaw_rate': 6.0},
            {'max_gimbal_pitch_rate': 4.0},
            {'fire_yaw_tolerance': 0.03},
            {'fire_pitch_tolerance': 0.03},
            {'debug.allow_fire_without_pose': False},
            {'latency_gate_sigma': 2.5},
            {'latency_warmup_samples': 5},

            # Planner / output
            {'indirect_vyaw_threshold': 3.0},
            {'indirect_timing_tolerance': 0.02},
            {'indirect_max_candidates': 8},
            {'oblique_exponent': 2.0},
            {'max_cmd_angle': 30.0},
            {'max_cmd_yaw_angle': 180.0},
            {'cmd_smooth_alpha': 1.0},
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'detector_topic', default_value='/detector/armors'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera_info'),
        DeclareLaunchArgument('micro_imu_topic', default_value='/micro_imu'),
        DeclareLaunchArgument(
            'twist_output_topic', default_value='/cmd_vel_AI'),
        auto_aim_node,
    ])
