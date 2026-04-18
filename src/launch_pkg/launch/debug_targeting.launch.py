import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    launch_pkg_dir = get_package_share_directory('launch_pkg')

    realsense_config_file_path = os.path.join(
        launch_pkg_dir, 'config', 'sensors', 'realsense.yaml'
    )
    realsense_json_file_path = os.path.join(
        launch_pkg_dir, 'config', 'sensors', 'realsense.json'
    )

    pose_source_arg = DeclareLaunchArgument(
        'pose_source',
        default_value='camera_imu',
        description="Orientation source for auto_aim_targeting: 'micro_pose', 'camera_imu', or 'none'."
    )
    pose_source = LaunchConfiguration('pose_source')
    filtered_imu_topic = '/camera/filtered_imu'

    realsense_node = ComposableNode(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[
            realsense_config_file_path,
            {'json_file_path': realsense_json_file_path, 'publish_tf': False},
        ],
    )

    imu_filter_node = ComposableNode(
        name='imu_filter',
        package='imu_filter_madgwick',
        plugin='ImuFilterMadgwick',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
        }],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
            ('imu/data', filtered_imu_topic),
        ],
    )

    encoder_node = ComposableNode(
        name='dnn_image_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        remappings=[('encoded_tensor', 'tensor_pub'),
                    ('image', '/camera/camera/color/image_raw')],
        parameters=[{
            'input_image_width': 640,
            'input_image_height': 480,
            'network_image_width': 640,
            'network_image_height': 640,
            'image_mean': [0.0, 0.0, 0.0],
            'image_stddev': [1.0, 1.0, 1.0],
            'num_blocks': 80,
            'flip_horizontal': False,
            'keep_aspect_ratio': True,
        }],
    )

    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            'model_file_path': os.path.join(launch_pkg_dir, 'resources', 'yolov8_op.onnx'),
            'engine_file_path': os.path.join(launch_pkg_dir, 'resources', 'yolov8_op.plan'),
            'output_binding_names': ['output0'],
            'output_tensor_names': ['output_tensor'],
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['images'],
            'verbose': False,
            'force_engine_update': False,
            'enable_fp16': True,
            'relaxed_dimension_check': False,
            'num_blocks': 80,
        }],
    )

    yolov8_decoder_node = ComposableNode(
        name='yolov8_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[{
            'confidence_threshold': 0.10,
            'nms_threshold': 0.45,
            'num_classes': 5,
            'in_width': 640.0,
            'in_height': 480.0,
            'out_width': 640.0,
            'out_height': 480.0,
            'tf_frame_id': 'camera_color_optical_frame',
        }],
        remappings=[('detections', '/detections_output')],
    )

    targeting_node = ComposableNode(
        package='auto_aim_targeting',
        plugin='rm_auto_aim::AutoAimNode',
        name='auto_aim_targeting',
        remappings=[
            ('/detector/armors', '/detections_output'),
            ('/camera_info', '/camera/camera/color/camera_info'),
            ('/imu/data', filtered_imu_topic),
        ],
        parameters=[
            {'target_frame': 'odom'},
            {'target_classes': ['3']},
            {'max_armor_distance': 10.0},
            {'light_ratio': 0.85},
            {'bbox_padding_y': 80.0},
            {'pnp.max_reprojection_error': 10.0},
            {'tracker.max_match_distance': 0.30},
            {'tracker.max_track_range': 6.0},
            {'tracker.tracking_thres': 4},
            {'tracker.lost_time_thres': 0.5},
            {'tracker.max_trackers': 5},
            {'tracker.new_tracker_min_dist': 0.5},
            {'tracker.new_tracker_assumed_radius': 0.26},
            {'tracker.initial_r1': 0.22},
            {'tracker.initial_r2': 0.30},
            {'tracker.r_kf_Q': 3.3e-8},
            {'tracker.r_kf_R': 4.0e-4},
            {'tracker.r_kf_P_init': 6.4e-3},
            {'tracker.r_adapt_max_dist': 4.0},
            {'tracker.dz_adapt_alpha': 0.05},
            {'tracker.r_yaw_uncertainty_scale': 50.0},
            {'tracker.maha_match_threshold': 13.3},
            {'tracker.maha_jump_threshold': 20.0},
            {'ekf.sigma2_q_xyz': 5.0},
            {'ekf.sigma2_q_yaw': 10.0},
            {'ekf.sigma2_q_r': 1.0e-6},
            {'ekf.xyz_damping_alpha': 0.95},
            {'ekf.yaw_damping_alpha': 0.95},
            {'ekf.coast_damping_factor': 0.85},
            {'ekf.damping_innov_threshold': 0.05},
            {'ekf.yaw_innov_threshold': 0.15},
            {'ekf.ref_frequency': 30.0},
            {'ekf.accel_ema_alpha': 0.30},
            {'ekf.r_xyz_base': 0.04},
            {'ekf.r_xyz_slope': 0.03},
            {'ekf.r_yaw_base': 0.05},
            {'ekf.r_yaw_slope': 0.002},
            {'ekf.r_yaw_angle_power': 4.0},
            {'ekf.max_yaw_oblique_deg': 65.0},
            {'ekf.secondary_face_fusion': True},
            {'ekf.secondary_r_inflation': 2.0},
            {'ekf.secondary_maha_threshold': 13.3},
            {'tracker.v_yaw_max': 15.0},
            {'tracker.stationary_measurement_threshold': 0.03},
            {'tracker.stationary_innovation_threshold': 0.05},
            {'tracker.stationary_speed_threshold': 0.35},
            {'tracker.stationary_yaw_rate_threshold': 0.60},
            {'tracker.stationary_required_frames': 4},
            {'tracker.stationary_velocity_collapse_factor': 0.25},
            {'tracker.stationary_zero_velocity_threshold': 0.05},
            {'tracker.visible_direct_obliquity_threshold_deg': 60.0},
            {'pose_source': pose_source},
            {'micro_pose_timeout': 0.15},
            {'tracker.self_orientation_timeout': 0.2},
            {'gimbal.height': 0.325},
            {'gimbal.yaw_sign': 1.0},
            {'gimbal.pitch_sign': 1.0},
            {'camera_offset_x': 0.107},
            {'camera_offset_z': 0.136},
            {'bullet_speed': 25.0},
            {'gravity': 9.8},
            {'linear_drag_coeff': 0.01},
            {'quadratic_drag_coeff': 0.01},
            {'use_quadratic_drag': False},
            {'time_bias': 0.05},
            {'time_bias_alpha': 0.35},
            {'gimbal_response_delay': 0.0},
            {'min_fire_dist': 0.5},
            {'max_fire_dist': 10.0},
            {'angular_window': 0.09},
            {'angular_window_ref_dist': 3.0},
            {'max_measurement_age': 0.15},
            {'detector_stall_timeout': 0.20},
            {'max_gimbal_yaw_rate': 6.0},
            {'max_gimbal_pitch_rate': 4.0},
            {'fire_yaw_tolerance': 0.03},
            {'fire_pitch_tolerance': 0.03},
            {'max_cmd_angle': 30.0},
            {'max_cmd_yaw_angle': 180.0},
            # cmd_smooth_alpha removed — no EMA smoothing on gimbal commands
            {'debug.allow_fire_without_pose': True},
            {'latency_gate_sigma': 2.5},
            {'latency_warmup_samples': 5},
            {'indirect_vyaw_threshold': 3.0},
            {'indirect_timing_tolerance': 0.02},
            {'indirect_max_candidates': 8},
            {'oblique_exponent': 2.0},
            {'cost.range': 0.30},
            {'cost.flight_time': 0.15},
            {'cost.uncertainty': 0.35},
            {'cost.slew': 0.25},
            {'cost.switch_target': 0.35},
            {'cost.switch_face': 0.20},
            {'cost.staleness': 0.25},
            {'cost.temp_lost': 0.70},
            {'cost.low_visibility': 0.35},
            {'cost.negative_margin': 1.00},
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='biggest_container',
        namespace='very_big',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            realsense_node,
            imu_filter_node,
            encoder_node,
            tensor_rt_node,
            yolov8_decoder_node,
            targeting_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    return LaunchDescription([
        pose_source_arg,
        TimerAction(period=2.0, actions=[container]),
    ])
