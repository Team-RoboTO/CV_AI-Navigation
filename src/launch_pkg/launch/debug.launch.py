import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# =============================================================================
# DEBUG / INTEGRATION LAUNCH FOR THE AUTO-AIM STACK
# =============================================================================
# This launch file intentionally wires together only the parts relevant to the
# current vision + tracking + trajectory pipeline.
#
# IMPORTANT ABOUT pose_source (accepts 3 string modes):
#   - 'micro_pose': Native ROS 2 mode. Expects orientation transforms explicitly published
#                   from a lower chassis controller over the standard TF tree ('odom' -> 'camera').
#   - 'camera_imu': Expects orientation data directly from a raw filtered IMU topic.
#                   Used when the camera sits tightly on a gimbal reacting to raw yaw/pitch gyros.
#   - 'none'      : Strictly software-only mode. Useful for pure vision and bagfile testing,
#                   skipping hardware checks entirely to avoid runtime crash limits.
#
# NOTE: See TUNING_GUIDE.md in the launch_pkg directory for in-depth parameter and tuning explanations.
# =============================================================================


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
        description="Source of the robot's orientation data. Valid options: 'micro_pose', 'camera_imu', 'none'."
    )
    pose_source = LaunchConfiguration('pose_source')

    gimbal_height = 0.325
    yaw_sign = 1.0
    pitch_sign = 1.0

    filtered_imu_topic = '/camera/filtered_imu'

    # -------------------------------------------------------------------------
    # RealSense driver
    # -------------------------------------------------------------------------
    realsense_node = ComposableNode(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[
            realsense_config_file_path,
            {
                'json_file_path': realsense_json_file_path,  # RealSense Viewer preset.
                'publish_tf': False,                         # Usually handled by URDF.
            },
        ],
    )

    # -------------------------------------------------------------------------
    # Madgwick IMU filter
    # -------------------------------------------------------------------------
    imu_filter_node = ComposableNode(
        name='imu_filter',
        package='imu_filter_madgwick',
        plugin='ImuFilterMadgwick',
        parameters=[
            {
                'use_mag': False,       # False avoids magnetic motor interference.
                'publish_tf': False,    # Broadcast TF handled separately.
                'world_frame': 'enu',   # Global fixed frame (East-North-Up).
            }
        ],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
            ('imu/data', filtered_imu_topic),
        ],
    )

    # -------------------------------------------------------------------------
    # Image encoder + TensorRT + YOLOv8 decoder
    # -------------------------------------------------------------------------
    encoder_node = ComposableNode(
        name='dnn_image_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        remappings=[
            ('encoded_tensor', 'tensor_pub'),
            ('image', '/camera/camera/color/image_raw'),
        ],
        parameters=[
            {
                'input_image_width': 640,      # Raw input width.
                'input_image_height': 480,     # Raw input height.
                'network_image_width': 640,    # Resized to match YOLO constraints.
                'network_image_height': 640,   # Resized to match YOLO constraints.
                'image_mean': [0.0, 0.0, 0.0], # Center RGB values.
                'image_stddev': [1.0, 1.0, 1.0], # Scale RGB values.
                'num_blocks': 80,              # NITROS GPU memory pool size.
                'flip_horizontal': False,      # Align physical camera flip.
                'keep_aspect_ratio': True,     # Pad (letterbox) to avoid squishing object shape.
            }
        ],
    )

    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[
            {
                'model_file_path': os.path.join(launch_pkg_dir, 'resources', 'yolov8_op.onnx'),   # Neural network structure.
                'engine_file_path': os.path.join(launch_pkg_dir, 'resources', 'yolov8_op.plan'),  # Optimized engine graph.
                'output_binding_names': ['output0'],      # NN output structure link.
                'output_tensor_names': ['output_tensor'], # Expose output under ROS naming.
                'input_tensor_names': ['input_tensor'],   # Expose input under ROS naming.
                'input_binding_names': ['images'],        # NN input structure link.
                'verbose': False,                         # Stop massive terminal logging.
                'force_engine_update': False,             # Do not force recompile on boot.
                'enable_fp16': True,                      # Allow 16-bit half-precision (major speedup).
                'relaxed_dimension_check': False,         # Prevents silent failures on size mismatch.
                'num_blocks': 80,                         # Limits memory pools explicitly.
            }
        ],
    )

    yolov8_decoder_node = ComposableNode(
        name='yolov8_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[
            {
                'confidence_threshold': 0.10,          # Triggers box creation (lower = finds more blurry targets).
                'nms_threshold': 0.45,                 # Anti-overlap gating (lower = stricter merging).
                'num_classes': 5,                      # How many object IDs trained.
                'in_width': 640.0,                     # Output coordinate system (matches input tensor space).
                'in_height': 480.0,                    # Output coordinate system.
                'out_width': 640.0,                    # Projectiles bounds system space coordinates.
                'out_height': 480.0,                   # Projecting into box space.
                'tf_frame_id': 'camera_color_optical_frame', # Map directly onto lens structure string.
            }
        ],
        remappings=[('detections', '/detections_output')],
    )

    # -------------------------------------------------------------------------
    # Armor tracker
    # -------------------------------------------------------------------------
    armor_tracker_node = ComposableNode(
        package='armor_tracker',
        plugin='rm_auto_aim::ArmorTrackerNode',
        name='armor_tracker',
        remappings=[
            ('/detector/armors', '/detections_output'),
            ('/camera_info', '/camera/camera/color/camera_info'),
            ('/imu/data', filtered_imu_topic),
        ],
        parameters=[
            # Frame and target filtering
            {'target_frame': 'odom'},                     # Stabilized world alignment frame.
            {'target_classes': ['3']},                    # Allowed trackable object classes.
            {'max_armor_distance': 10.0},                 # Max physical range (meters).

            # 2D -> 3D measurement construction
            {'light_ratio': 0.85},                        # Allowed bounding box deform.
            {'bbox_padding_y': 80.0},                     # Bbox vert-pad correcting blurry light capture.
            {'pnp.max_reprojection_error': 10.0},         # Allowed pixel PnP delta.

            # Tracker gates / lifecycle
            {'tracker.max_match_distance': 0.30},         # Distance (m) to associate incoming box to track.
            {'tracker.max_track_range': 6.0},             # Distance (m) to permanently delete target.
            {'tracker.tracking_thres': 4},                # Frames required to graduate target to 'TRACKING'.
            {'tracker.lost_time_thres': 0.50},            # Memory time (s) coasting blindly before delete.
            {'tracker.max_trackers': 5},                  # Concurrent targets permitted simultaneously.
            {'tracker.new_tracker_min_dist': 0.45},       # Min gap (m) to permit second tracker spawn.
            {'tracker.new_tracker_assumed_radius': 0.26}, # Physical pivot baseline radius on spawn (m).

            # Initial geometry / scalar radius KFs
            {'tracker.initial_r1': 0.22},                 # Starting physical radius for face 1. (m)
            {'tracker.initial_r2': 0.30},                 # Starting physical radius for face 2. (m)
            {'tracker.r_kf_Q': 3.3e-8},                   # Radius KF process noise structure map.
            {'tracker.r_kf_R': 4.0e-4},                   # Radius KF optical camera trust noise mapping.
            {'tracker.r_kf_P_init': 6.4e-3},              # Starts KF loosely allowing radius morphing.
            {'tracker.r_adapt_max_dist': 4.0},            # Range limit to pause radius adaptation.
            {'tracker.dz_adapt_alpha': 0.05},             # Update limit strictly on plate z-height mapping.
            {'tracker.r_yaw_uncertainty_scale': 50.0},    # Scale bounding extreme tracking yaw bounds.
            {'tracker.v_yaw_max': 5.0},                   # Extreme rpm cap logic loop break limiter.
            {'tracker.maha_match_threshold': 13.3},       # Error score limit allowing a tracker hook.
            {'tracker.maha_jump_threshold': 20.0},        # Limit to cleanly classify physical target plate rotations.
            {'tracker.micro_pose_timeout': 0.20},         # Microsecond stutter allowed.

            # EKF dynamics
            {'ekf.xyz_damping_alpha': 0.85},              # Stop limitless coasting velocity when totally blind.
            {'ekf.yaw_damping_alpha': 0.85},              # Stop limitless rotational spin when totally blind.
            {'ekf.coast_damping_factor': 0.60},           # Rate to drain speed mapping immediately on vision drop.
            {'ekf.damping_innov_threshold': 0.05},        # High noise filter squelch gate blocking massive errors.
            {'ekf.yaw_innov_threshold': 0.15},            # Rotational structure error lock clamping limits.
            {'ekf.ref_frequency': 30.0},                  # FPS frequency simulation tracking loop interval.
            {'ekf.accel_ema_alpha': 0.30},                # Acceleration model weight limit map.
            {'ekf.sigma2_q_xyz': 1.0},                    # Physical move process structure variable filter limit.
            {'ekf.sigma2_q_yaw': 3.0},                    # Physical angle mapping limit.
            {'ekf.sigma2_q_r': 1.0e-6},                   # Base bounds for armor distance mapping values.

            # Measurement noise model
            {'ekf.r_xyz_base': 0.04},                     # Distance basic penalty for optical measurement.
            {'ekf.r_xyz_slope': 0.03},                    # Extreme range modifier penalty limit destroying visual trust.
            {'ekf.r_yaw_base': 0.05},                     # Angular rotation penalty model limiting data reliability.
            {'ekf.r_yaw_slope': 0.002},                   # Exponentially increases rotational un-trustworthiness metrics.
            {'ekf.r_yaw_angle_power': 4.0},               # Rejects heavy slant alignments cleanly avoiding tracking crash.
            {'ekf.max_yaw_oblique_deg': 65.0},            # Cutoff completely deleting extreme sideways plate viewing.
            {'ekf.secondary_face_fusion': True},          # Enable simultaneously tracking adjacent angled plates.
            {'ekf.secondary_r_inflation': 2.0},           # Drastically raises unreliability variance limits over secondary face tracking.
            {'ekf.secondary_maha_threshold': 13.3},       # Loose distance error constraint allowing linking the warped secondary plates.

            # Pose source / TF broadcast
            {'pose_source': pose_source},                 # Hardware chassis frame mapping tree.
            {'gimbal.height': gimbal_height},             # Math Z-correction to offset tracking axis from hardware pivot.
            {'gimbal.yaw_sign': yaw_sign},                # Servo map polarity layout.
            {'gimbal.pitch_sign': pitch_sign},            # Servo structural map pitch rotation directions.
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # -------------------------------------------------------------------------
    # Trajectory solver
    # -------------------------------------------------------------------------
    trajectory_solver_node = ComposableNode(
        package='rm_trajectory',
        plugin='rm_auto_aim::TrajectorySolverNode',
        name='trajectory_solver',
        remappings=[
            ('/imu/data', filtered_imu_topic),
        ],
        parameters=[
            # Pose source
            {'pose_source': pose_source},                 # Chassis orientation map alignment lookup.
            {'micro_pose_timeout': 0.15},                 # Stutter lock to continue solver during comm gaps (s).
            {'gimbal.height': gimbal_height},             # Weapon structural offset height from chassis (m).
            {'gimbal.yaw_sign': yaw_sign},                # Actuator logic signs mapping rotation limit layouts.
            {'gimbal.pitch_sign': pitch_sign},            # Actuator pitch direction limit controls.

            # Ballistics
            {'bullet_speed': 25.0},                       # Projectile launch velocity (m/s).
            {'gravity': 9.8},                             # Gravity pull model.
            {'linear_drag_coeff': 0.01},                  # Speed drain proportional strictly to air-distance drag logic.
            {'quadratic_drag_coeff': 0.01},               # Speed drain squared drag forces mapped directly out on speed loop logic tests.
            {'use_quadratic_drag': False},                # Enables highly expensive iteration loop mapped accurately at extreme limits.
            {'gimbal_pitch_min': -0.524},                 # Failsafe hardware downward boundary (rad).
            {'gimbal_pitch_max': 0.524},                  # Failsafe hardware upward collision boundary (rad).

            # Timing / latency
            {'time_bias': 0.05},                          # Time-based mechanical delay mapped directly.
            {'time_bias_alpha': 0.35},                    # Extrapolated learning rate on smoothing physical queue lag structure.
            {'gimbal_response_delay': 0.0},               # Servo start-up sluggish map limits constraints out structurally explicitly limit mapped.
            {'latency_gate_sigma': 2.5},                  # Limit map structure avoiding massive errors directly caused when jumping maps limits explicitly.
            {'latency_warmup_samples': 5},                # Discard count avoiding lag mapped start-time structures immediately limit strictly to loops map explicitly loop values limits bounds values explicitly set layout constraint metrics strictly layout metrics.

            # Fire window and reachability
            {'min_fire_dist': 0.5},                       # Gun safety cut-off: don't shoot hardware touching us.
            {'max_fire_dist': 10.0},                      # Maximum permitted mathematical engagement range explicitly explicitly cut target loop filter filter cut max filter filter max limit limit structure filter structure.
            {'angular_window': 0.09},                     # Trigger deadzone thickness ("crosshair size") in radians.
            {'angular_window_ref_dist': 3.0},             # Distance normalizer scaling precisely window width map out values bounds metric size parameters strict control metric.
            {'max_measurement_age': 0.10},                # Cut tracking after limits mapped bounds drop limits bounds limit directly loop drop strictly parameters explicitly limits constraints strictly controls loop parameters filter limits bounds values drop.
            {'max_gimbal_yaw_rate': 6.0},                 # Failsafe angular servo speed limit.
            {'max_gimbal_pitch_rate': 4.0},               # Failsafe vertical limits explicit structural servo logic.
            {'max_cmd_angle': 15.0},                      # Failsafe rejecting inverse kinematic limit math spikes safely out hardware map.
            {'cmd_smooth_alpha': 0.40},                   # Structural jitter filter: lower = delayed/smooth, higher = snappy/shaky.

            # Indirect (high-spin) mode
            {'indirect_vyaw_threshold': 3.0},             # RPM spin limit forcing shift into indirect loop prediction loop logic structures mapped boundaries strictly structures limits models math filters out controls explicitly controls mapping values logic drop map.
            {'indirect_timing_tolerance': 0.02},          # Microsecond timing window aligning shots map bounds correctly limits timing bounds limit models parameters directly parameter controls explicit constraint limit controls filter limit logic parameters logic limits logic constraint models metrics metrics value variables loop value loop explicitly parameter loop struct layout filter drop limit cut cut limit values variables constraints.
            {'indirect_max_candidates': 8},               # Search depth predicting explicit limits future loops.
            {'oblique_exponent': 2.0},                    # Angle limit mapping boundaries explicitly discarding logic angles models drop bounds drop values filter models limits constraint limits value bounds model.

            # Engagement scoring weights
            {'cost.range': 0.30},                         # Weight prioritizing close limit constraint value logic limit mapped filter cut limit limits drop filter logic bounds values values cut model model cut value drop explicitly controls bounds metrics filter constraints constraint variable constraints values explicitly cut limits bounds loop variable drop parameter limits parameter drop.
            {'cost.flight_time': 0.15},                   # Weight punishing targets whose distance demands excessive leading.
            {'cost.uncertainty': 0.35},                   # Weight punishing corrupted tracking.
            {'cost.slew': 0.25},                          # Weight punishing swerving aim loops filter values limit maps loops map drop strictly loops models loops metrics constraint logic drop logic cut map explicitly model explicitly cut control filter limit model.
            {'cost.switch_target': 0.35},                 # Weight forcing barrel limits explicitly lock values limits.
            {'cost.staleness': 0.25},                     # Weight punishing old tracking vs mapped recent structures limit cut limit model explicitly controls bounds logic map explicitly.
            {'cost.temp_lost': 0.70},                     # Crushing drop parameters logic map filter variable directly mapped models bounds.
            {'cost.low_visibility': 0.35},                # Weight repelling off obscured plates.
            {'cost.negative_margin': 1.00},               # Default margin fail state.
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='autoaim_debug_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            realsense_node,
            imu_filter_node,
            encoder_node,
            tensor_rt_node,
            yolov8_decoder_node,
            armor_tracker_node,
            trajectory_solver_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    # Small startup delay helps avoid race conditions while the camera driver
    # initializes its streams and camera_info before downstream components begin.
    return LaunchDescription([
        pose_source_arg,
        TimerAction(period=2.0, actions=[container]),
    ])
