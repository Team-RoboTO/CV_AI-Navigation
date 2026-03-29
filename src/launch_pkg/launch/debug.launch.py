# # SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# # Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# # http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.
# #
# # SPDX-License-Identifier: Apache-2.0

# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import ComposableNodeContainer, Node
# from launch_ros.descriptions import ComposableNode
# from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
# from launch.conditions import IfCondition, UnlessCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration


# def generate_launch_description():


#     launch_pkg_dir = get_package_share_directory('launch_pkg')

#     # RealSense
#     realsense_config_file_path = os.path.join(
#         launch_pkg_dir,
#         'config','sensors','realsense.yaml'
#     )

    
#     # Realsense
#     realsense_node = ComposableNode(
#         package='realsense2_camera',
#         plugin='realsense2_camera::RealSenseNodeFactory',
#         parameters=[realsense_config_file_path],
#         remappings=[('/color/image_raw', '/image_origin')]
#     )

#     image_flip = pointcloud_node = ComposableNode(
#         name='image_flip',
#         package='isaac_ros_image_proc',
#         plugin='nvidia::isaac_ros::image_proc::ImageFlipNode',
#         parameters= [{
#             'flip_mode' : 'VERTICAL'
#         }],
#         remappings=[('/image', '/image_origin'),
#                    ('image_flipped', '/image')]
#                    )
    
#     depth_image_flip = pointcloud_node = ComposableNode(
#         name='depth_image_flip',
#         package='isaac_ros_image_proc',
#         plugin='nvidia::isaac_ros::image_proc::ImageFlipNode',
#         parameters= [{
#             'flip_mode' : 'VERTICAL'
#         }],
#         remappings=[('/image', '/depth/image_rect_raw'),
#                    ('image_flipped', '/depth/image_rect_flipped')]
#                    )
    
#     pointcloud_node = ComposableNode(
#         name='pointcloud_node',
#         package='depth_image_proc',
#         plugin='depth_image_proc::PointCloudXyzNode',
#         remappings=[('image_rect', '/depth/image_rect_flipped'),
#                    ('camera_info', '/depth/camera_info')])

#     # yolov8 path
#     model_file_path  = os.path.join(launch_pkg_dir, 'resources', 'yolov8s_od_gigadataset.v1.onnx')
#     engine_file_path  = os.path.join(launch_pkg_dir, 'resources', 'yolov8s_od_gigadataset.v1.plan')

#     encoder_node = ComposableNode(
#         name='dnn_image_encoder',
#         package='isaac_ros_dnn_image_encoder',
#         plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
#         remappings=[('encoded_tensor', 'tensor_pub')],
#         parameters=[{
#             'input_image_width': 640,
#             'input_image_height': 360,
#             'network_image_width': 640,
#             'network_image_height': 640,
#             'image_mean': [0.0,0.0,0.0],
#             'image_stddev': [1.0,1.0,1.0],
#             'num_blocks': 80,
#         }]
#     )
#     tensor_rt_node = ComposableNode(
#         name='tensor_rt',
#         package='isaac_ros_tensor_rt',
#         plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
#         parameters=[{
#             'model_file_path': '/workspaces/isaac_ros-dev/src/launch_pkg/resources/yolov8s_od_gigadataset.v1.onnx',
#             'engine_file_path': '/workspaces/isaac_ros-dev/src/launch_pkg/resources/yolov8s_od_gigadataset.v1.plan',
#             'output_binding_names': ['output0'],
#             'output_tensor_names': ["output_tensor"],
#             'input_tensor_names': ["input_tensor"],
#             'input_binding_names':['images'] ,
#             'verbose': False,
#             'force_engine_update': False,
#             'enable_fp16': True,
#             'relaxed_dimension_check': False,
#             'num_blocks': 80
#       }]
#     )
#     yolov8_decoder_node = ComposableNode(
#         name='yolov8_decoder_node',
#         package='isaac_ros_yolov8',
#         plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
#         parameters=[{
#             'confidence_threshold': 0.50,
#             'nms_threshold': 0.45,
#             'num_classes' : 5,
#             'in_width': 640.0,
#             'out_width': 640.0,
#             'in_height': 360.0,
#             'out_height': 640.0
#         }]
#     )
#     bbox_extactor = ComposableNode(
#         name='bbox_xyz_node',
#         package='pointcloud_consumer',
#         plugin='pointcloud_consumer::BboxXyzNode',
#         remappings=[('/pointcloud2', '/points'),
#                     ('/input_detections', '/detections_output')]
#     )

#     # depth_node = ComposableNode(
#     #     name='bbox_depth_node',
#     #     package='pointcloud_consumer',
#     #     plugin='pointcloud_consumer::BoundingBoxDepthNode',
#     #     remappings=[('/detections_output', '/detections_output'),
#     #                 ('/camera/depth/image_raw', '/depth/image_rect_raw'),
#     #                 ('/camera/depth/camera_info', '/depth/camera_info')]
#     #             )
    
#     container = ComposableNodeContainer(
#         name='biggest_container',
#         namespace='very_big',
#         package='rclcpp_components',
#         executable='component_container',
#         composable_node_descriptions=[realsense_node,
#         image_flip,
#         pointcloud_node,
#         encoder_node,
#         tensor_rt_node,
#         yolov8_decoder_node,
#         bbox_extactor
#         ],
#         output='screen',
#         arguments=['--ros-args', '--log-level', 'INFO']
#     )
    
#     return LaunchDescription([container])


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import TimerAction

def generate_launch_description():
    launch_pkg_dir = get_package_share_directory('launch_pkg')

    realsense_config_file_path = os.path.join(
        launch_pkg_dir, 'config', 'sensors', 'realsense.yaml'
    )

    realsense_json_file_path = os.path.join(
        launch_pkg_dir, 'config', 'sensors', 'realsense.json'
    )

    # RealSense camera
    realsense_node = ComposableNode(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[
            realsense_config_file_path, 
            {'json_file_path': realsense_json_file_path,
            'publish_tf': False}],
        # No topic remappings — let the driver publish on its default
        # /camera/camera/* topics.
    )

    # Madgwick IMU Filter
    # Takes raw sensor data (accelerometer + gyroscope) from /camera/camera/imu 
    # and calculates the absolute orientation (quaternions), publishing it to /imu/data.
    imu_filter_node = ComposableNode(
        name='imu_filter',
        package='imu_filter_madgwick',
        plugin='ImuFilterMadgwick',
        parameters=[{
            'use_mag': False,                   # The Intel RealSense D455 does not have/use a magnetometer
            'publish_tf': False,                # We rely on the tracker node to publish the TF, not the IMU filter
            'world_frame': 'odom',              # Reference frame for the absolute orientation
        }],
        remappings=[
            ('/imu/data_raw', '/camera/camera/imu')
        ]
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
            'image_stddev': [1.0,1.0,1.0],
            'num_blocks': 80,
            'flip_horizontal': False,
            'keep_aspect_ratio': True
        }]
    )

    # TensorRT inference node
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
            'num_blocks': 80
        }]
    )

    # YOLOv8 decoder
    yolov8_decoder_node = ComposableNode(
        name='yolov8_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[{
            'confidence_threshold': 0.10, # Adjust after initial testing
            'nms_threshold': 0.45,
            'num_classes': 5,
            'in_width': 640.0,
            'in_height': 480.0,
            'out_width': 640.0,
            'out_height': 480.0,
            'tf_frame_id': 'camera_color_optical_frame'
        }],
        remappings=[('detections', '/detections_output')]
    )

    # Armor Tracker Node
    armor_tracker_node = ComposableNode(
        package='armor_tracker',
        plugin='rm_auto_aim::ArmorTrackerNode',
        name='armor_tracker',
        remappings=[
            ('/detector/armors', '/detections_output'),
            ('/camera_info', '/camera/camera/color/camera_info'),
            ('/imu', '/camera/camera/gyro/imu_info'),
        ],
        parameters=[
            {'target_frame': 'odom'},              # TF frame for 3D pose output
            {'target_classes': ['3']},              # YOLO class IDs to track (0=blue,1=grey,2=purple,3=red); grey always excluded
            {'max_armor_distance': 10.0},           # [m] discard PnP detections farther than this
            {'light_ratio': 1.0},                   # min light-bar aspect ratio to accept as armor
            # Tracker
            {'tracker.max_match_distance': 0.30},   # [m] tighter gate to prevent matching wrong armor (was 0.60)
            {'tracker.max_track_range': 6.0},        # [m] drop target if farther than this from camera
            {'tracker.tracking_thres': 4},           # require 4 frames of stability before shooting (was 2)
            {'tracker.lost_time_thres': 0.5},        # [s] die faster to prevent long ghost tracks (was 1.0)
            # EKF process noise
            {'ekf.sigma2_q_xyz': 1.0},              # position process noise variance (was 8.0, lowered to reduce velocity spikes)
            {'ekf.sigma2_q_yaw': 3.0},              # yaw process noise variance (was 10.0, lowered to smooth rotations)
            {'ekf.sigma2_q_r': 1.0},                # radius process noise variance
            # EKF velocity damping (alpha^(dt/T) decay per step; 1.0 = no decay)
            {'ekf.xyz_damping_alpha': 0.85},         # position velocity damping (was 0.95, lowered to brake faster on noisy frames)
            {'ekf.yaw_damping_alpha': 0.85},         # yaw velocity damping
            {'ekf.coast_damping_factor': 0.60},      # extra damping multiplier during TEMP_LOST coasting
            {'ekf.damping_innov_threshold': 0.05},   # [m] pos innovation threshold before damping (was 0.10)
            {'ekf.yaw_innov_threshold': 0.15},       # [rad] yaw innovation above which yaw damping activates
            {'ekf.ref_frequency': 30.0},              # [Hz] reference frame rate for time-normalization
            {'tracker.v_yaw_max': 5.0},              # [rad/s] clamp on yaw spin rate (stops 360-degree glitches)
            # Pose Info Source ('none', 'micro_pose', 'camera_imu')
            # 'none' disables hardware limits/requirements, vital for local software/rendering testing.
            {'pose_source': 'none'},
            # Gimbal TF
            {'gimbal.height': .325},                 # [m] gimbal pivot height above ground
            {'gimbal.yaw_sign': 1.0},                # sign flip for yaw axis (+1 or -1)
            {'gimbal.pitch_sign': 1.0},              # sign flip for pitch axis (+1 or -1)
            # Legacy IMU parameters have been cleared out. Used 'pose_source' paradigm instead to choose sensor fusion mode.
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Trajectory Solver Node
    trajectory_solver_node = ComposableNode(
        package='rm_trajectory',
        plugin='rm_auto_aim::TrajectorySolverNode',
        name='trajectory_solver',
        parameters=[
            {'pose_source': 'camera_imu'},            # 'none' bypasses hardware safety checks, use camera_imu or micro_pose for hardware
            # Ballistics
            {'bullet_speed': 25.0},                 # [m/s] muzzle velocity of the projectile
            {'gravity': 9.8},                       # [m/s²] gravitational acceleration
            {'linear_drag_coeff': 0.01},            # linear drag coefficient (proportional to velocity), higher values cause more drop at longer ranges
            {'quadratic_drag_coeff': 0.01},         # quadratic drag coefficient (proportional to velocity squared), higher values cause more drop at longer ranges and high speeds
            {'use_quadratic_drag': False},          # whether to apply quadratic drag in the ballistics model; if false, only linear drag is applied
            # Latency compensation
            {'time_bias': 0.05},                     # [s] fixed pipeline latency added to flight time
            {'time_bias_alpha': 0.35},               # EMA smoothing factor for adaptive time bias (0–1)
            # Fire gate
            {'min_fire_dist': 0.5},                  # [m] minimum range to allow firing
            {'max_fire_dist': 10.0},                 # [m] maximum range to allow firing
            {'angular_window': 0.09},                # [rad] half-width of face-aligned fire gate (~5°)
            # Acceleration estimator
            {'latency_gate_sigma': 2.5},             # sigma multiplier for timing tolerance in indirect mode
            {'latency_warmup_samples': 5},
            # Indirect mode (fast-spinning targets)
            {'indirect_vyaw_threshold': 3.0},        # [rad/s] spin rate above which indirect aiming activates
            {'indirect_timing_tolerance': 0.02},     # [s] base timing tolerance for alignment windows
            {'indirect_max_candidates': 8},          # max alignment candidates to evaluate per frame
            # Gimbal
            {'gimbal.height': 0.325},                 # [m] gimbal pivot height (shared with tracker)
            {'gimbal.yaw_sign': 1.0},                 # sign flip for yaw axis (shared with tracker)
            {'gimbal.pitch_sign': 1.0},               # sign flip for pitch axis (shared with tracker)
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
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
            armor_tracker_node,
            trajectory_solver_node
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    return LaunchDescription([
        TimerAction(
            period=2.0,
            actions=[container]
        )
    ])