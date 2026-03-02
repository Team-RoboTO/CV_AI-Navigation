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
        remappings=[
            ('color/image_raw', '/image'),      
            ('color/camera_info', '/camera_info'),
            ('depth/image_rect_raw', '/depth_image'),
            ('depth/camera_info', '/depth_info')
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
            'confidence_threshold': 0.10, # da cambiare dopo test
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
            ('/camera_info', '/camera/camera/color/camera_info')
        ],
        parameters=[
            {'target_frame': 'odom'},              # TF frame for 3D pose output
            {'target_classes': ['3']},              # YOLO class IDs to track (0=blue,1=grey,2=purple,3=red); grey always excluded
            {'max_armor_distance': 10.0},           # [m] discard PnP detections farther than this
            {'light_ratio': 1.0},                   # min light-bar aspect ratio to accept as armor
            # Tracker
            {'tracker.max_match_distance': 0.60},   # [m] max 3D distance between EKF prediction and detection to match
            {'tracker.max_track_range': 6.0},        # [m] drop target if farther than this from camera
            {'tracker.tracking_thres': 2},           # consecutive matches needed to transition DETECTING → TRACKING
            {'tracker.lost_time_thres': 1.0},        # [s] time without match before TEMP_LOST → LOST
            # EKF process noise
            {'ekf.sigma2_q_xyz': 2.0},              # position process noise variance (higher → trusts measurements more)
            {'ekf.sigma2_q_yaw': 5.0},              # yaw process noise variance
            {'ekf.sigma2_q_r': 0.5},                # radius process noise variance
            # EKF velocity damping (alpha^(dt/T) decay per step; 1.0 = no decay)
            {'ekf.xyz_damping_alpha': 0.95},         # position velocity damping (lower → stronger braking)
            {'ekf.yaw_damping_alpha': 0.95},         # yaw velocity damping
            {'ekf.coast_damping_factor': 0.85},      # extra damping multiplier during TEMP_LOST coasting
            {'ekf.damping_innov_threshold': 0.10},   # [m] position innovation above which overshoot damping activates
            {'ekf.yaw_innov_threshold': 0.15},       # [rad] yaw innovation above which yaw damping activates
            # Gimbal TF
            {'gimbal.height': .325},                 # [m] gimbal pivot height above ground
            {'gimbal.yaw_sign': 1.0},                # sign flip for yaw axis (+1 or -1)
            {'gimbal.pitch_sign': 1.0},              # sign flip for pitch axis (+1 or -1)
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Trajectory Solver Node
    trajectory_solver_node = ComposableNode(
        package='rm_trajectory',
        plugin='rm_auto_aim::TrajectorySolverNode',
        name='trajectory_solver',
        parameters=[
            # Ballistics
            {'bullet_speed': 15.0},                 # [m/s] muzzle velocity of the projectile
            {'gravity': 9.8},                        # [m/s²] gravitational acceleration
            {'k': 0.01},                             # air drag coefficient (higher → more drop compensation)
            # Latency compensation
            {'time_bias': 0.05},                     # [s] fixed pipeline latency added to flight time
            {'time_bias_alpha': 0.35},               # EMA smoothing factor for adaptive time bias (0–1)
            # Fire gate
            {'min_fire_dist': 0.5},                  # [m] minimum range to allow firing
            {'max_fire_dist': 10.0},                 # [m] maximum range to allow firing
            {'angular_window': 0.09},                # [rad] half-width of face-aligned fire gate (~5°)
            # Acceleration estimator
            {'accel_ema_alpha': 0.3},                # EMA smoothing for target acceleration estimate (0–1)
            {'max_accel': 6.0},                      # [m/s²] clamp on estimated acceleration
            {'latency_gate_sigma': 2.5},             # sigma multiplier for timing tolerance in indirect mode
            # Indirect mode (fast-spinning targets)
            {'indirect_vyaw_threshold': 3.0},        # [rad/s] spin rate above which indirect aiming activates
            {'indirect_timing_tolerance': 0.02},     # [s] base timing tolerance for alignment windows
            {'indirect_max_candidates': 8},          # max alignment candidates to evaluate per frame
            # Gimbal
            {'gimbal_height': 0.325},                # [m] gimbal pivot height (must match tracker's gimbal.height)
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