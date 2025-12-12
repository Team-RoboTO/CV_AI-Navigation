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

def generate_launch_description():
    launch_pkg_dir = get_package_share_directory('launch_pkg')

    realsense_config_file_path = os.path.join(
        launch_pkg_dir, 'config', 'sensors', 'realsense.yaml'
    )

    # RealSense camera
    realsense_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[realsense_config_file_path],
        remappings=[
            ('/color/image_raw', '/image_origin'),
            ('/color/camera_info', '/camera_info')
        ]
    )

    # Vertical flip on color image
    image_flip_node = ComposableNode(
        name='image_flip',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFlipNode',
        parameters=[{'flip_mode': 'VERTICAL'}],
        remappings=[('/image', '/image_origin'),
                    ('image_flipped', '/image')]
    )

    pointcloud_node = ComposableNode(
    name='pointcloud_node',
    package='depth_image_proc',
    plugin='depth_image_proc::PointCloudXyzNode',
    remappings=[('image_rect', '/depth/image_rect_raw'),  # <-- niente più flipped
                ('camera_info', '/depth/camera_info')]
    )


    # DNN Image Encoder with flip_horizontal = True
    encoder_node = ComposableNode(
        name='dnn_image_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        remappings=[('encoded_tensor', 'tensor_pub')],
        parameters=[{
            'input_image_width': 640,
            'input_image_height': 360,
            'network_image_width': 640,
            'network_image_height': 640,
            'image_mean': [0.0, 0.0, 0.0],
            'image_stddev': [1.0, 1.0, 1.0],
            'num_blocks': 80,
            'flip_horizontal': True  # <--- aggiunto
        }]
    )

    # TensorRT inference node
    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            'model_file_path': os.path.join(launch_pkg_dir, 'resources', 'yolov8s_od_gigadataset.v1.onnx'),
            'engine_file_path': os.path.join(launch_pkg_dir, 'resources', 'yolov8s_od_gigadataset.v1.plan'),
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
            'confidence_threshold': 0.50,
            'nms_threshold': 0.45,
            'num_classes': 5,
            'in_width': 640.0,
            'out_width': 640.0,
            'in_height': 360.0,
            'out_height': 640.0
        }],
        remappings=[('detections', '/detections_output')]
    )

    # 3D bbox extraction
    bbox_extactor_node = ComposableNode(
        name='bbox_xyz_node',
        package='pointcloud_consumer',
        plugin='pointcloud_consumer::BboxXyzNode',
        remappings=[('/pointcloud2', '/points'),
                    ('/input_detections', '/detections_output')]
    )

    # Armor Tracker Node
    armor_tracker_node = ComposableNode(
        package='armor_tracker',
        plugin='rm_auto_aim::ArmorTrackerNode',
        name='armor_tracker',
        parameters=[
            {'max_armor_distance': 10.0},
            {'tracker.max_match_distance': 0.15},
            {'tracker.max_match_yaw_diff': 1.0},
            {'tracker.tracking_thres': 5},
            {'tracker.lost_time_thres': 0.3},
            # EKF parameters
            {'ekf.sigma2_q_xyz': 20.0},
            {'ekf.sigma2_q_yaw': 100.0},
            {'ekf.sigma2_q_r': 800.0}
        ],
        remappings=[
            ('/detector/armors', '/detections_output'),
            ('/camera_info', '/camera_info') 
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Trajectory Solver Node
    trajectory_solver_node = ComposableNode(
        package='rm_trajectory',
        plugin='rm_auto_aim::TrajectorySolverNode',
        name='trajectory_solver',
        parameters=[
            {'bullet_speed': 25.0},
            {'gravity': 9.8},
            {'k': 0.01},
            {'time_bias': 0.05}
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    container = ComposableNodeContainer(
        name='biggest_container',
        namespace='very_big',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            realsense_node,
            image_flip_node,
            pointcloud_node,
            encoder_node,
            tensor_rt_node,
            yolov8_decoder_node,
            bbox_extactor_node,
            armor_tracker_node,
            trajectory_solver_node
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    return LaunchDescription([container])
