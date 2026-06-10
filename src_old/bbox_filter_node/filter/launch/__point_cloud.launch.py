from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='depth_image_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXYZ',
                name='point_cloud_xyz',
                remappings=[
                    ('image_rect', '/camera/depth/image_rect'),  # Input depth image topic
                    ('camera_info', '/camera/depth/camera_info'),  # Camera info topic
                    ('points', '/camera/depth/points')  # Output point cloud topic
                ],
                parameters=[
                    {'queue_size': 5},  # Queue size for messages
                ],
                extra_arguments=[{'use_intra_process_comms': True}]  # Intra-process communication
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])
