# import os
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
# from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
# from launch.conditions import IfCondition, UnlessCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration


def generate_launch_description():


    # launch_pkg_dir = get_package_share_directory('launch_pkg')

    # # RealSense
    # realsense_config_file_path = os.path.join(
    #     launch_pkg_dir,
    #     'config','sensors','realsense.yaml'
    # )

    
    # # Realsense
    # realsense_node = ComposableNode(
    #     package='realsense2_camera',
    #     plugin='realsense2_camera::RealSenseNodeFactory',
    #     parameters=[realsense_config_file_path],
    #     remappings=[('/infra1/image_rect_raw', '/left/image_rect'),
    #                 ('/infra1/camera_info', '/left/camera_info'),
    #                 ('/infra2/image_rect_raw', '/right/image_rect'),
    #                 ('/infra2/camera_info', '/right/camera_info'),
    #                 ('/color/image_raw', '/image'),
    #                 ('input/depth_metadata', '/depth/metadata'),
    #                 ('input/pointcloud', '/depth/color/points'),
    #                 ('input/pointcloud_metadata', '/depth/metadata')
    #                 ]
    # )

    pointcloud_node = ComposableNode(
        name='pointcloud_node',
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyzNode',
        remappings=[('image_rect', '/depth/image_rect_raw'),
                    ('camera_info', '/depth/camera_info'),
                    ('points', '/points_local')]
    )
    

    container = ComposableNodeContainer(
        name='point_cloud_xyz_container',
        namespace='xyz_namespace',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            pointcloud_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    return LaunchDescription([container])
