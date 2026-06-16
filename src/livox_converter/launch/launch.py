from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livox_converter',
            executable='converter_node',
            name='livox_custom_to_pc2',
            output='screen',
            parameters=[{
                'input_topic': '/livox/lidar',
                'output_topic': '/livox/lidar_pc2',
            }],
        ),
    ])
