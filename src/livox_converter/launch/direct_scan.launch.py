from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livox_converter',
            executable='livox_to_scan_node',
            name='livox_to_scan',
            output='screen',
            parameters=[{
                'input_topic': '/livox/lidar',
                'output_topic': '/scan',
                'frame_id': 'base_scan',
                'publish_rate_hz': 5.0,
                'publish_every_n': 1,
                'keep_every_n': 4,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.020,
                'scan_time': 0.20,
                'range_min': 0.20,
                'range_max': 4.0,
                'z_min': -0.55,
                'z_max': -0.15,
                'drop_zero_points': True,
                'valid_tag': -1,
                'reflectivity_min': 0.0,
                'max_raw_points': 0,
                'max_used_points': 0,
                'qos_depth': 1,
                'input_reliability': 'best_effort',
                'output_reliability': 'reliable',
                'log_stats_period_sec': 5.0,
            }],
        )
    ])
