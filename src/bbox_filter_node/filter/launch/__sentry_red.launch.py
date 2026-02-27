from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='filter',
            executable='talker',
            parameters=[{
                'color_to_shoot': 'red_armor'
            }]
        ),
        Node(
            package='filter',
            executable='imu_filter',
            parameters=[{
                'accelerometer_noise_density': 0.001082195019761597,
                'accelerometer_random_walk': 0.00010095570453457349,
                'gyroscope_noise_density': 0.0001960643375142761,
                'gyroscope_random_walk': 1.3215206616748797e-05,
                'sampling_frequency': 200.0
            }]
        ),
    ])
