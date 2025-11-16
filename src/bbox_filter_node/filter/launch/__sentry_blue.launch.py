from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='filter',
            executable='bbox_shoot',
            parameters=[{
                'color_to_shoot': 'blue_armor'
            }]
        ),
        #Node(
        #    package='filter',
        #    executable='shoot_flip',
        #    parameters=[{
        #        'camera_x_displacement': + 0.085,
        #        'camera_z_displacement': - 0.04,
        #        'camera_y_displacement': + 0.015
        #    }]
        #),
        Node(
            package='filter',
            executable='imu_filter',
            parameters=[{
                        'accelerometer_noise_density' :0.001082195019761597, 
                        'accelerometer_random_walk': 0.00010095570453457349 ,
                        'gyroscope_noise_density': 0.0001960643375142761,
                        'gyroscope_random_walk': 1.3215206616748797e-05 ,
                        'sampling_frequency': 200.0
            }]
        ),
        # Node(
        #     package='filter',
        #     executable='depth',
        #     remappings=[('/camera/depth/image_raw', '/aligned_depth_to_color/image_raw'),
        #                 ('/camera/depth/camera_info', '/aligned_depth_to_color/camera_info'),
        #                 ('/detections', '/detections_output')]
        # ),
        # Node(
        #     package = 'tf2_ros',
        #     executable = 'static_transform_publisher',
        #     arguments = ['0', '0', '0', '1.5707963267948966', '3.141592653589793', '1.5707963267948966', 'camera_link', 'TF' ]
        # ),
        # Node(
        #     package='depth_image_proc',
        #     executable='point_cloud_xyz_node',
        #     name='pointcloud_node',
        #     remappings=[
        #         ('image_rect', '/depth/image_rect_raw'),
        #         ('camera_info', '/depth/camera_info'),source install/setup.bash && ros2 launch filter __sentry_blue.launch.py
        #         ('points', 'points_local')
        #     ]
        # )
    ])
