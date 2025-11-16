from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #  Node(
        #      package='filter',
        #      executable='talker_debug',
        #      remappings=[
        #          ('filter_publisher', 'fp_debug'),
        #          ('/tracking_pose', 'tracking_pose_debug'),
        #          ('/optimal_bbox', 'optimal_bbox_debug')
        #      ]
        #  ),
         Node(
             package='filter',
             executable='listener_debug',
             remappings=[
                 ('/yolov8_processed_image', 'yolov8_processed_image_debug'),
             ]
         ),
        #  Node(
        #      package='filter',
        #      executable='shoot_debug',
        #      remappings=[
        #         ('optimal_bbox', 'optimal_bbox_debug'),
        #         ('predicted_shoot', 'predicted_shoot_debug')
        #      ]
        #  ),
        Node(
            package='depth_image_proc',
            executable='point_cloud_xyz_node',
            name='pointcloud_node_debug',
            remappings=[
                ('image_rect', '/depth/image_rect_raw'),
                ('camera_info', '/depth/camera_info'),
                ('points', 'points_debug'),
            ]
        )
    ])