"""
localization_fusion.launch.py  (OPT-IN, experimental)

Brings up the wheel-odometry + FAST-LIO EKF fusion layer:
  - chassis_odom_publisher   : /micro_status -> /odom_wheel (vx,vy in base_link)
  - tf_frame_relay           : re-run with publish_tf:=false, publish_odom:=true
                               so it stops publishing odom->base_link on /tf and
                               instead feeds the corrected FAST-LIO pose to /odom_lio
  - ekf_filter_node          : fuses /odom_lio + /odom_wheel -> odom->base_link (TF)

IMPORTANT — do NOT also start the normal tf_frame_relay from sensors.launch.py
when you use this, or two nodes will fight over odom->base_link. Two ways:
  (A) Quick test: launch sensors.launch.py as usual, then `ros2 lifecycle`/kill
      the original tf_frame_relay, then run this launch. Messy.
  (B) Clean: in sensors.launch.py set the original tf_frame_relay parameter
      publish_tf:=false (or comment that node out) and add THIS launch to your
      bringup. Recommended once validated.

Rollback: just don't include this launch and keep tf_frame_relay publish_tf:=true.
Your current pipeline is then byte-for-byte unchanged.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_new')
    default_ekf = os.path.join(pkg_share, 'config', 'ekf.yaml')

    ekf_params = LaunchConfiguration('ekf_params')
    micro_topic = LaunchConfiguration('micro_status_topic')

    return LaunchDescription([
        DeclareLaunchArgument('ekf_params', default_value=default_ekf),
        DeclareLaunchArgument('micro_status_topic', default_value='/micro_status'),

        # 1) wheel velocity -> /odom_wheel (base_link frame)
        Node(
            package='nav2_new',
            executable='chassis_odom_publisher',
            name='chassis_odom_publisher',
            output='screen',
            parameters=[{
                'micro_status_topic': micro_topic,
                'output_topic': '/odom_wheel',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'head_yaw_index': 0,
                'vx_index': 2,
                'vy_index': 3,
                'head_yaw_sign': 1.0,   # flip to -1.0 if motion is mirrored (see README)
                'vx_sign': 1.0,
                'vy_sign': -1.0,
                'deadband_mps': 0.10,
                'vxy_variance': 0.05,
            }],
        ),

        # 2) FAST-LIO corrected pose -> /odom_lio, and DO NOT publish TF here
        Node(
            package='nav2_new',
            executable='tf_frame_relay',
            name='tf_frame_relay',
            output='screen',
            parameters=[{
                'input_topic': '/tf_fastlio',
                'source_parent_frame': 'camera_init',
                'source_child_frame': 'body',
                'target_parent_frame': 'odom',
                'target_child_frame': 'base_link',
                # IMPORTANT: keep these identical to your sensors.launch.py values
                # for body_to_base_q* and the invert_* flags, or the pose frame
                # will not match. Fill them in if you override them there.
                'planarize': True,
                'force_z_zero': True,
                # Lever-arm: set enable True and fill Lx,Ly after calibrating
                # (see README "LIVELLO 1.5"). Anchors base_link on the yaw axis
                # so head rotation does not translate the frame.
                'lever_arm_enable': False,
                'lever_arm_x': 0.0,
                'lever_arm_y': 0.0,
                'lever_arm_z': 0.0,
                'publish_tf': False,        # EKF owns the TF now
                'publish_odom': True,       # feed FAST-LIO pose to the EKF
                'odom_output_topic': '/odom_lio',
                'odom_xy_variance': 0.02,
                'odom_yaw_variance': 0.02,
                # keep the stationary gate as before
                'use_micro_stationary_gate': True,
                'micro_status_topic': micro_topic,
                'micro_vx_index': 2,
                'micro_vy_index': 3,
                'micro_vx_sign': 1.0,
                'micro_vy_sign': -1.0,
                'stationary_vxy_threshold': 0.12,
            }],
        ),

        # 3) EKF: /odom_lio + /odom_wheel -> odom->base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
        ),
    ])
