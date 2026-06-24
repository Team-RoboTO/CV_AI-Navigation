import os
from glob import glob
from setuptools import setup

package_name = 'nav2_new'

setup(
    name=package_name,
    version='1.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roboto Team',
    maintainer_email='team@roboto.dev',
    description='RoboMaster navigation with Livox Mid-360, Nav2, waypoint strategy, turret mux & game state FSM',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Core runtime


            'turret_yaw_mux       = nav2_new.turret_yaw_mux:main',
            'cmd_vel_safety       = nav2_new.cmd_vel_safety:main',
            'cmd_vel_xy_only      = nav2_new.cmd_vel_xy_only:main',
            # Game state
            'micro_status_parser  = nav2_new.micro_status_parser:main',


            'health_monitor       = nav2_new.health_monitor:main',
            # Tools
            'waypoint_editor      = nav2_new.waypoint_editor:main',
            'save_map             = nav2_new.save_map:main',
            'tf_frame_relay       = nav2_new.tf_frame_relay:main',
            'livox_custom_filter  = nav2_new.livox_custom_filter:main',
            'save_pcd_once        = nav2_new.save_pcd_once:main',
            'pcd_to_2d_map        = nav2_new.pcd_to_2d_map:main',
            # Test / simulation
            'fake_cv_publisher    = nav2_new.fake_cv_publisher:main',
            'fake_micro_status    = nav2_new.fake_micro_status:main',
            'scan_memory_filter   = nav2_new.scan_memory_filter:main',
            # New entries, the cv handle the commands to the micro, cv command only the idle pitch and yaw to keep the barrel pointed to the center of the arena
            'micro_status_adapter = nav2_new.micro_status_adapter:main',
            'set_initial_pose = nav2_new.set_initial_pose:main',
            'waypoint_manager = nav2_new.waypoint_manager:main',
            'game_state_manager = nav2_new.game_state_manager:main',
            'turret_idle_target_publisher = nav2_new.turret_idle_target_publisher:main',
            'game_status_reporter = nav2_new.game_status_reporter:main',
            'nav_match_reset = nav2_new.nav_match_reset:main',
            'amcl_scan_stabilizer = nav2_new.amcl_scan_stabilizer:main',
            'amcl_motion_gate     = nav2_new.amcl_motion_gate:main',
        ],
    },
)
