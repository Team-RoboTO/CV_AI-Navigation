import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_pkg_dir = get_package_share_directory('launch_pkg')
    debug_targeting_launch = os.path.join(
        launch_pkg_dir, 'launch', 'debug_targeting.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(debug_targeting_launch)
        )
    ])
