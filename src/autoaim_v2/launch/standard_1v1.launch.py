import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("autoaim_v2")
    config = os.path.join(pkg, "config", "standard_1v1.yaml")

    # Highest-priority overrides for the values you touch at the field.
    engine_default = os.environ.get(
        "AUTOAIM_ENGINE_PATH",
        "/workspaces/isaac_ros-dev/AI-models/yolov26_keypoints.engine")

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("engine_path", default_value=engine_default),
        DeclareLaunchArgument("shooting_active", default_value="false",
                              description="MASTER fire switch — true for matches"),
        DeclareLaunchArgument("input_mode", default_value="zed_trt"),
        DeclareLaunchArgument("debug", default_value="false"),

        Node(
            package="autoaim_v2",
            executable="aim_node",
            name="aim_v2",
            output="screen",
            parameters=[
                config,
                {
                    "serial_port": LaunchConfiguration("serial_port"),
                    "engine_path": LaunchConfiguration("engine_path"),
                    "shooting_active": ParameterValue(
                        LaunchConfiguration("shooting_active"), value_type=bool),
                    "input_mode": LaunchConfiguration("input_mode"),
                    "debug_enable": ParameterValue(
                        LaunchConfiguration("debug"), value_type=bool),
                },
            ],
        ),
    ])
