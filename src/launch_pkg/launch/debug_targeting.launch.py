import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("auto_aim")
    launch_pkg_share = get_package_share_directory("launch_pkg")
    default_params = os.path.join(pkg_share, "config", "params_realsense_16.yaml")
    default_engine = os.path.join(
        launch_pkg_share, "resources", "yolov26_keypoints.engine"
    )

    engine_path = LaunchConfiguration("engine_path")
    params_file = LaunchConfiguration("params_file")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    use_fake_micro_imu = LaunchConfiguration("use_fake_micro_imu")
    detector_threshold = LaunchConfiguration("detector_threshold")
    publish_debug_every = LaunchConfiguration("publish_debug_every")
    detector_debug_scores = LaunchConfiguration("detector_debug_scores")

    realsense_node = ComposableNode(
        name="camera",
        namespace="camera",
        package="realsense2_camera",
        plugin="realsense2_camera::RealSenseNodeFactory",
        parameters=[{
            "enable_color": True,
            "enable_depth": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": True,
            "enable_accel": True,
            "unite_imu_method": 2,
            "publish_tf": False,
            "rgb_camera.profile": "640x480x60",
            "pointcloud.enable": False,
            "align_depth.enable": False,
        }],
    )

    realsense_container = ComposableNodeContainer(
        name="realsense_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[realsense_node],
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    yolo26_detector = Node(
        package="auto_aim",
        executable="yolo26_pose_realsense_node.py",
        name="yolo26_pose_realsense",
        parameters=[{
            "engine_path": engine_path,
            "image_topic": image_topic,
            "frame_id": "camera_color_optical_frame",
            "threshold": ParameterValue(detector_threshold, value_type=float),
            "nms_iou": 0.20,
            "keypoint_score_threshold": 0.05,
            "publish_debug_every": ParameterValue(publish_debug_every, value_type=int),
            "debug_scores": ParameterValue(detector_debug_scores, value_type=bool),
            "clip_keypoints_to_image": False,
        }],
        output="screen",
        emulate_tty=True,
    )

    auto_aim_node = Node(
        package="auto_aim",
        executable="auto_aim_node",
        name="auto_aim",
        parameters=[
            params_file,
            {
                # Keypoint topic + thresholds come from params_file; this dict
                # only overrides the per-launch pieces.
                "keypoint_topic": "/detector/armors_keypoints",
                # Debug/calibration keeps both common YOLOv26 color ids live.
                # Tighten this to the enemy color before a match.
                "target_classes": ["0", "2", "3"],
            },
        ],
        remappings=[
            ("/camera_info", camera_info_topic),
        ],
        output="screen",
        emulate_tty=True,
    )

    fake_micro_imu = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub",
            "/micro_imu",
            "std_msgs/msg/Float32MultiArray",
            "{data: [0.0, 0.0]}",
            "-r", "60",
            "--print", "120",
        ],
        name="fake_micro_imu",
        output="screen",
        condition=IfCondition(use_fake_micro_imu),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "engine_path",
            default_value=default_engine,
            description="TensorRT engine installed from launch_pkg/resources/yolov26_keypoints.engine.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Auto-aim parameter YAML for the Jetson 16 RealSense robot.",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/camera/color/image_raw",
            description="RealSense color image topic consumed by the YOLOv26 detector.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera/color/camera_info",
            description="RealSense CameraInfo topic remapped into auto_aim /camera_info.",
        ),
        DeclareLaunchArgument(
            "use_fake_micro_imu",
            default_value="true",
            description="Publish /micro_imu=[0,0] for bench Jetsons without the gimbal microcontroller. Set false on the robot.",
        ),
        DeclareLaunchArgument(
            "detector_threshold",
            default_value="0.20",
            description="YOLOv26 confidence threshold. For bench diagnosis, try 0.10 if max_conf is below 0.20.",
        ),
        DeclareLaunchArgument(
            "publish_debug_every",
            default_value="0",
            description="Publish /yolo/debug_image every N detector frames. 0 disables the image copy.",
        ),
        DeclareLaunchArgument(
            "detector_debug_scores",
            default_value="false",
            description="Log detector score summaries every 60 frames.",
        ),
        realsense_container,
        fake_micro_imu,
        TimerAction(period=1.0, actions=[yolo26_detector]),
        TimerAction(period=2.0, actions=[auto_aim_node]),
    ])
