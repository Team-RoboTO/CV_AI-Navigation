from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # Parametri configurabili
    camera_serial = LaunchConfiguration('camera_serial', default='138322252714')
    usb_port = LaunchConfiguration('usb_port', default='2-1.2')
    device_type = LaunchConfiguration('device_type', default='Intel RealSense D455')
    dummy_rx = LaunchConfiguration('dummy_rx', default='false')
    dev_id = LaunchConfiguration('dev_id', default='0')

    # Container composable node principale
    container = ComposableNodeContainer(
        name='very_big_biggest_container',
        namespace='very_big',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],
        output='screen',
    )

    # Nodi composabili
    realsense_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='camera',
        parameters=[{
            'serial_no': camera_serial,
            'usb_port_id': usb_port,
            'device_type': device_type,
            'publish_tf': True,
            'tf_publish_rate': 20.0,
            'reset_device': True,
        }]
    )

    depth_proc_node = ComposableNode(
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyzNode',
        name='pointcloud_node',
    )

    dnn_image_encoder_node = ComposableNode(
        package='isaac_ros_nitros',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        name='dnn_image_encoder',
        parameters=[{
            'dummy_rx': dummy_rx.perform(None),
            'dev_id': dev_id.perform(None),
        }]
    )

    tensorrt_node = ComposableNode(
        package='isaac_ros_nitros',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        name='tensor_rt',
    )

    yolov8_decoder_node = ComposableNode(
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        name='yolov8_decoder_node',
    )

    bbox_xyz_node = ComposableNode(
        package='pointcloud_consumer',
        plugin='pointcloud_consumer::BboxXyzNode',
        name='bbox_xyz_node',
    )

    # Caricamento nodi nel container
    load_nodes = LoadComposableNodes(
        target_container=container.name,
        composable_node_descriptions=[
            realsense_node,
            depth_proc_node,
            dnn_image_encoder_node,
            tensorrt_node,
            yolov8_decoder_node,
            bbox_xyz_node,
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_serial', default_value='138322252714', description='Serial number of the RealSense camera'),
        DeclareLaunchArgument('usb_port', default_value='2-1.2', description='USB port of the RealSense camera'),
        DeclareLaunchArgument('device_type', default_value='Intel RealSense D455', description='Type of RealSense device'),
        DeclareLaunchArgument('dummy_rx', default_value='false', description='Dummy RX parameter for Nitros'),
        DeclareLaunchArgument('dev_id', default_value='0', description='Device ID parameter for Nitros'),

        container,
        load_nodes,
    ])
