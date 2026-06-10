import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    # Argomenti per i percorsi dei modelli (da modificare con i tuoi path reali)
    engine_file_arg = DeclareLaunchArgument(
        'engine_file_path',
        default_value='/workspaces/isaac_ros-dev/src/launch_pkg/resources/yolov8_op.plan', # [cite: 78]
        description='Percorso assoluto al file .plan generato da trtexec, conversione da onnx a trtexec sempre e solo da jetson per ottimizzazione GPU Orin'
    )
    
    # 1. Configurazione Nodo Realsense (ISAAC ROS Driver)
    # Sostituisce realsense2_camera. Usa NvBufSurface per zero-copy [cite: 36, 19]
    realsense_node = ComposableNode(
        package='isaac_ros_realsense',
        plugin='nvidia::isaac_ros::realsense::RealSenseNode',
        name='realsense_node',
        namespace='',
        parameters=[{
            'rgb_width': 640,
            'rgb_height': 480,     # Risoluzione nativa richiesta
            'rgb_framerate': 60,   # Consigliato 60Hz per target veloci [cite: 42]
            'enable_depth': False, # Disabilita depth se non serve per risparmiare banda USB
            'align_depth': False,  # EVITARE align_depth su driver (usa troppa CPU) 
            'enable_infra1': False,
            'enable_infra2': False
        }],
        remappings=[
            ('image_raw', 'camera/image_raw'),
            ('camera_info', 'camera/camera_info')
        ]
    )

    # 2. Nodo Encoder (Bridge Vision -> AI)
    # Converte l'immagine in tensore normalizzato su GPU 
    encoder_node = ComposableNode(
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        name='dnn_image_encoder',
        namespace='',
        parameters=[{
            'input_image_width': 640,
            'input_image_height': 480,
            'network_image_width': 640,
            'network_image_height': 480,
            'tensor_output_topic': 'tensor_pub',
            'enable_padding': False # Non serve se input == network size
        }],
        remappings=[
            ('encoded_tensor', 'tensor_pub'),
            ('image', 'camera/image_raw') # Input dalla camera
        ]
    )

    # 3. Nodo TensorRT (Inferenza Zero-Copy)
    # Esegue il modello YOLOv8 compilato .plan [cite: 70, 71]
    tensorrt_node = ComposableNode(
        package='isaac_ros_tensorrt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        name='tensorrt_node',
        namespace='',
        parameters=[{
            'engine_file_path': LaunchConfiguration('engine_file_path'),
            'output_binding_names': ['output0'], # Nome standard output YOLOv8
            'output_tensor_names': ['output_tensor'],
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['images'],
            'force_engine_update': False
        }],
        remappings=[
            ('tensor_pub', 'tensor_pub'), # Input dall'encoder
            ('tensor_sub', 'output_tensor') # Output verso il decoder
        ]
    )

    # 4. Nodo Decoder YOLOv8
    # Esegue NMS e decodifica i box (accelerato, non in Python puro) [cite: 86, 88]
    yolov8_decoder_node = ComposableNode(
        package='isaac_ros_yolov8', # Assicurati di avere questo pacchetto o equivalente
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        name='yolov8_decoder',
        namespace='',
        parameters=[{
            'confidence_threshold': 0.5,
            'nms_threshold': 0.45,
            'num_classes': 4 
        }],
        remappings=[
            ('tensor_sub', 'output_tensor'),
            ('detections_output', 'detections') # Output finale: Detection2DArray
        ]
    )

    # Container Unico per NITROS (Zero-Copy) [cite: 109, 110]
    container = ComposableNodeContainer(
        name='isaac_ros_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt', # Multithreaded container
        composable_node_descriptions=[
            realsense_node,
            encoder_node,
            tensorrt_node,
            yolov8_decoder_node
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        engine_file_arg,
        container
    ])
