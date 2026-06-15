cmake_minimum_required(VERSION 3.10)
project(autoaim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ── RealSense detector path dependencies (librealsense2 + CUDA + TensorRT) ──
# The RealSense detector is a high-performance C++ node that drives the camera
# with librealsense2 directly and runs the same TensorRT engine as the ZED path.
find_package(realsense2 REQUIRED)
find_package(CUDAToolkit REQUIRED)

# TensorRT ships no CMake config on Jetson; locate the headers/libs by hand.
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include/aarch64-linux-gnu /usr/include)
find_library(TENSORRT_NVINFER nvinfer
  HINTS /usr/lib/aarch64-linux-gnu)
find_library(TENSORRT_NVINFER_PLUGIN nvinfer_plugin
  HINTS /usr/lib/aarch64-linux-gnu)
if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_NVINFER OR NOT TENSORRT_NVINFER_PLUGIN)
  message(FATAL_ERROR "TensorRT not found (NvInfer.h / libnvinfer / libnvinfer_plugin)")
endif()

# Build the CUDA preprocess kernel for the device GPU. Override on the command
# line with -DCMAKE_CUDA_ARCHITECTURES=<sm> for other Jetsons.
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 72 87)  # Xavier (72), Orin (87)
endif()
enable_language(CUDA)

# Generate messages
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalCmd.msg"
  "msg/ArmorKeypoint.msg"
  "msg/ArmorKeypointArray.msg"
  DEPENDENCIES std_msgs
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}_component SHARED
  src/pnp_solver.cpp
  src/tracker.cpp
  src/autoaim_node.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  angles
)

target_link_libraries(${PROJECT_NAME}_component ${OpenCV_LIBS})

# Link generated message typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_component ${cpp_typesupport_target})

# Register composable node
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "autoaim::AutoAimNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# Serial bridge. This is the only microcontroller transport implementation.
add_executable(serial_bridge src/serial_bridge.cpp)
ament_target_dependencies(serial_bridge
  rclcpp
  geometry_msgs
  std_msgs
)
install(TARGETS serial_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# ── RealSense + TensorRT detector (competition camera path) ──
# CUDA kernel built as an isolated static lib so nvcc never sees ROS compile
# flags; the node itself is plain C++ and only links the CUDA runtime.
add_library(realsense_preprocess STATIC src/realsense_preprocess.cu)
target_include_directories(realsense_preprocess PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CUDAToolkit_INCLUDE_DIRS}
)
set_target_properties(realsense_preprocess PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(realsense_detector src/realsense_detector.cpp)
ament_target_dependencies(realsense_detector
  rclcpp
  sensor_msgs
  vision_msgs
  std_msgs
)
target_include_directories(realsense_detector PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${TENSORRT_INCLUDE_DIR}
  ${CUDAToolkit_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(realsense_detector
  realsense_preprocess
  realsense2::realsense2
  ${TENSORRT_NVINFER}
  ${TENSORRT_NVINFER_PLUGIN}
  CUDA::cudart
  ${OpenCV_LIBS}
  ${cpp_typesupport_target}  # autoaim generated messages (ArmorKeypoint[Array])
)
install(TARGETS realsense_detector
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Sensor configs (zed.yaml / realsense.yaml) consumed by the launch files.
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# NOTE: the TensorRT engine is intentionally NOT installed with the package.
# It lives outside the workspace (default:
# /workspaces/isaac_ros-dev/AI-models/yolov26_keypoints.engine) so colcon does
# not copy a large binary on every build. Point to it with the engine_path
# launch arg or the AUTOAIM_ENGINE_PATH env var.

install(PROGRAMS
  viewer_node.py
  zed_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
