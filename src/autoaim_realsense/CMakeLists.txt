cmake_minimum_required(VERSION 3.10)
project(autoaim_realsense)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(autoaim REQUIRED)

# RealSense detector path dependencies (librealsense2 + CUDA + TensorRT).
# Keep them optional at package-configure time so machines that do not carry a
# RealSense stack can still run a plain `colcon build` over the whole workspace.
find_package(realsense2 QUIET)
find_package(CUDAToolkit QUIET)

# TensorRT ships no CMake config on Jetson; locate the headers/libs by hand.
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include/aarch64-linux-gnu /usr/include)
find_library(TENSORRT_NVINFER nvinfer
  HINTS /usr/lib/aarch64-linux-gnu)
find_library(TENSORRT_NVINFER_PLUGIN nvinfer_plugin
  HINTS /usr/lib/aarch64-linux-gnu)
if(realsense2_FOUND AND CUDAToolkit_FOUND
   AND TENSORRT_INCLUDE_DIR AND TENSORRT_NVINFER AND TENSORRT_NVINFER_PLUGIN)
  # Build the CUDA preprocess kernel for the device GPU. Override on the command
  # line with -DCMAKE_CUDA_ARCHITECTURES=<sm> for other Jetsons.
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 72 87)  # Xavier (72), Orin (87)
  endif()
  enable_language(CUDA)

  # CUDA kernel built as an isolated static lib so nvcc never sees ROS compile
  # flags; the node itself is plain C++ and only links the CUDA runtime.
  add_library(realsense_preprocess STATIC src/realsense_preprocess.cu)
  target_include_directories(realsense_preprocess PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CUDAToolkit_INCLUDE_DIRS}
  )
  set_target_properties(realsense_preprocess PROPERTIES POSITION_INDEPENDENT_CODE ON)

  add_executable(realsense_detector src/realsense_detector.cpp)
  ament_target_dependencies(realsense_detector
    rclcpp
    sensor_msgs
    vision_msgs
    std_msgs
    autoaim
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
  )

  install(TARGETS realsense_detector
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "Skipping realsense_detector: librealsense2, CUDA Toolkit, or TensorRT "
    "was not found. Build this package on a RealSense-equipped target to "
    "install the detector executable.")
endif()

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
