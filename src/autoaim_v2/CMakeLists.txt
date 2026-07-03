cmake_minimum_required(VERSION 3.16)
project(autoaim_v2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d imgproc)
find_package(autoaim REQUIRED)   # ArmorKeypointArray for ros_topics input mode

# ── Optional Jetson-only deps: CUDA + TensorRT + camera SDKs ────────────────
# Auto-detected. Without them the node still builds with the ros_topics input
# mode only (A/B behind the old Python detector) and all tests still run.
find_package(CUDA QUIET)
find_package(ZED 4 QUIET)      # SDK >= 4 required (CUDA primary context)
if(NOT ZED_FOUND)
  find_package(ZED QUIET)      # SDK 5 config rejects "4" — accept and check
  if(ZED_FOUND AND ZED_VERSION_MAJOR LESS 4)
    set(ZED_FOUND FALSE)
  endif()
endif()
find_package(realsense2 QUIET)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/include)
find_library(TENSORRT_LIB nvinfer
  HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

set(HAVE_CUDA_TRT FALSE)
if(CUDA_FOUND AND TENSORRT_INCLUDE_DIR AND TENSORRT_LIB)
  set(HAVE_CUDA_TRT TRUE)
  enable_language(CUDA)
endif()

set(HAVE_ZED_TRT FALSE)
if(HAVE_CUDA_TRT AND ZED_FOUND)
  set(HAVE_ZED_TRT TRUE)
  message(STATUS "autoaim_v2: ZED + TensorRT found (zed_trt mode available)")
endif()

set(HAVE_RS_TRT FALSE)
if(HAVE_CUDA_TRT AND realsense2_FOUND)
  set(HAVE_RS_TRT TRUE)
  message(STATUS "autoaim_v2: RealSense + TensorRT found (rs_trt mode available)")
endif()

if(NOT HAVE_ZED_TRT AND NOT HAVE_RS_TRT)
  if(HAVE_CUDA_TRT)
    message(STATUS "autoaim_v2: CUDA/TRT found but no camera SDK -> ros_topics mode only")
  else()
    message(STATUS "autoaim_v2: no GPU deps found -> ros_topics input mode only")
  endif()
endif()

# ── Core library: pure math + serial, no ROS, unit-testable anywhere ────────
add_library(aim_core STATIC
  src/ballistics.cpp
  src/solver.cpp
  src/tracker.cpp
  src/aimer.cpp
  src/serial_port.cpp
)
target_include_directories(aim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(aim_core ${OpenCV_LIBS})

# ── Node ────────────────────────────────────────────────────────────────────
add_executable(aim_node src/aim_node.cpp)
target_link_libraries(aim_node aim_core)
ament_target_dependencies(aim_node rclcpp std_msgs sensor_msgs geometry_msgs
  visualization_msgs autoaim)

if(HAVE_CUDA_TRT)
  # Shared GPU library: TRT engine + preprocess kernels (always needed for
  # both zed_trt and rs_trt modes).
  set(AIM_GPU_SOURCES src/detector_trt.cpp src/preprocess.cu)
  set(AIM_GPU_INCLUDES
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CUDA_INCLUDE_DIRS} ${TENSORRT_INCLUDE_DIR})
  set(AIM_GPU_LIBS ${CUDA_CUDART_LIBRARY} ${CUDA_CUDA_LIBRARY}
    ${TENSORRT_LIB} ${OpenCV_LIBS})

  if(HAVE_ZED_TRT)
    list(APPEND AIM_GPU_SOURCES src/zed_camera.cpp)
    list(APPEND AIM_GPU_INCLUDES ${ZED_INCLUDE_DIRS})
    list(APPEND AIM_GPU_LIBS ${ZED_LIBRARIES})
    target_compile_definitions(aim_node PRIVATE AIM_WITH_ZED_TRT=1)
  endif()

  if(HAVE_RS_TRT)
    list(APPEND AIM_GPU_SOURCES src/rs_camera.cpp)
    target_compile_definitions(aim_node PRIVATE AIM_WITH_RS_TRT=1)
  endif()

  add_library(aim_gpu STATIC ${AIM_GPU_SOURCES})
  target_include_directories(aim_gpu PUBLIC ${AIM_GPU_INCLUDES})
  target_link_libraries(aim_gpu ${AIM_GPU_LIBS})
  if(HAVE_RS_TRT)
    target_link_libraries(aim_gpu realsense2::realsense2)
  endif()
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 72 87)  # Xavier (72), Orin (87)
  endif()
  set_target_properties(aim_gpu PROPERTIES
    CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
  target_link_libraries(aim_node aim_gpu)
endif()

install(TARGETS aim_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ── Tests ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t ballistics solver tracker aimer protocol gimbal_buffer)
    ament_add_gtest(test_${t} test/test_${t}.cpp)
    target_link_libraries(test_${t} aim_core)
  endforeach()
endif()

ament_package()
