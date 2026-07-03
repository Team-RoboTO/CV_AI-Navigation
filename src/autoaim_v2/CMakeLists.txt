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

# ── Optional Jetson-only deps: ZED SDK + CUDA + TensorRT ────────────────────
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
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/include)
find_library(TENSORRT_LIB nvinfer
  HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

set(HAVE_ZED_TRT FALSE)
if(CUDA_FOUND AND ZED_FOUND AND TENSORRT_INCLUDE_DIR AND TENSORRT_LIB)
  set(HAVE_ZED_TRT TRUE)
  enable_language(CUDA)
  message(STATUS "autoaim_v2: building WITH ZED + TensorRT (competition binary)")
else()
  message(STATUS "autoaim_v2: ZED/TensorRT not found -> ros_topics input mode only")
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

if(HAVE_ZED_TRT)
  add_library(aim_gpu STATIC
    src/detector_trt.cpp
    src/zed_camera.cpp
    src/preprocess.cu
  )
  target_include_directories(aim_gpu PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(aim_gpu
    ${ZED_LIBRARIES} ${CUDA_CUDART_LIBRARY} ${CUDA_CUDA_LIBRARY}
    ${TENSORRT_LIB} ${OpenCV_LIBS})
  set_target_properties(aim_gpu PROPERTIES CUDA_ARCHITECTURES "87")  # AGX Orin
  target_compile_definitions(aim_node PRIVATE AIM_WITH_ZED_TRT=1)
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
