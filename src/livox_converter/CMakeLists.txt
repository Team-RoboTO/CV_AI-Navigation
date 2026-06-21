cmake_minimum_required(VERSION 3.8)
project(livox_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(livox_to_scan_node src/livox_to_scan_node.cpp)
ament_target_dependencies(livox_to_scan_node
  rclcpp
  sensor_msgs
  std_msgs
  livox_ros_driver2
)

install(TARGETS livox_to_scan_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
