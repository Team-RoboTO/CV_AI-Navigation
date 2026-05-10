cmake_minimum_required(VERSION 3.10)
project(auto_aim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# dependencies

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

# generate the gimbal command message
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalCmd.msg"
  "msg/AutoAimDebug.msg"
  "msg/ArmorKeypoint.msg"
  "msg/ArmorKeypointArray.msg"
  DEPENDENCIES std_msgs
)

# build

add_library(${PROJECT_NAME}_component SHARED
  src/pnp_solver.cpp
  src/tracker.cpp
  src/config_validator.cpp
  src/debug_publisher.cpp
  src/detection_adapter.cpp
  src/fire_gate.cpp
  src/frame_transformer.cpp
  src/auto_aim_node.cpp
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

# link generated message typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_component ${cpp_typesupport_target})

# register the composable node
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "auto_aim::AutoAimNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# install

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/yolo26_pose_realsense_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/docs")
  install(DIRECTORY docs/
    DESTINATION share/${PROJECT_NAME}/docs
  )
endif()

ament_package()
