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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

########################
## Generate messages  ##
########################

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

# Serial bridge: /turret/cmd + /cmd_vel_NAV -> micro serial.
add_executable(serial_bridge src/serial_bridge.cpp)
ament_target_dependencies(serial_bridge
  rclcpp
  geometry_msgs
  std_msgs
)

# Turret yaw mux:
# /cmd_vel_AI + /detector/armors + /turret/idle_target + TF -> /turret/cmd.
add_executable(turret_yaw_mux src/turret_yaw_mux.cpp)
ament_target_dependencies(turret_yaw_mux
  rclcpp
  geometry_msgs
  std_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  serial_bridge
  turret_yaw_mux
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# NOTE: the TensorRT engine is intentionally NOT installed with the package.
# It lives outside the workspace. Point to it with the engine_path launch arg
# or the AUTOAIM_ENGINE_PATH env var.

install(PROGRAMS
  viewer_node.py
  serial_bridge.py
  zed_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
