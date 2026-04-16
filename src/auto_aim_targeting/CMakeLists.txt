cmake_minimum_required(VERSION 3.10)
project(auto_aim_targeting)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-Wall -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/planning/shot_planner.cpp
  src/planning/ballistics_solver.cpp
  src/io/gimbal_command_controller.cpp
  src/io/debug_publisher.cpp
  src/planning/engagement_planner.cpp
  src/tracking/ekf.cpp
  src/planning/fire_gate.cpp
  src/planning/latency_compensator.cpp
  src/measurement/detection_converter.cpp
  src/io/output_publisher.cpp
  src/measurement/pnp_solver.cpp
  src/io/gimbal_pose_adapter.cpp
  src/tracking/tracker_manager.cpp
  src/tracking/tracker_runtime_factory.cpp
  src/auto_aim_node.cpp
  src/tracking/tracker.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  auto_aim_interfaces
  vision_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::AutoAimNode
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_engagement_planner
    test/planning/test_engagement_planner.cpp)
  target_link_libraries(test_engagement_planner ${PROJECT_NAME})
  ament_target_dependencies(test_engagement_planner rclcpp auto_aim_interfaces)

  ament_add_gtest(test_latency_compensator
    test/planning/test_latency_compensator.cpp)
  target_link_libraries(test_latency_compensator ${PROJECT_NAME})
  ament_target_dependencies(test_latency_compensator rclcpp)

  ament_add_gtest(test_fire_gate
    test/planning/test_fire_gate.cpp)
  target_link_libraries(test_fire_gate ${PROJECT_NAME})
  ament_target_dependencies(test_fire_gate auto_aim_interfaces)

  ament_add_gtest(test_tracker_runtime_factory
    test/tracking/test_runtime_factory.cpp)
  target_link_libraries(test_tracker_runtime_factory ${PROJECT_NAME})
  ament_target_dependencies(test_tracker_runtime_factory rclcpp auto_aim_interfaces)

  ament_add_gtest(test_gimbal_command_controller
    test/io/test_gimbal_command_controller.cpp)
  target_link_libraries(test_gimbal_command_controller ${PROJECT_NAME})
  ament_target_dependencies(
    test_gimbal_command_controller
    std_msgs
    geometry_msgs
    visualization_msgs
    auto_aim_interfaces
  )

  ament_add_gtest(test_gimbal_pose_adapter
    test/io/test_gimbal_pose_adapter.cpp)
  target_link_libraries(test_gimbal_pose_adapter ${PROJECT_NAME})
  ament_target_dependencies(
    test_gimbal_pose_adapter
    rclcpp
    geometry_msgs
    tf2
    tf2_ros
  )

  ament_add_gtest(test_tracker_runtime_manager
    test/tracking/test_runtime_manager.cpp)
  target_link_libraries(test_tracker_runtime_manager ${PROJECT_NAME})
  ament_target_dependencies(
    test_tracker_runtime_manager
    rclcpp
    tf2
    tf2_geometry_msgs
    geometry_msgs
    auto_aim_interfaces
  )

  ament_add_gtest(test_include_layout
    test/pipeline/test_include_layout.cpp)
  target_link_libraries(test_include_layout ${PROJECT_NAME})
  target_compile_definitions(
    test_include_layout
    PRIVATE AUTO_AIM_TARGETING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_package()
