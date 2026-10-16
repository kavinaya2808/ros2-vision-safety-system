cmake_minimum_required(VERSION 3.8)
project(vision_safety_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(vision_safety_interfaces REQUIRED)
find_package(vision_safety_core REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# ----- obstacle_detector ------------------------------------------------------
# Built as a component (loadable into a container for zero-copy intra-process
# transport) AND as a standalone executable.
add_library(obstacle_detector_component SHARED src/obstacle_detector_node.cpp)
ament_target_dependencies(obstacle_detector_component
  rclcpp rclcpp_components sensor_msgs geometry_msgs cv_bridge tf2 tf2_ros
  vision_safety_interfaces)
target_link_libraries(obstacle_detector_component
  vision_safety_core::vision_safety_core opencv_core opencv_imgproc)

rclcpp_components_register_node(obstacle_detector_component
  PLUGIN "vision_safety_ros::ObstacleDetectorNode"
  EXECUTABLE obstacle_detector)

# ----- safety_monitor ---------------------------------------------------------
add_library(safety_monitor_component SHARED src/safety_monitor_node.cpp)
ament_target_dependencies(safety_monitor_component
  rclcpp rclcpp_components geometry_msgs diagnostic_updater diagnostic_msgs
  vision_safety_interfaces)
target_link_libraries(safety_monitor_component vision_safety_core::vision_safety_core)

rclcpp_components_register_node(safety_monitor_component
  PLUGIN "vision_safety_ros::SafetyMonitorNode"
  EXECUTABLE safety_monitor)

# ----- cruise_driver ----------------------------------------------------------
add_library(cruise_driver_component SHARED src/cruise_driver_node.cpp)
ament_target_dependencies(cruise_driver_component rclcpp rclcpp_components geometry_msgs)

rclcpp_components_register_node(cruise_driver_component
  PLUGIN "vision_safety_ros::CruiseDriverNode"
  EXECUTABLE cruise_driver)

install(TARGETS
  obstacle_detector_component
  safety_monitor_component
  cruise_driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ----- node-level integration tests (real nodes, fake inputs, no Gazebo) -------
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_safety_monitor_node.py TIMEOUT 180)
  add_launch_test(test/test_obstacle_detector_node.py TIMEOUT 180)
endif()

ament_package()
