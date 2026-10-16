cmake_minimum_required(VERSION 3.8)
project(vision_safety_core)

# Pure C++17 library: perception + safety decision logic.
# Deliberately NO rclcpp dependency, so it is unit-testable without ROS running.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(${PROJECT_NAME} SHARED
  src/clock_watchdog.cpp
  src/color_detector.cpp
  src/danger_zone.cpp
  src/debug_overlay.cpp
  src/safety_monitor.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC opencv_core opencv_imgproc)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Downstream packages: find_package(vision_safety_core) then
# target_link_libraries(my_node vision_safety_core::vision_safety_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_safety_monitor test/test_safety_monitor.cpp)
  target_link_libraries(test_safety_monitor ${PROJECT_NAME})

  ament_add_gtest(test_clock_watchdog test/test_clock_watchdog.cpp)
  target_link_libraries(test_clock_watchdog ${PROJECT_NAME})

  ament_add_gtest(test_color_detector test/test_color_detector.cpp)
  target_link_libraries(test_color_detector ${PROJECT_NAME})

  ament_add_gtest(test_danger_zone test/test_danger_zone.cpp)
  target_link_libraries(test_danger_zone ${PROJECT_NAME})

  ament_add_gtest(test_debug_overlay test/test_debug_overlay.cpp)
  target_link_libraries(test_debug_overlay ${PROJECT_NAME})

  # End-to-end: synthetic 3D scene -> image -> detector -> danger zone
  ament_add_gtest(test_pipeline test/test_pipeline.cpp)
  target_include_directories(test_pipeline PRIVATE test)
  target_link_libraries(test_pipeline ${PROJECT_NAME})
endif()

ament_package()
