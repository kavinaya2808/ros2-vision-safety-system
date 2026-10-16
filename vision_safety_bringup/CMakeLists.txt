cmake_minimum_required(VERSION 3.8)
project(vision_safety_bringup)

find_package(ament_cmake REQUIRED)

# Launch files, parameter files and the RViz config that tie the system together.
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# ros2 run vision_safety_bringup robustness_study.py
install(PROGRAMS scripts/robustness_study.py DESTINATION lib/${PROJECT_NAME})

# Fast unit tests of the robustness study's scenario logic (no ROS, no Gazebo)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_robustness_logic test/test_robustness_logic.py)
endif()

# ----- end-to-end scenario tests in headless Gazebo ---------------------------
# Slow (~1-2 min) and needs GPU rendering. Skip with:
#   colcon build --cmake-args -DVISION_SAFETY_SIM_TESTS=OFF
option(VISION_SAFETY_SIM_TESTS "Build the Gazebo scenario tests" ON)
if(BUILD_TESTING AND VISION_SAFETY_SIM_TESTS)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_sim_scenarios.py TIMEOUT 300)
endif()

ament_package()
