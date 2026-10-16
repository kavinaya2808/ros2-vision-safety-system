cmake_minimum_required(VERSION 3.8)
project(vision_safety_sim)

find_package(ament_cmake REQUIRED)

# Pure resource package: robot description, worlds, bridge config, launch files.
install(
  DIRECTORY urdf worlds config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
