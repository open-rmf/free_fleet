cmake_minimum_required(VERSION 3.5.0)

project(free_fleet_examples_ros1)

# -----------------------------------------------------------------------------

find_package(catkin REQUIRED)

catkin_package()

# -----------------------------------------------------------------------------

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY test_maps
  DESTINATION share/${PROJECT_NAME}
)
