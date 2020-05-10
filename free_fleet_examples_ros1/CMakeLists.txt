cmake_minimum_required(VERSION 3.5.0)

project(free_fleet_examples_ros1)

# -----------------------------------------------------------------------------

find_package(catkin QUIET)

if (catkin_FOUND)

  catkin_package()

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

  install(
    DIRECTORY test_maps
    DESTINATION share/${PROJECT_NAME}
  )

else()
  message("catkin not found so skipping this ROS1 package")
endif()

