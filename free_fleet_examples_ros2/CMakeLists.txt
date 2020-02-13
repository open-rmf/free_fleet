cmake_minimum_required(VERSION 3.5.0)

if (NOT $ENV{ROS_VERSION} MATCHES "2")
  return()
endif()

project(free_fleet_examples_ros2)

find_package(ament_cmake REQUIRED)

# -----------------------------------------------------------------------------

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS 
    scripts/send_destination_request.py
    scripts/send_path_request.py
    scripts/send_mode_request.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
