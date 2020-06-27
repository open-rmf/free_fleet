cmake_minimum_required(VERSION 3.5.0)

project(free_fleet_examples)

find_package(ament_cmake QUIET)

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
