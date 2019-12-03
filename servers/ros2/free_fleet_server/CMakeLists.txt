cmake_minimum_required(VERSION 3.5)

project(free_fleet_server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(cyclonedds_cmake_module REQUIRED)
find_package(rmw_cyclonedds_cpp REQUIRED)

# -----------------------------------------------------------------------------

add_executable(free_fleet_server 
  free_fleet_server.cpp
)
ament_target_dependencies(free_fleet_server
  rmw_cyclonedds_cpp
)

# -----------------------------------------------------------------------------

install(TARGETS free_fleet_server
  RUNTIME DESTINATION bin
)

ament_package()
