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
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(cyclonedds_cmake_module REQUIRED)
find_package(CycloneDDS QUIET CONFIG)
if(NOT CycloneDDS_FOUND)
  message(WARNING "Could not find Eclipse Cyclone DDS - skipping '${PROJECT_NAME}'")
  ament_package()
  return()
endif()

# -----------------------------------------------------------------------------

add_executable(free_fleet_server 
  src/main.cpp
  src/Server.cpp
  src/ServerConfig.cpp
  src/math/math.cpp
  src/dds_utils/common.cpp
  src/free_fleet/FreeFleet.c
)
ament_target_dependencies(free_fleet_server
  rclcpp
  rmf_fleet_msgs
)
target_link_libraries(free_fleet_server
  CycloneDDS::ddsc
  Eigen3::Eigen
)

# -----------------------------------------------------------------------------

install(
  DIRECTORY
    launch
  DESTINATION share/free_fleet_server
)

install(
  TARGETS free_fleet_server
  EXPORT
  RUNTIME DESTINATION lib/free_fleet_server
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
