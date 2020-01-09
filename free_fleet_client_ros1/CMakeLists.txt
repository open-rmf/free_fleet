cmake_minimum_required(VERSION 3.5.0)

if (NOT $ENV{ROS_VERSION} MATCHES "1")
  return()
endif()

project(free_fleet_client_ros1 LANGUAGES CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

# -----------------------------------------------------------------------------

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  actionlib
  move_base_msgs
)

find_package(free_fleet REQUIRED)
find_package(CycloneDDS REQUIRED)

catkin_package()

# -----------------------------------------------------------------------------

add_executable(free_fleet_client_ros1
  src/main.cpp
  src/ClientNode.cpp
  src/ClientNodeConfig.cpp
)
target_link_libraries(free_fleet_client_ros1
  ${free_fleet_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_include_directories(free_fleet_client_ros1
  PRIVATE
    ${free_fleet_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

install(
  DIRECTORY launch
  DESTINATION share/free_fleet_client_ros1
)

install(
  TARGETS
    free_fleet_client_ros1
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
