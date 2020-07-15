cmake_minimum_required(VERSION 3.5.0)

project(free_fleet_rmf_adapter)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(free_fleet REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)

#===============================================================================

add_executable(free_fleet_rmf_adapter
  src/main.cpp
  src/utilities.cpp
  src/Parameters.cpp
  src/RobotCommand.cpp
  src/RmfFrameTransformer.cpp
)
target_include_directories(free_fleet_rmf_adapter
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_task_msgs_INCLUDE_DIRS}
)
target_link_libraries(free_fleet_rmf_adapter
  PRIVATE
    ${rclcpp_LIBRARIES}
    ${rmf_task_msgs_LIBRARIES}
    Eigen3::Eigen
    free_fleet::free_fleet
    rmf_traffic::rmf_traffic
    rmf_traffic_ros2::rmf_traffic_ros2
    rmf_fleet_adapter::rmf_fleet_adapter
)

#===============================================================================

install(
  TARGETS free_fleet_rmf_adapter
  RUNTIME DESTINATION lib/free_fleet_rmf_adapter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
