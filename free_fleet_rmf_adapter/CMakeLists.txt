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
find_package(rmf_fleet_adapter REQUIRED)

#===============================================================================

add_executable(free_fleet_rmf_adapter
  src/main.cpp
  src/RobotCommand.cpp
  src/RmfFrameTransformer.cpp
  # src/FleetAdapter.cpp
)
target_include_directories(free_fleet_rmf_adapter
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    ${free_fleet_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(free_fleet_rmf_adapter
  PRIVATE
    ${free_fleet_LIBRARIES}
    ${rclcpp_LIBRARIES}

    rmf_fleet_adapter
    Eigen3::Eigen
)

#===============================================================================

install(
  TARGETS free_fleet_rmf_adapter
  RUNTIME DESTINATION lib/free_fleet_rmf_adapter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
