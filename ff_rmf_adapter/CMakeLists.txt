cmake_minimum_required(VERSION 3.5.0)

project(ff_rmf_adapter)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(free_fleet REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)

#===============================================================================

add_executable(ff_rmf_adapter
  src/main.cpp
  src/RobotCommand.cpp
  # src/FleetAdapter.cpp
)
target_link_libraries(ff_rmf_adapter
  ${free_fleet_LIBRARIES}
  Eigen3::Eigen
)
target_include_directories(ff_rmf_adapter
  PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_fleet_adapter_INCLUDE_DIRS}
    ${free_fleet_INCLUDE_DIRS}
)

#===============================================================================

install(
  TARGETS 
    ff_rmf_adapter
  RUNTIME DESTINATION lib/ff_rmf_adapter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
