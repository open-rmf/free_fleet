cmake_minimum_required(VERSION 3.5.0)

project(free_fleet_server LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(free_fleet REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB bin_srcs 
  "src/*.cpp"
)

add_executable(free_fleet_server
  ${bin_srcs}
)

target_link_libraries(free_fleet_server
  free_fleet::free_fleet
  Eigen3::Eigen
  ${rclcpp_LIBRARIES}
  ${rmf_fleet_msgs_LIBRARIES}
)

target_include_directories(free_fleet_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
)

install(
  TARGETS free_fleet_server
  RUNTIME DESTINATION lib/free_fleet_server
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
