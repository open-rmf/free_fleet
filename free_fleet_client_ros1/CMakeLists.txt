cmake_minimum_required(VERSION 3.5.0)

project(free_fleet_client_ros1 LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include(GNUInstallDirs)

#===============================================================================

find_package(catkin QUIET COMPONENTS
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  actionlib
  move_base_msgs
)

if (catkin_FOUND)

  find_package(free_fleet REQUIRED)

  catkin_package()

  #=============================================================================

  add_executable(free_fleet_client_ros1
    src/main.cpp
    src/utilities.cpp
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

  #=============================================================================

  set(testing_targets
    fake_action_server
  )
  
  foreach(target ${testing_targets})
    add_executable(${target}
      src/tests/${target}.cpp
    )
    target_link_libraries(${target}
      ${catkin_LIBRARIES}
    )
    target_include_directories(${target}
      PRIVATE
        ${catkin_INCLUDE_DIRS}
    )
  endforeach()
  
  #=============================================================================

  install(
    TARGETS
      free_fleet_client_ros1
      ${testing_targets}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

else()
  message("catkin not found so skipping this ROS1 package")
endif()
