cmake_minimum_required(VERSION 3.5.0)
project(free_fleet_client_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)

  set(dependencies
    rclcpp
    rclcpp_action
    tf2
    tf2_ros
    nav2_util
    std_srvs
    sensor_msgs
    nav2_msgs
    geometry_msgs
    rmf_fleet_msgs
    free_fleet
  )

  foreach(dep ${dependencies})
    find_package(${dep} REQUIRED)
  endforeach()

  #=============================================================================

  include_directories(include)
  add_executable(free_fleet_client_ros2
    src/main.cpp
    src/utilities.cpp
    src/client_node.cpp
    src/client_node_config.cpp
  )
  ament_target_dependencies(free_fleet_client_ros2
    ${dependencies}
  )

  #=============================================================================

  set(testing_targets
    fake_action_server
    fake_docking_server
  )
  foreach(target ${testing_targets})
    add_executable(${target}
      src/tests/${target}.cpp
    )
    ament_target_dependencies(${target}
      ${dependencies}
    )
  endforeach()

  #=============================================================================

  install(TARGETS free_fleet_client_ros2
    ${testing_targets}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  ament_package()

else()
  message("ament_cmake not found so skipping this ROS2 package")
endif()
