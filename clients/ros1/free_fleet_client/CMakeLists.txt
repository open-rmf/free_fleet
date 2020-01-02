cmake_minimum_required(VERSION 3.0.0)

if (NOT $ENV{ROS_VERSION} MATCHES "1")
  return()
endif()

project(free_fleet_client)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  actionlib
  move_base_msgs
)

find_package(free_fleet_utils REQUIRED)

catkin_package()

# -----------------------------------------------------------------------------

add_executable(free_fleet_client
  src/main.cpp
  src/Client.cpp
  src/ClientConfig.cpp
)
target_link_libraries(free_fleet_client
  ${free_fleet_utils_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_include_directories(free_fleet_client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${free_fleet_utils_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

# set(testing_targets
#   fake_action_server
#   test_dds_sub_state
#   test_dds_pub_mode_request
#   test_dds_pub_path_request
#   test_dds_pub_sim_path_request
#   test_dds_pub_destination_request
# )

# foreach(target ${testing_targets})
#   add_executable(${target}
#     src/tests/${target}.cpp
#     src/dds_utils/common.cpp
#     src/free_fleet/FreeFleet.c
#   )
#   add_dependencies(${target} cyclonedds)
#   target_link_libraries(${target}
#     ddsc
#     crypto
#     ssl
#     ${catkin_LIBRARIES}
#   )
#   target_include_directories(${target}
#     PRIVATE
#       ${catkin_INCLUDE_DIRS}
#       ${CYCLONE_INSTALL_PATH}/include
#   )
# endforeach()

# -----------------------------------------------------------------------------

install(
  DIRECTORY
    launch
  DESTINATION share/free_fleet_client
)

install(
  TARGETS 
    free_fleet_client 
    # ${testing_targets}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
