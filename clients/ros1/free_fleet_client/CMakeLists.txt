cmake_minimum_required(VERSION 3.0.0)
project(free_fleet_client)

# -----------------------------------------------------------------------------

include(ExternalProject)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  free_fleet_msgs
  actionlib
  move_base_msgs
)

set(CYCLONE_INSTALL_PATH ${CMAKE_BINARY_DIR}/cyclonedds)

ExternalProject_Add(cyclonedds
  GIT_REPOSITORY https://github.com/eclipse-cyclonedds/cyclonedds.git
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CYCLONE_INSTALL_PATH} -DBUILD_IDLC=NO -DBUILD_SHARED_LIBS=FALSE
)

catkin_package()

link_directories(${CYCLONE_INSTALL_PATH}/lib)

# -----------------------------------------------------------------------------

add_executable(free_fleet_client
  src/main.cpp
  src/Client.cpp
  src/free_fleet/FreeFleet.c
)
target_link_libraries(free_fleet_client
  ddsc
  crypto
  ssl
  ${catkin_LIBRARIES}
)
target_include_directories(free_fleet_client
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
    ${catkin_INCLUDE_DIRS}
    ${CYCLONE_INSTALL_PATH}/include
)

# -----------------------------------------------------------------------------

set(testing_targets
  fake_action_server
  test_dds_sub_state
  test_dds_pub_mode_command
  test_dds_pub_path_command
  test_dds_pub_sim_path_command
  test_dds_pub_location_command
)

foreach(target ${testing_targets})
  add_executable(${target}
    src/tests/${target}.cpp
    src/free_fleet/FreeFleet.c
  )
  target_link_libraries(${target}
    ddsc
    crypto
    ssl
    ${catkin_LIBRARIES}
  )
  target_include_directories(${target}
    PRIVATE
      ${catkin_INCLUDE_DIRS}
      ${CYCLONE_INSTALL_PATH}/include
  )
endforeach()

# -----------------------------------------------------------------------------

install(
  TARGETS 
    free_fleet_client 
    ${testing_targets}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
