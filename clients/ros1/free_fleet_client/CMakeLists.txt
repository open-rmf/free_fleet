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
    ${catkin_INCLUDE_DIRS}
    ${CYCLONE_INSTALL_PATH}/include
)

# -----------------------------------------------------------------------------

add_executable(test_sub_server
  src/tests/test_sub_server.cpp
  src/free_fleet/FreeFleet.c
)
target_link_libraries(test_sub_server
  ddsc
  crypto
  ssl
  ${catkin_LIBRARIES}
)
target_include_directories(test_sub_server
  PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CYCLONE_INSTALL_PATH}/include
)

# -----------------------------------------------------------------------------

add_executable(test_action_server
  src/tests/test_action_server.cpp
)
target_link_libraries(test_action_server
  ${catkin_LIBRARIES}
)
target_include_directories(test_action_server
  PRIVATE
    ${catkin_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

add_executable(test_pub_location_command
  src/tests/test_pub_location_command.cpp
  src/free_fleet/FreeFleet.c
)
target_link_libraries(test_pub_location_command
  ddsc
  crypto
  ssl
  ${catkin_LIBRARIES}
)
target_include_directories(test_pub_location_command
  PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CYCLONE_INSTALL_PATH}/include
)

# -----------------------------------------------------------------------------

add_executable(sub
  src/tests/subscriber.cpp
  src/free_fleet/FreeFleet.c
)
target_link_libraries(sub
  ddsc
  crypto
  ssl
  ${catkin_LIBRARIES}
)
target_include_directories(sub
  PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CYCLONE_INSTALL_PATH}/include
)

# -----------------------------------------------------------------------------

add_executable(pub
  src/tests/publisher.cpp
  src/free_fleet/FreeFleet.c
)
target_link_libraries(pub
  ddsc
  crypto
  ssl
  ${catkin_LIBRARIES}
)
target_include_directories(pub
  PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CYCLONE_INSTALL_PATH}/include
)

# -----------------------------------------------------------------------------

install(TARGETS free_fleet_client test_sub_server test_action_server sub pub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
