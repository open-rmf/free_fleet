cmake_minimum_required(VERSION 3.0.0)
project(free_fleet_client)

include(ExternalProject)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

set(CYCLONE_INSTALL_PATH ${CMAKE_BINARY_DIR}/cyclonedds)

ExternalProject_Add(cyclonedds
  GIT_REPOSITORY https://github.com/eclipse-cyclonedds/cyclonedds.git
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CYCLONE_INSTALL_PATH} -DBUILD_IDLC=NO -DBUILD_SHARED_LIBS=FALSE
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CYCLONE_INSTALL_PATH}/include
)

link_directories(${CYCLONE_INSTALL_PATH}/lib)

add_executable(free_fleet_client free_fleet_client.cpp FreeFleet.c)
target_link_libraries(
  free_fleet_client
  ddsc
  crypto
  ssl
  ${catkin_LIBRARIES})

add_executable(test_client
  src/main.cpp
  src/Client.cpp
  src/FreeFleet.c)
target_link_libraries(test_client
  ddsc
  crypto
  ssl
  ${catkin_LIBRARIES})

install(TARGETS free_fleet_client test_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
