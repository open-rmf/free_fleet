cmake_minimum_required(VERSION 2.8.3)
project(free_fleet_client)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(CycloneDDS REQUIRED PATHS /opt/ros/dashing)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  /opt/ros/dashing/include
)

link_directories(/opt/ros/dashing/lib)

idlc_generate(free_fleet_messages_lib FreeFleet.idl)

add_executable(free_fleet_client free_fleet_client.cpp)
target_link_libraries(
  free_fleet_client
  free_fleet_messages_lib
  ddsc
  ${catkin_LIBRARIES})

install(TARGETS free_fleet_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
