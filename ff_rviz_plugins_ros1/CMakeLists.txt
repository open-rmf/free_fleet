cmake_minimum_required(VERSION 3.5.0)

project(ff_rviz_plugins_ros1)

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
  geometry_msgs
  pluginlib
  rviz
)

if (catkin_FOUND)

  set(CMAKE_AUTOMOC ON)

  find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  set(QT_VERSION ${Qt5Widgets_VERSION})

  find_package(free_fleet REQUIRED)

  catkin_package()
  
  #=============================================================================

  add_library(${PROJECT_NAME}
    src/ff_nav_goal_panel.cpp
    src/utilities.cpp
  )
  target_link_libraries(${PROJECT_NAME}
    ${free_fleet_LIBRARIES}
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
  )
  target_include_directories(${PROJECT_NAME}
    PRIVATE
      ${free_fleet_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
  )

  #=============================================================================

  install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(FILES
    plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

else()
  message("catkin not found so skipping this ROS1 package")
endif()
