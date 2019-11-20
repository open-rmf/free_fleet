cmake_minimum_required(VERSION 3.5.0)
project(free_fleet_utilities)

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

# -----------------------------------------------------------------------------
# For including DDS implementations

include(ExternalProject)
set(CYCLONE_INSTALL_PATH ${CMAKE_BINARY_DIR}/cyclonedds)
ExternalProject_Add(cyclonedds
  GIT_REPOSITORY https://github.com/eclipse-cyclonedds/cyclonedds.git
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CYCLONE_INSTALL_PATH} -DBUILD_IDLC=NO -DBUILD_SHARED_LIBS=FALSE
)
# -----------------------------------------------------------------------------

add_library(free_fleet_utilities
  src/Publisher.cpp
)
target_link_libraries(free_fleet_utilities
  ddsc
  crypto
  ssl
)
target_include_directories(free_fleet_utilities
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CYCLONE_INSTALL_PATH}/include
)

# -----------------------------------------------------------------------------

install(
  TARGETS free_fleet_utilities
  EXPORT  free_fleet_utilities
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
