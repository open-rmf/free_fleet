cmake_minimum_required(VERSION 3.5.0)
project(free_fleet_utils VERSION 0.0.1 LANGUAGES CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------

add_library(${PROJECT_NAME} STATIC
  src/FreeFleet.c
  src/common.cpp
)

# -----------------------------------------------------------------------------

find_package(cyclonedds_cmake_module QUIET CONFIG)
find_package(CycloneDDS QUIET CONFIG)

if (cyclonedds_cmake_module_FOUND AND CycloneDDS_FOUND)

  # TODO ROS2 builds
  # target_link_libraries(${PROJECT_NAME}
  #   CycloneDDS::ddsc
  # )
  return()

endif()

# -----------------------------------------------------------------------------

# Getting CycloneDDS as an external project for ROS 1
include(ExternalProject)
set(CYCLONE_INSTALL_PATH ${CMAKE_BINARY_DIR}/cyclonedds)
ExternalProject_Add(cyclonedds
  GIT_REPOSITORY https://github.com/eclipse-cyclonedds/cyclonedds.git
  CMAKE_ARGS 
    -DCMAKE_INSTALL_PREFIX=${CYCLONE_INSTALL_PATH} 
    -DBUILD_IDLC=NO 
    -DBUILD_SHARED_LIBS=FALSE
)
install(
  DIRECTORY ${CYCLONE_INSTALL_PATH}/include/dds
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.hxx"
  PATTERN ".svn" EXCLUDE
)
link_directories(${CYCLONE_INSTALL_PATH}/lib)

add_dependencies(${PROJECT_NAME} 
  cyclonedds
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CYCLONE_INSTALL_PATH}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME}
  ddsc
)

# -----------------------------------------------------------------------------

# Mark executables and/or libraries for installation
list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})
install(
  TARGETS ${PACKAGE_LIBRARIES}
  EXPORT ${PROJECT_NAME}-targets 
  DESTINATION lib
)
install(
  EXPORT ${PROJECT_NAME}-targets 
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.hxx"
  PATTERN ".svn" EXCLUDE
)
install(
  FILES package.xml 
  DESTINATION share/${PROJECT_NAME}
)

# Create cmake config files
# See doc: https://cmake.org/cmake/help/latest/module/CMakePackageConfigHelpers.html#example-generating-package-files
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

export(
  EXPORT ${PROJECT_NAME}-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake
)
