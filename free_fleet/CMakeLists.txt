cmake_minimum_required(VERSION 3.5.0)
project(free_fleet VERSION 0.1.0 LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

find_package(rmf_utils REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

file(GLOB lib_srcs
  "src/*.cpp"
  "src/agv/*.cpp"
  "src/requests/*.cpp"
  "src/utilities/*.cpp"
)

add_library(free_fleet SHARED
  ${lib_srcs}
)

target_link_libraries(free_fleet
  rmf_traffic::rmf_traffic
  rmf_utils::rmf_utils
  Threads::Threads
)

target_include_directories(free_fleet
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Tests
find_package(ament_cmake_catch2 QUIET)
find_package(rmf_cmake_uncrustify QUIET)
if (BUILD_TESTING AND ament_cmake_catch2_FOUND AND rmf_cmake_uncrustify_FOUND)
  file(GLOB_RECURSE unit_test_srcs "test/*.cpp")

  ament_add_catch2(
    test_free_fleet test/main.cpp ${unit_test_srcs}
    TIMEOUT 300)
  target_link_libraries(test_free_fleet
    free_fleet
    rmf_utils::rmf_utils
    rmf_traffic::rmf_traffic
  )
  target_include_directories(test_free_fleet
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )

  find_file(uncrustify_config_file NAMES "share/format/rmf_code_style.cfg")

  rmf_uncrustify(
    ARGN include src test
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )
endif()

# Create cmake config files
include(CMakePackageConfigHelpers)

set(INSTALL_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/free_fleet/cmake")
set(PACKAGE_CONFIG_VERSION_FILE "${CMAKE_CURRENT_BINARY_DIR}/free_fleet-config-version.cmake")
set(PACKAGE_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/free_fleet-config.cmake")

configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cmake/free_fleet-config.cmake.in"
  ${PACKAGE_CONFIG_FILE}
  INSTALL_DESTINATION ${INSTALL_CONFIG_DIR}
)

write_basic_package_version_file(
  ${PACKAGE_CONFIG_VERSION_FILE}
  COMPATIBILITY ExactVersion
)

install(
  TARGETS free_fleet
  EXPORT  free_fleet-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY include/free_fleet
  DESTINATION include
)

install(
  FILES
    ${PACKAGE_CONFIG_VERSION_FILE}
    ${PACKAGE_CONFIG_FILE}
  DESTINATION ${INSTALL_CONFIG_DIR}
)

install(
  EXPORT free_fleet-targets
  FILE free_fleet-targets.cmake
  NAMESPACE free_fleet::
  DESTINATION ${INSTALL_CONFIG_DIR}
)

export(
  EXPORT free_fleet-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/free_fleet-targets.cmake
  NAMESPACE free_fleet::
)

export(PACKAGE free_fleet)
