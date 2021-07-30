cmake_minimum_required(VERSION 3.5.0)
project(free_fleet_cyclonedds VERSION 0.1.0 LANGUAGES C CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

find_package(rmf_utils REQUIRED)
find_package(free_fleet REQUIRED)
find_package(CycloneDDS REQUIRED COMPONENTS idlc)

idlc_generate(MiddlewareMessages_lib "idl/MiddlewareMessages.idl")

add_library(free_fleet_cyclonedds SHARED
  "src/messages/convert.cpp"
)
target_link_libraries(free_fleet_cyclonedds
  PRIVATE
    MiddlewareMessages_lib
    CycloneDDS::ddsc
    free_fleet::free_fleet
    rmf_utils::rmf_utils
)

# # Tests
# find_package(ament_cmake_catch2 QUIET)
# find_package(rmf_cmake_uncrustify QUIET)
# if (BUILD_TESTING AND ament_cmake_catch2_FOUND AND rmf_cmake_uncrustify_FOUND)
#   file(GLOB_RECURSE unit_test_srcs "test/*.cpp")
# 
#   ament_add_catch2(
#     test_free_fleet_cyclonedds test/main.cpp ${unit_test_srcs}
#     TIMEOUT 300)
#   target_link_libraries(test_free_fleet_cyclonedds
#     free_fleet_cyclonedds
#     rmf_utils::rmf_utils
#     free_fleet::free_fleet
#     CycloneDDS::ddsc
#   )
#   target_include_directories(test_free_fleet_cyclonedds
#     PRIVATE
#       $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
#   )
# 
#   find_file(uncrustify_config_file NAMES "share/format/rmf_code_style.cfg")
# 
#   rmf_uncrustify(
#     ARGN include src test
#     CONFIG_FILE ${uncrustify_config_file}
#     MAX_LINE_LENGTH 80
#   )
# endif()

# Create cmake config files
include(CMakePackageConfigHelpers)

set(INSTALL_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/free_fleet_cyclonedds/cmake")
set(PACKAGE_CONFIG_VERSION_FILE "${CMAKE_CURRENT_BINARY_DIR}/free_fleet_cyclonedds-config-version.cmake")
set(PACKAGE_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/free_fleet_cyclonedds-config.cmake")

configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cmake/free_fleet_cyclonedds-config.cmake.in"
  ${PACKAGE_CONFIG_FILE}
  INSTALL_DESTINATION ${INSTALL_CONFIG_DIR}
)

write_basic_package_version_file(
  ${PACKAGE_CONFIG_VERSION_FILE}
  COMPATIBILITY ExactVersion
)

install(
  TARGETS free_fleet_cyclonedds
  EXPORT  free_fleet_cyclonedds-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY include/free_fleet_cyclonedds
  DESTINATION include
)

install(
  FILES
    ${PACKAGE_CONFIG_VERSION_FILE}
    ${PACKAGE_CONFIG_FILE}
  DESTINATION ${INSTALL_CONFIG_DIR}
)

install(
  EXPORT free_fleet_cyclonedds-targets
  FILE free_fleet_cyclonedds-targets.cmake
  NAMESPACE free_fleet_cyclonedds::
  DESTINATION ${INSTALL_CONFIG_DIR}
)

export(
  EXPORT free_fleet_cyclonedds-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/free_fleet_cyclonedds-targets.cmake
  NAMESPACE free_fleet_cyclonedds::
)

export(PACKAGE free_fleet_cyclonedds)
