cmake_minimum_required(VERSION 3.5.0)
project(free_fleet VERSION 0.0.1 LANGUAGES C CXX)

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

find_package(ament_cmake REQUIRED)
find_package(CycloneDDS QUIET CONFIG)
if(NOT CycloneDDS_FOUND)
  message(WARNING "Could not find Eclipse Cyclone DDS - skipping '${PROJECT_NAME}'")
  ament_package()
  return()
endif()

file(GLOB lib_srcs 
  "src/*.cpp"
  "src/configs/*.cpp"
  "src/dds_utils/*.cpp"
  "src/messages/*.c"
  "src/messages/*.cpp"
)

add_library(free_fleet SHARED
  ${lib_srcs}
)

target_link_libraries(free_fleet
  CycloneDDS::ddsc
  ssl
  crypto
)

target_include_directories(free_fleet
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_export_interfaces(free_fleet HAS_LIBRARY_TARGET)
ament_export_dependencies(CycloneDDS)

install(
  DIRECTORY include/free_fleet
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS free_fleet
  EXPORT  free_fleet
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
