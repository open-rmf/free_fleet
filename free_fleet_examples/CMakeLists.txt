cmake_minimum_required(VERSION 3.5)

project(free_fleet_examples)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

file(GLOB_RECURSE traffic_editor_paths "maps/*.building.yaml")

foreach(path ${traffic_editor_paths})

  # Get the output world name
  string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${path})
  string(REGEX MATCH "[^\/]+$" world_name  ${no_extension_path})

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})

  ##############################################################################
  # Generate the nav graphs
  ##############################################################################

  set(output_nav_graphs_dir ${output_dir}/nav_graphs/)
  set(output_nav_graphs_phony ${output_nav_graphs_dir}/phony)
  add_custom_command(
    OUTPUT ${output_nav_graphs_phony}
    COMMAND ros2 run rmf_building_map_tools building_map_generator nav ${map_path} ${output_nav_graphs_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name}_nav_graphs ALL
    DEPENDS ${output_nav_graphs_phony}
  )

  install(
    DIRECTORY ${output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )

endforeach()

install(PROGRAMS
  ${PROJECT_NAME}/nav1_get_tf.py
  ${PROJECT_NAME}/nav1_move_base_cancel.py
  ${PROJECT_NAME}/nav1_move_base_simple_goal.py
  ${PROJECT_NAME}/nav2_cancel_all_goals.py
  ${PROJECT_NAME}/nav2_get_tf.py
  ${PROJECT_NAME}/nav2_send_navigate_to_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    tests/test_copyright.py
    tests/test_flake8.py
    tests/test_pep257.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()

  add_test(NAME e2e_tests COMMAND python3 -m unittest WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests/e2e)
endif()

ament_package()
