cmake_minimum_required(VERSION 3.5.0)

project(free_fleet_examples)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS 
    scripts/send_destination_request.py
    scripts/send_path_request.py
    scripts/send_mode_request.py
    scripts/send_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

file(GLOB_RECURSE traffic_editor_paths "maps/*.building.yaml")

foreach(path ${traffic_editor_paths})

  # Get the output world name
  string(REPLACE "." ";" list1 ${path})
  list(GET list1 0 name)
  string(REPLACE "/" ";" list2 ${name})
  list(GET list2 -1 world_name)

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})
  set(output_world_path ${output_dir}/${output_world_name}.world)
  set(output_model_dir ${output_dir}/models)

  # first, generate the world
  message("BUILDING WORLDFILE WITH COMMAND: ros2 run building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}")
  if (NO_DOWNLOAD_MODELS)
    add_custom_command(
      DEPENDS ${map_path}
      COMMAND ros2 run building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}
      OUTPUT ${output_world_path}
    )
  else()
    message("DOWNLOADING MODELS WITH COMMAND: ros2 run building_map_tools building_map_model_downloader ${map_path}")
    add_custom_command(
      DEPENDS ${map_path}
      COMMAND ros2 run building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}
      COMMAND ros2 run building_map_tools building_map_model_downloader ${map_path}
      OUTPUT ${output_world_path}
    )
  endif()

  add_custom_target(generate_${output_world_name} ALL
    DEPENDS ${output_world_path}
  )

  install(
    DIRECTORY ${output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )

endforeach()

ament_package()
