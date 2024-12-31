cmake_minimum_required(VERSION 3.5)

project(free_fleet_adapter)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/fleet_adapter.py
  ${PROJECT_NAME}/nav1_robot_adapter.py
  ${PROJECT_NAME}/nav2_robot_adapter.py
  ${PROJECT_NAME}/robot_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
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

  if (NAV2_INTEGRATION_TESTING)
    set(_pytest_tests
      tests/integration/test_nav2_tf_handler.py
      tests/integration/test_nav2_robot_adapter.py
    )
    foreach(_test_path ${_pytest_tests})
      get_filename_component(_test_name ${_test_path} NAME_WE)
      ament_add_pytest_test(${_test_name} ${_test_path}
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 60
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
    endforeach()
  elseif (NAV1_INTEGRATION_TESTING)
    # test_nav1_move_base_handler and test_nav1_robot_adapter will affect one
    # another, therefore only one can be run during integration testing
    set(_pytest_tests
      # tests/integration/test_nav1_move_base_handler.py
      tests/integration/test_nav1_tf_handler.py
      tests/integration/test_nav1_robot_adapter.py
    )
    foreach(_test_path ${_pytest_tests})
      get_filename_component(_test_name ${_test_path} NAME_WE)
      ament_add_pytest_test(${_test_name} ${_test_path}
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 60
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
    endforeach()
  elseif (E2E_TESTING)
    set(_pytest_tests
      tests/e2e/test_nav1_fleet_adapter.py
    )
    foreach(_test_path ${_pytest_tests})
      get_filename_component(_test_name ${_test_path} NAME_WE)
      ament_add_pytest_test(${_test_name} ${_test_path}
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 60
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
    endforeach()
  endif()
endif()

ament_package()
