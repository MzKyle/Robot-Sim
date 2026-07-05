cmake_minimum_required(VERSION 3.8)
project(robot_sim_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  schemas
  DESTINATION share/${PROJECT_NAME}
)

foreach(robot_sim_asset_dir examples integrations)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../${robot_sim_asset_dir}")
    install(DIRECTORY
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../${robot_sim_asset_dir}"
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

install(PROGRAMS
  scripts/migrate_config
  scripts/profile_lint
  scripts/run_case
  scripts/scaffold_robot
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_validation_cases test/test_validation_cases.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../robot_sim_scenarios
  )
endif()

ament_package()
