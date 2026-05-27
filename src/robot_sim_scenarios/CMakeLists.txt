cmake_minimum_required(VERSION 3.8)
project(robot_sim_scenarios)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  assets
  scenarios
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
