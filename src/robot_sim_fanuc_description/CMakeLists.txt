cmake_minimum_required(VERSION 3.8)
project(robot_sim_fanuc_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  models
  robots
  upstream
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
