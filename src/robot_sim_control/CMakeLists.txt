cmake_minimum_required(VERSION 3.8)
project(robot_sim_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joint_state_to_gz_joint_cmd_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
