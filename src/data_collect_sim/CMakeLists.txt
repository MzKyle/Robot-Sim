cmake_minimum_required(VERSION 3.8)
project(data_collect_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sim_camera_2d_node
  scripts/sim_camera_3d_node
  scripts/sim_fanuc_robot_node
  scripts/tf_to_tcp_node
  scripts/spawn_panda_weld_arm_node
  scripts/check_gz_model_node
  scripts/joint_state_to_gz_joint_cmd_node
  scripts/panda_joint_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
