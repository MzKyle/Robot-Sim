cmake_minimum_required(VERSION 3.8)
project(robot_sim_sensor_depth)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(depth_receiver src/depth_receiver.cpp)
target_compile_features(depth_receiver PUBLIC cxx_std_17)
ament_target_dependencies(depth_receiver
  diagnostic_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS
  depth_receiver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
