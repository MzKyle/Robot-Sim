cmake_minimum_required(VERSION 3.8)
project(data_collect_quality)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(weld_interface REQUIRED)
find_package(acquisition_interfaces REQUIRED)

add_executable(data_collect_quality_node
  src/data_collect_quality_node.cpp
)

ament_target_dependencies(data_collect_quality_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  weld_interface
  acquisition_interfaces
)

install(TARGETS
  data_collect_quality_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
