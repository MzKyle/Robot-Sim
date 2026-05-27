cmake_minimum_required(VERSION 3.8)
project(acquisition_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_task_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AcquisitionQuality.msg"
  "msg/AcquisitionStatus.msg"
  "srv/SetAcquisitionTask.srv"
  DEPENDENCIES std_msgs robot_task_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
