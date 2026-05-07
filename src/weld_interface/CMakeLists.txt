cmake_minimum_required(VERSION 3.8)
project(weld_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}_headers INTERFACE)
target_include_directories(${PROJECT_NAME}_headers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )


rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/FanucRobotInfo.msg"
        "msg/TcpPos.msg"
        "msg/Height.msg"
        "msg/LiftDrop.msg"
        "msg/Line.msg"
        "msg/MoveAxes.msg"
        "msg/Point.msg"
        "msg/LineCoeffs.msg"
        "msg/PlaneCoeffs.msg"
        "msg/RobotCommand.msg"
        "msg/DataCollectStatus.msg"
        "srv/SpecialSpeedl.srv"
        "srv/Move.srv"
        "srv/FanucMovRate.srv"
        "srv/CallAi.srv"
        "srv/InitPoint.srv"
        "srv/Scan3d.srv"
        "srv/Run.srv"
        "srv/UpdateHeight.srv"
        "srv/UpdateRange.srv"
        "srv/MoveToFirstTeachingPoint.srv"
        "srv/MoveToNextTeachingPoint.srv"
        "srv/RecordCurrentTeaching.srv"
        "srv/UpdateLineCoeffs.srv"
        "srv/UpdateMoveAxes.srv"
        "srv/MoveToInitialPoint.srv"
        "srv/UpdatePlaneCoeffs.srv"
        "srv/StartForward.srv"
        "srv/IntData.srv"
        "srv/ReadFanucRegister.srv"
        "srv/SetCollectionTask.srv"
        "msg/ModbusData.msg"
        "msg/ModbusDataArray.msg"
        "srv/WriteDataPoint.srv"
        DEPENDENCIES std_msgs sensor_msgs
        )

      install(DIRECTORY include/
        DESTINATION include
        )

      install(TARGETS ${PROJECT_NAME}_headers
        EXPORT export_${PROJECT_NAME}
        )

      ament_export_targets(export_${PROJECT_NAME})
      ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
