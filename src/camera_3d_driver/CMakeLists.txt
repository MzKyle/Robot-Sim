cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

project(camera_3d_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ====================== 查找ROS2依赖包 ======================
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)          # ROS2替代roscpp
find_package(cv_bridge REQUIRED)       # CV-Bridge（ROS2版本）
find_package(pcl_conversions REQUIRED) # PCL与ROS2消息转换
find_package(pcl_ros REQUIRED)         # PCL ROS2接口
find_package(sensor_msgs REQUIRED)     # 传感器消息
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)         # ROS2 TF（替代原tf）
find_package(tf2 REQUIRED)             # TF2核心库
find_package(tf2_geometry_msgs REQUIRED) # TF2几何消息转换
find_package(std_srvs REQUIRED)
# ====================== 查找系统依赖包 ======================
find_package(OpenCV REQUIRED)          # OpenCV
find_package(PCL REQUIRED)             # PCL点云库
find_package(yaml-cpp REQUIRED)        # YAML解析库
# ====================== 自定义消息/服务生成（ROS2） ======================
find_package(weld_interface REQUIRED)  # 新增这行，确保先找到该包
find_package(file_reader REQUIRED)     # 文件读取器
find_package(rosidl_default_generators REQUIRED)

find_package(tf2_eigen REQUIRED)

set(RVC_ROOT "$ENV{RVC_ROOT}")
if(NOT RVC_ROOT)
        set(RVC_ROOT "/opt/RVC")
endif()
set(RVC_INCLUDE_DIR "${RVC_ROOT}/include")
set(RVC_LIBRARY_DIR "${RVC_ROOT}/lib")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_camera_3d_driver
    test/test_camera_3d_driver.cpp
  )
  target_include_directories(test_camera_3d_driver PRIVATE
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_camera_3d_driver
    tf2
    tf2_eigen
  )
  target_link_libraries(test_camera_3d_driver
    ${PCL_LIBRARIES}
  )
endif()



# ====================== 头文件目录 ======================
include_directories(
        include
        ${catkin_INCLUDE_DIRS}  # 兼容旧代码，ROS2中可移除
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        ${RVC_INCLUDE_DIR}
)

# ====================== 库路径 ======================
link_directories(
        libs                     # 保留原自定义库路径
        ${PCL_LIBRARY_DIRS}      # PCL库路径
        ${RVC_LIBRARY_DIR}       # RVC SDK库路径
)


# 罗列所有源文件
add_executable(camera_3d_driver
        src/camera_3d_driver.cpp
        )

ament_target_dependencies(
        camera_3d_driver
        rclcpp
        weld_interface
        file_reader
        tf2          # 新增
        tf2_ros      # 新增
        cv_bridge
        pcl_conversions
        pcl_ros
        sensor_msgs
        tf2_geometry_msgs
        OpenCV
        std_msgs
        std_srvs  # ✅ 新增：链接 std_srvs 依赖
        tf2_eigen
        ament_index_cpp
)

# 3. 查找 RVC 库
find_library(RVC_LIB
        NAMES RVC libRVC.so.1
        PATHS ${RVC_LIBRARY_DIR}
        REQUIRED
        NO_DEFAULT_PATH
        )


target_link_libraries(camera_3d_driver
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${RVC_LIB}  # 使用找到的库变量
        )

install(TARGETS
        camera_3d_driver
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
        )

ament_package()
