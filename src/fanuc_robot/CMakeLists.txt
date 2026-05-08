cmake_minimum_required(VERSION 3.8)
project(fanuc_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(weld_interface REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)          # 新增：tf2 核心包
find_package(tf2_ros REQUIRED)      # 新增：tf2 的ROS2接口包


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



# 罗列所有源文件
add_executable(fanuc_robot
        src/fanuc_interface.cpp
        src/fanuc_interface.h
        src/fanuc_driver.cpp
        )
ament_target_dependencies(
        fanuc_robot
        rclcpp
        std_msgs
        weld_interface
        std_srvs  # 新增这一行，将std_srvs链接到可执行文件
        tf2          # 新增
        tf2_ros      # 新增
        ament_index_cpp
)

# 3. 安装可执行文件到指定路径（ROS2标准要求）
install(TARGETS
        fanuc_robot
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY lib
        DESTINATION share/${PROJECT_NAME}
        )
