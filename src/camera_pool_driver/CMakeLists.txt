cmake_minimum_required(VERSION 3.8)
project(camera_pool_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 声明头文件目录
include_directories(
        include/camera_sdk
        ${OpenCV_INCLUDE_DIRS}
)

# 设置相机SDK库路径（保留原有逻辑）
link_directories(lib)

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

add_executable(camera_pool_driver
        src/camera_pool_driver.cpp
        )

# 设置目标依赖
ament_target_dependencies(
        camera_pool_driver
        rclcpp
        cv_bridge
        sensor_msgs
        OpenCV
)
# 链接库文件（保留原有相机SDK链接）
target_link_libraries(camera_pool_driver
        ${OpenCV_LIBRARIES}
        MVSDK  # 相机SDK库（原MVSDK）
        )

install(TARGETS
        camera_pool_driver
        DESTINATION lib/${PROJECT_NAME}
        )