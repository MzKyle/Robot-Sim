
cmake_minimum_required(VERSION 3.8)
project(file_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# 查找nlohmann_json（ROS2 Humble+已内置，若未找到可手动安装：sudo apt install ros-$ROS_DISTRO-nlohmann-json）
#find_package(nlohmann_json REQUIRED)

# 构建库（通用功能封装为库，方便其他节点调用）
add_library(json_reader_lib
        src/json_reader.cpp
        )
ament_target_dependencies(json_reader_lib
        rclcpp
        #        nlohmann_json
        )
# 包含头文件目录
target_include_directories(json_reader_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

# 安装库和头文件
install(TARGETS json_reader_lib
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )
install(DIRECTORY include/
        DESTINATION include
        )

# 示例节点（可选，用于测试）
add_executable(json_reader_test
        src/json_reader_test.cpp
        )
ament_target_dependencies(json_reader_test
        rclcpp
        #        nlohmann_json
        )
target_link_libraries(json_reader_test
        json_reader_lib
        )

install(TARGETS json_reader_test
        DESTINATION lib/${PROJECT_NAME}
        )


#ymal

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

#add_executable(yaml_reader src/yaml_reader.cpp)
# 生成库文件（供其他节点调用）
add_library(yaml_reader_lib SHARED
        src/yaml_reader.cpp)
ament_target_dependencies(yaml_reader_lib
        rclcpp
        yaml-cpp
        )

#target_include_directories(yaml_reader PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
target_include_directories(yaml_reader_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )
target_link_libraries(yaml_reader_lib
        yaml-cpp
        )

# 安装库和头文件
install(TARGETS yaml_reader_lib
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        )
install(DIRECTORY include/
        DESTINATION include
        )

#target_compile_features(yaml_reader PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(yaml_reader_lib PUBLIC c_std_99 cxx_std_17)

# 测试节点（可选，验证功能）
add_executable(yaml_reader_test
        src/yaml_reader_test.cpp
        )
ament_target_dependencies(yaml_reader_test
        rclcpp
        yaml-cpp
        )
target_link_libraries(yaml_reader_test
        yaml_reader_lib
        )
install(TARGETS yaml_reader_test
        DESTINATION lib/${PROJECT_NAME}
        )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_package()