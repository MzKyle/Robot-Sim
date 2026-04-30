#include "file_reader/yaml_reader.h"
#include "rclcpp/rclcpp.hpp"

// 辅助函数：打印6D位姿
static void printPose6D(const rclcpp::Logger &logger, const std::string &name, const Pose6D &pose) {
    RCLCPP_INFO(logger, "%s位姿：", name.c_str());
    RCLCPP_INFO(logger, "  x: %.6f, y: %.6f, z: %.6f", pose.x, pose.y, pose.z);
    RCLCPP_INFO(logger, "  rx: %.6f, ry: %.6f, rz: %.6f", pose.rx, pose.ry, pose.rz);
}

int main(int argc, char *argv[]) {
    // 初始化ROS2节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("yaml_reader_test_node");

    try {
        //////// nodemanage.yaml
        // 传入YAML文件路径（请替换为你的实际文件路径）
        std::string yaml_path = "/etc/WR/Project/nodemanage.yaml";
        ROS2YamlReader reader(yaml_path);
        auto cover_pass_params = reader.readCoverPassNodeParams();
        RCLCPP_INFO(node->get_logger(), "cover_pass_node：%s:%s", "so_file_path", cover_pass_params.so_file_path.c_str());
        RCLCPP_INFO(node->get_logger(), "cover_pass_node：%s:%s", "config_file_path", cover_pass_params.config_file_path.c_str());

        auto camera_3d_params = reader.readCameraDriver3DParams();
        RCLCPP_INFO(node->get_logger(), "camera_driver_3d：%s:%s", "cfg", camera_3d_params.cfg.c_str());

        auto fanuc_params = reader.readRobotDriverFanucParams();
        // 使用参数初始化机器人驱动
        RCLCPP_INFO(node->get_logger(), "连接机器人：%s:%d", fanuc_params.robot_ip.c_str(), fanuc_params.robot_port);
        // load_so(fanuc_params.so_file_path); // 示例：加载SO文件


        RCLCPP_INFO(node->get_logger(), "读取节点参数:");
        auto datacollect_params = reader.readDataCollectNodeParams();
        RCLCPP_INFO(node->get_logger(), "datacollect_params：%s:%s", "save_dir_root", datacollect_params.save_dir_root.c_str());
        RCLCPP_INFO(node->get_logger(), "datacollect_params：%s:%d", "image_save_interval", datacollect_params.image_save_interval);

        RCLCPP_INFO(node->get_logger(), "%s 配置读取成功！",yaml_path.c_str());
        RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------------");
        RCLCPP_INFO(node->get_logger(), " ");

        ////// weldingpara.yaml
        yaml_path = "/etc/WR/Project/weldingpara.yaml";
        // 调用通用读取函数
        WeldingConfig config = readWeldingConfigFromYaml(yaml_path);

        // 打印读取结果（示例）
        RCLCPP_INFO(node->get_logger(), "日志根目录: %s", config.logs_root.c_str());
        RCLCPP_INFO(node->get_logger(), "SAM2模型路径: %s", config.seam_detection.sam2_onnx_path.c_str());
        RCLCPP_INFO(node->get_logger(), "点云过滤y轴范围: [%.4f, %.4f]",
                    config.height_detection.cloud_clip_y_min,
                    config.height_detection.cloud_clip_y_max);
        RCLCPP_INFO(node->get_logger(), "默认焊接方向: [%.1f, %.1f, %.1f]",
                    config.welding_direction.default_welding_direction[0],
                    config.welding_direction.default_welding_direction[1],
                    config.welding_direction.default_welding_direction[2]);

        RCLCPP_INFO(node->get_logger(), "%s 配置读取成功！",yaml_path.c_str());
        RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------------");
        RCLCPP_INFO(node->get_logger(), " ");

        /////// cameratcp.yaml
        // 传入YAML文件路径（请替换为你的实际文件路径）
        yaml_path = "/etc/WR/Project/cameratcp.yaml";

        // 调用读取函数（核心：仅需传入文件路径）
        CameraConfig cameraConfig = readCameraConfigFromYaml(yaml_path);

        // 打印解析结果
        RCLCPP_INFO(node->get_logger(), "===== 解析的YAML配置 =====");
        printPose6D(node->get_logger(), "Camera", cameraConfig.camera);
        printPose6D(node->get_logger(), "Tool", cameraConfig.tool);

        RCLCPP_INFO(node->get_logger(), "\n阈值配置：");
        RCLCPP_INFO(node->get_logger(), "y_min: %.6f, y_max: %.6f", cameraConfig.y_min, cameraConfig.y_max);
        RCLCPP_INFO(node->get_logger(), "z_min: %.6f, z_max: %.6f", cameraConfig.z_min, cameraConfig.z_max);
        RCLCPP_INFO(node->get_logger(), "percentile_low: %.3f, percentile_high: %.3f",
                    cameraConfig.percentile_low, cameraConfig.percentile_high);
        RCLCPP_INFO(node->get_logger(), "number_points_threshold: %d", cameraConfig.number_points_threshold);
        RCLCPP_INFO(node->get_logger(), "no_detection_count_threshold: %d", cameraConfig.no_detection_count_threshold);
        RCLCPP_INFO(node->get_logger(), "y_min_f: %.6f, y_max_f: %.6f", cameraConfig.y_min_f, cameraConfig.y_max_f);

        RCLCPP_INFO(node->get_logger(), "%s 配置读取成功！",yaml_path.c_str());

    } catch (const std::runtime_error &e) {
        RCLCPP_ERROR(node->get_logger(), "读取YAML配置失败: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}
