#include "file_reader/yaml_reader.h"

// 构造函数：加载YAML文件
ROS2YamlReader::ROS2YamlReader(const std::string &yaml_file_path) {
    try {
        yaml_node_ = YAML::LoadFile(yaml_file_path);
    } catch (const YAML::BadFile &e) {
        throw std::runtime_error("加载YAML文件失败: " + yaml_file_path + "，错误：" + e.what());
    } catch (const std::exception &e) {
        throw std::runtime_error("解析YAML文件异常: " + std::string(e.what()));
    }
}

// 读取CoverPassNode参数
CoverPassNodeParams ROS2YamlReader::readCoverPassNodeParams(const std::string &node_name) {
    CoverPassNodeParams params;
    YAML::Node params_node = yaml_node_[node_name]["ros__parameters"];

    if (!params_node) {
        throw std::runtime_error("节点 " + node_name + " 无 ros__parameters 配置");
    }

    // 读取参数（带存在性检查）
    if (params_node["so_file_path"]) {
        params.so_file_path = params_node["so_file_path"].as<std::string>();
    } else {
        throw std::runtime_error(node_name + " 缺失参数：so_file_path");
    }

    if (params_node["config_file_path"]) {
        params.config_file_path = params_node["config_file_path"].as<std::string>();
    } else {
        throw std::runtime_error(node_name + " 缺失参数：config_file_path");
    }

    if (params_node["image_height"]) {
        params.image_height = params_node["image_height"].as<int>();
    } else {
        throw std::runtime_error(node_name + " 缺失参数：image_height");
    }

    if (params_node["mode"]) {
        params.mode = params_node["mode"].as<int>();
    } else {
        throw std::runtime_error(node_name + " 缺失参数：mode");
    }

    return params;
}

// 读取TeachNode参数
TeachNodeParams ROS2YamlReader::readTeachNodeParams(const std::string &node_name) {
    TeachNodeParams params;
    YAML::Node params_node = yaml_node_[node_name]["ros__parameters"];

    if (!params_node) {
        throw std::runtime_error("节点 " + node_name + " 无 ros__parameters 配置");
    }

    if (params_node["image_height"]) {
        params.image_height = params_node["image_height"].as<int>();
    } else {
        throw std::runtime_error(node_name + " 缺失参数：image_height");
    }

    return params;
}

// 读取CameraDriver3D参数
CameraDriver3DParams ROS2YamlReader::readCameraDriver3DParams(const std::string &node_name) {
    CameraDriver3DParams params;
    YAML::Node params_node = yaml_node_[node_name]["ros__parameters"];

    if (!params_node) {
        throw std::runtime_error("节点 " + node_name + " 无 ros__parameters 配置");
    }

    if (params_node["cfg"]) {
        params.cfg = params_node["cfg"].as<std::string>();
    } else {
        throw std::runtime_error(node_name + " 缺失参数：cfg");
    }

    return params;
}

// 读取RobotDriverFanuc参数
RobotDriverFanucParams ROS2YamlReader::readRobotDriverFanucParams(const std::string &node_name) {
    RobotDriverFanucParams params;
    YAML::Node params_node = yaml_node_[node_name]["ros__parameters"];

    if (!params_node) {
        throw std::runtime_error("节点 " + node_name + " 无 ros__parameters 配置");
    }

    if (params_node["so_file_path"]) {
        params.so_file_path = params_node["so_file_path"].as<std::string>();
    } else {
        throw std::runtime_error(node_name + " 缺失参数：so_file_path");
    }

    if (params_node["robot_ip"]) {
        params.robot_ip = params_node["robot_ip"].as<std::string>();
    } else {
        throw std::runtime_error(node_name + " 缺失参数：robot_ip");
    }

    if (params_node["robot_port"]) {
        params.robot_port = params_node["robot_port"].as<int>();
    } else {
        throw std::runtime_error(node_name + " 缺失参数：robot_port");
    }

    return params;
}

// 读取DataCollectNode参数
DataCollectNodeParams ROS2YamlReader::readDataCollectNodeParams(const std::string &node_name) {
    DataCollectNodeParams params;
    YAML::Node params_node = yaml_node_[node_name]["ros__parameters"];

    if (!params_node) {
        throw std::runtime_error("节点 " + node_name + " 无 ros__parameters 配置");
    }

    if (params_node["save_dir_root"]) {
        params.save_dir_root = params_node["save_dir_root"].as<std::string>();
    } else {
        throw std::runtime_error(node_name + " 缺失参数：save_dir_root");
    }

    if (params_node["image_save_interval"]) {
        params.image_save_interval = params_node["image_save_interval"].as<int>();
    }

    if (params_node["image_log_save_interval"]) {
        params.image_log_save_interval = params_node["image_log_save_interval"].as<int>();
    }

    if (params_node["height_log_save_interval"]) {
        params.height_log_save_interval = params_node["height_log_save_interval"].as<int>();
    }

    if (params_node["fix_scan_interval"]) {
        params.fix_scan_interval = params_node["fix_scan_interval"].as<int>();
    }
    if (params_node["auto_save_flag"]) {
        params.auto_save_flag = params_node["auto_save_flag"].as<int>();
    }
    if (params_node["target_register_index"]) {
        params.target_register_index = params_node["target_register_index"].as<int>();
    }
    if (params_node["weld_type_mapping"]) {
        YAML::Node mapping_node = params_node["weld_type_mapping"];
        for (auto it = mapping_node.begin(); it != mapping_node.end(); ++it) {
            int key = it->first.as<int>();
            std::string value = it->second.as<std::string>();
            params.weld_type_mapping[key] = value;
        }
    }
    return params;
}

// 模板特化 - CoverPassNode
template <>
CoverPassNodeParams readROS2NodeParams<CoverPassNodeParams>(const std::string &yaml_file_path, const std::string &node_name) {
    ROS2YamlReader reader(yaml_file_path);
    return reader.readCoverPassNodeParams(node_name);
}

// 模板特化 - TeachNode
template <>
TeachNodeParams readROS2NodeParams<TeachNodeParams>(const std::string &yaml_file_path, const std::string &node_name) {
    ROS2YamlReader reader(yaml_file_path);
    return reader.readTeachNodeParams(node_name);
}

// 模板特化 - RobotDriverFanuc
template <>
RobotDriverFanucParams readROS2NodeParams<RobotDriverFanucParams>(const std::string &yaml_file_path, const std::string &node_name) {
    ROS2YamlReader reader(yaml_file_path);
    return reader.readRobotDriverFanucParams(node_name);
}

// 辅助函数：检查节点是否存在，不存在则抛异常
void checkYamlNodeExists(const YAML::Node &node, const std::string &key, const std::string &parent = "") {
    std::string full_key = parent.empty() ? key : parent + "." + key;
    if (!node[key]) {
        throw std::runtime_error("YAML文件缺失必填字段: " + full_key);
    }
}

WeldingConfig readWeldingConfigFromYaml(const std::string &yaml_file_path) {
    WeldingConfig config;

    // 1. 加载YAML文件
    YAML::Node yaml_node;
    try {
        yaml_node = YAML::LoadFile(yaml_file_path);
    } catch (const YAML::BadFile &e) {
        throw std::runtime_error("无法加载YAML文件: " + yaml_file_path + "，错误: " + e.what());
    } catch (const std::exception &e) {
        throw std::runtime_error("解析YAML文件失败: " + yaml_file_path + "，错误: " + e.what());
    }

    // 2. 读取根级字段
    checkYamlNodeExists(yaml_node, "logs_root");
    config.logs_root = yaml_node["logs_root"].as<std::string>();

    // 3. 读取焊缝检测参数
    checkYamlNodeExists(yaml_node, "sam2_onnx_path");
    config.seam_detection.sam2_onnx_path = yaml_node["sam2_onnx_path"].as<std::string>();

    checkYamlNodeExists(yaml_node, "seam_remove_high_white_threshold");
    config.seam_detection.seam_remove_high_white_threshold = yaml_node["seam_remove_high_white_threshold"].as<int>();

    checkYamlNodeExists(yaml_node, "seam_width_pixels_min_threshold");
    config.seam_detection.seam_width_pixels_min_threshold = yaml_node["seam_width_pixels_min_threshold"].as<int>();

    checkYamlNodeExists(yaml_node, "seam_width_pixels_max_threshold");
    config.seam_detection.seam_width_pixels_max_threshold = yaml_node["seam_width_pixels_max_threshold"].as<int>();

    checkYamlNodeExists(yaml_node, "seam_width_filtering_threshold");
    config.seam_detection.seam_width_filtering_threshold = yaml_node["seam_width_filtering_threshold"].as<double>();

    checkYamlNodeExists(yaml_node, "line_number_points_threshold_ratio_threshold");
    config.seam_detection.line_number_points_threshold_ratio_threshold =
            yaml_node["line_number_points_threshold_ratio_threshold"].as<double>();

    checkYamlNodeExists(yaml_node, "line_history_size");
    config.seam_detection.line_history_size = yaml_node["line_history_size"].as<int>();

    // 4. 读取高度检测参数
    checkYamlNodeExists(yaml_node, "cloud_clip_y_min");
    config.height_detection.cloud_clip_y_min = yaml_node["cloud_clip_y_min"].as<double>();

    checkYamlNodeExists(yaml_node, "cloud_clip_y_max");
    config.height_detection.cloud_clip_y_max = yaml_node["cloud_clip_y_max"].as<double>();

    checkYamlNodeExists(yaml_node, "cloud_clip_z_min");
    config.height_detection.cloud_clip_z_min = yaml_node["cloud_clip_z_min"].as<double>();

    checkYamlNodeExists(yaml_node, "cloud_clip_z_max");
    config.height_detection.cloud_clip_z_max = yaml_node["cloud_clip_z_max"].as<double>();

    checkYamlNodeExists(yaml_node, "height_number_points_threshold");
    config.height_detection.height_number_points_threshold = yaml_node["height_number_points_threshold"].as<int>();

    checkYamlNodeExists(yaml_node, "height_percentile_low");
    config.height_detection.height_percentile_low = yaml_node["height_percentile_low"].as<double>();

    checkYamlNodeExists(yaml_node, "height_percentile_high");
    config.height_detection.height_percentile_high = yaml_node["height_percentile_high"].as<double>();

    checkYamlNodeExists(yaml_node, "height_history_size");
    config.height_detection.height_history_size = yaml_node["height_history_size"].as<int>();

    // 5. 读取臂架检测参数
    checkYamlNodeExists(yaml_node, "classifier_onnx_path");
    config.boom_detection.classifier_onnx_path = yaml_node["classifier_onnx_path"].as<std::string>();

    checkYamlNodeExists(yaml_node, "detection_history_size");
    config.boom_detection.detection_history_size = yaml_node["detection_history_size"].as<int>();

    checkYamlNodeExists(yaml_node, "detection_count_theshold");
    config.boom_detection.detection_count_theshold = yaml_node["detection_count_theshold"].as<int>();

    checkYamlNodeExists(yaml_node, "no_detection_count_theshold");
    config.boom_detection.no_detection_count_theshold = yaml_node["no_detection_count_theshold"].as<int>();

    // 6. 读取焊接方向检测参数
    checkYamlNodeExists(yaml_node, "default_welding_direction");
    config.welding_direction.default_welding_direction =
            yaml_node["default_welding_direction"].as<std::vector<double>>();

    // 验证数组长度（必须是3个元素）
    if (config.welding_direction.default_welding_direction.size() != 3) {
        throw std::runtime_error("default_welding_direction 必须是3维向量（x,y,z）");
    }

    checkYamlNodeExists(yaml_node, "welding_direction_update_distance_threshold");
    config.welding_direction.welding_direction_update_distance_threshold =
            yaml_node["welding_direction_update_distance_threshold"].as<double>();

    checkYamlNodeExists(yaml_node, "dynamic_update_welding_direction_alpha");
    config.welding_direction.dynamic_update_welding_direction_alpha =
            yaml_node["dynamic_update_welding_direction_alpha"].as<double>();

    return config;
}


/**
 * CameraConfig
 * **/
// 辅助函数：解析6自由度位姿（复用camera/tool的解析逻辑）
static Pose6D parsePose6D(const YAML::Node &node, const std::string &node_name) {
    Pose6D pose;
    // 检查节点是否存在
    if (!node) {
        throw std::runtime_error("YAML文件中缺失节点: " + node_name);
    }

    // 解析x字段
    if (node["x"]) {
        pose.x = node["x"].as<double>();
    } else {
        throw std::runtime_error(node_name + "节点缺失字段: x");
    }

    // 解析y字段
    if (node["y"]) {
        pose.y = node["y"].as<double>();
    } else {
        throw std::runtime_error(node_name + "节点缺失字段: y");
    }

    // 解析z字段
    if (node["z"]) {
        pose.z = node["z"].as<double>();
    } else {
        throw std::runtime_error(node_name + "节点缺失字段: z");
    }

    // 解析rx字段
    if (node["rx"]) {
        pose.rx = node["rx"].as<double>();
    } else {
        throw std::runtime_error(node_name + "节点缺失字段: rx");
    }

    // 解析ry字段
    if (node["ry"]) {
        pose.ry = node["ry"].as<double>();
    } else {
        throw std::runtime_error(node_name + "节点缺失字段: ry");
    }

    // 解析rz字段
    if (node["rz"]) {
        pose.rz = node["rz"].as<double>();
    } else {
        throw std::runtime_error(node_name + "节点缺失字段: rz");
    }

    return pose;
}

// 核心读取函数
CameraConfig readCameraConfigFromYaml(const std::string &yaml_file_path) {
    CameraConfig config;

    // 加载YAML文件
    YAML::Node root_node;
    try {
        root_node = YAML::LoadFile(yaml_file_path);
    } catch (const YAML::BadFile &e) {
        throw std::runtime_error("无法加载YAML文件: " + yaml_file_path + "，错误: " + e.what());
    } catch (const std::exception &e) {
        throw std::runtime_error("解析YAML文件格式错误: " + std::string(e.what()));
    }

    // 解析camera节点
    config.camera = parsePose6D(root_node["camera"], "camera");

    // 解析tool节点
    config.tool = parsePose6D(root_node["tool"], "tool");

    // 解析平级浮点数字段
    auto parseDoubleField = [&](const std::string &field_name, double &dest) {
        if (root_node[field_name]) {
            dest = root_node[field_name].as<double>();
        } else {
            throw std::runtime_error("YAML文件缺失字段: " + field_name);
        }
    };

    parseDoubleField("y_min", config.y_min);
    parseDoubleField("y_max", config.y_max);
    parseDoubleField("z_min", config.z_min);
    parseDoubleField("z_max", config.z_max);
    parseDoubleField("percentile_low", config.percentile_low);
    parseDoubleField("percentile_high", config.percentile_high);
    parseDoubleField("y_min_f", config.y_min_f);
    parseDoubleField("y_max_f", config.y_max_f);

    // 解析整型字段（单独处理，避免类型转换错误）
    auto parseIntField = [&](const std::string &field_name, int &dest) {
        if (root_node[field_name]) {
            // 先转double再转int，兼容YAML中写成浮点的整型（如25.0）
            double val = root_node[field_name].as<double>();
            if (std::floor(val) != val) {
                throw std::runtime_error(field_name + "字段应为整数，实际值: " + std::to_string(val));
            }
            dest = static_cast<int>(val);
        } else {
            throw std::runtime_error("YAML文件缺失字段: " + field_name);
        }
    };

    parseIntField("number_points_threshold", config.number_points_threshold);
    parseIntField("no_detection_count_threshold", config.no_detection_count_threshold);

    return config;
}
