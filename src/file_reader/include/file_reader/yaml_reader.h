#ifndef YAML_READER__YAML_READER_HPP_
#define YAML_READER__YAML_READER_HPP_
//
//#include <yaml-cpp/yaml.h>
//#include <string>
//#include <stdexcept>
//
//// 定义与YAML文件对应的结构体
//struct GeneralConfig {
//    // PG数据库连接字符串
//    std::string pg_conn_str;
//    // 插件路径
//    std::string plugin_path;
//    // 厚板场景参数路径
//    std::string scene_config_path;
//};
//
///**
// * @brief 通用的YAML文件读取函数
// * @param yaml_file_path YAML文件的绝对路径
// * @return 解析后的配置结构体
// * @throw std::runtime_error 文件不存在/字段缺失时抛出异常
// */
//GeneralConfig readGeneralConfigFromYaml(const std::string &yaml_file_path);

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <unordered_map>

/**
 * 节点管理参数
 * */

// 定义各节点的参数结构体（按需扩展）
struct CoverPassNodeParams {
    std::string so_file_path;    // 算法SO文件路径
    std::string config_file_path;// 配置文件路径
    int image_height{0};         // 图像高度
    int mode{0};                 // 运行模式
};

struct TeachNodeParams {
    int image_height{0};         // 图像高度
};

struct CameraDriver3DParams {
    std::string cfg;             // 相机配置文件路径
};

struct RobotDriverFanucParams {
    std::string so_file_path;    // 机器人SO文件路径
    std::string robot_ip;        // 机器人IP
    int robot_port{0};           // 机器人端口
};

struct DataCollectNodeParams {
    std::string save_dir_root;   // 数据保存根目录
    int image_save_interval{12};
    int image_log_save_interval{3};
    int height_log_save_interval{4};
    int fix_scan_interval{6};
    int auto_save_flag{0};
    int target_register_index{100};
};

/**
 * @brief ROS2 YAML参数文件通用读取类
 * 支持读取指定节点的ros__parameters下的所有参数
 */
class ROS2YamlReader {
public:
    /**
     * @brief 构造函数（加载YAML文件）
     * @param yaml_file_path YAML文件绝对路径
     */
    explicit ROS2YamlReader(const std::string &yaml_file_path);

    /**
     * @brief 读取CoverPassNode节点参数
     * @param node_name 节点名称（默认：cover_pass_node）
     * @return 解析后的参数结构体
     */
    CoverPassNodeParams readCoverPassNodeParams(const std::string &node_name = "cover_pass_node");

    /**
     * @brief 读取TeachNode节点参数
     * @param node_name 节点名称（默认：teach_node）
     * @return 解析后的参数结构体
     */
    TeachNodeParams readTeachNodeParams(const std::string &node_name = "teach_node");

    /**
     * @brief 读取CameraDriver3D节点参数
     * @param node_name 节点名称（默认：camera_driver_3d）
     * @return 解析后的参数结构体
     */
    CameraDriver3DParams readCameraDriver3DParams(const std::string &node_name = "camera_driver_3d");

    /**
     * @brief 读取RobotDriverFanuc节点参数
     * @param node_name 节点名称（默认：robot_driver_fanuc）
     * @return 解析后的参数结构体
     */
    RobotDriverFanucParams readRobotDriverFanucParams(const std::string &node_name = "robot_driver_fanuc");

    /**
     * @brief 读取DataCollectNode节点参数
     * @param node_name 节点名称（默认：data_collect_node）
     * @return 解析后的参数结构体
     */
    DataCollectNodeParams readDataCollectNodeParams(const std::string &node_name = "data_collect_node");

    /**
     * @brief 通用读取单个参数（适用于自定义参数）
     * @tparam T 参数类型（string/int/double等）
     * @param node_name 节点名称
     * @param param_key 参数键名
     * @return 参数值
     */
    template <typename T>
    T readParam(const std::string &node_name, const std::string &param_key) {
        // 检查节点是否存在
        if (!yaml_node_[node_name]) {
            throw std::runtime_error("节点 " + node_name + " 在YAML文件中不存在");
        }
        // 检查ros__parameters是否存在
        YAML::Node params_node = yaml_node_[node_name]["ros__parameters"];
        if (!params_node) {
            throw std::runtime_error("节点 " + node_name + " 下无 ros__parameters 配置");
        }
        // 检查参数是否存在
        if (!params_node[param_key]) {
            throw std::runtime_error("节点 " + node_name + " 的参数 " + param_key + " 不存在");
        }
        return params_node[param_key].as<T>();
    }

private:
    YAML::Node yaml_node_;  // YAML根节点
};

/**
 * @brief 便捷函数：直接读取指定节点的参数（无需创建类实例）
 * @tparam T 参数结构体类型
 * @param yaml_file_path YAML文件路径
 * @param node_name 节点名称
 * @return 解析后的参数结构体
 */
template <typename T>
T readROS2NodeParams(const std::string &yaml_file_path, const std::string &node_name);

// 模板特化（针对各节点参数结构体）
template <>
CoverPassNodeParams readROS2NodeParams<CoverPassNodeParams>(const std::string &yaml_file_path, const std::string &node_name);

template <>
TeachNodeParams readROS2NodeParams<TeachNodeParams>(const std::string &yaml_file_path, const std::string &node_name);

template <>
RobotDriverFanucParams readROS2NodeParams<RobotDriverFanucParams>(const std::string &yaml_file_path, const std::string &node_name);




/**
 * 焊接参数
 * **/
// 焊缝检测参数结构体
struct SeamDetectionParams {
    std::string sam2_onnx_path;              // SAM2模型路径
    int seam_remove_high_white_threshold;    // 去除高光像素阈值
    int seam_width_pixels_min_threshold;     // 单行焊缝像素数最小阈值
    int seam_width_pixels_max_threshold;     // 单行焊缝像素数最大阈值
    double seam_width_filtering_threshold;   // 参与直线拟合点的比例
    double line_number_points_threshold_ratio_threshold; // 焊缝行数比例阈值
    int line_history_size;                   // 直线历史缓冲区长度
};

// 高度检测参数结构体
struct HeightDetectionParams {
    double cloud_clip_y_min;                 // 点云过滤y轴最小值
    double cloud_clip_y_max;                 // 点云过滤y轴最大值
    double cloud_clip_z_min;                 // 点云过滤z轴最小值
    double cloud_clip_z_max;                 // 点云过滤z轴最大值
    int height_number_points_threshold;      // 参与高度估算至少点数
    double height_percentile_low;            // 高度估计下分位点
    double height_percentile_high;           // 高度估计上分位点
    int height_history_size;                 // 高度历史缓冲区长度
};

// 臂架检测参数结构体
struct BoomDetectionParams {
    std::string classifier_onnx_path;        // 分类器模型路径
    int detection_history_size;              // 臂架历史缓冲区长度
    int detection_count_theshold;            // 臂架检测阈值
    int no_detection_count_theshold;         // 臂架未检测阈值
};

// 焊接方向检测参数结构体
struct WeldingDirectionParams {
    std::vector<double> default_welding_direction; // 默认焊缝方向
    double welding_direction_update_distance_threshold; // 方向更新距离阈值
    double dynamic_update_welding_direction_alpha; // 方向更新权重
};

// 总配置结构体
struct WeldingConfig {
    std::string logs_root;                   // 日志根目录
    SeamDetectionParams seam_detection;      // 焊缝检测参数
    HeightDetectionParams height_detection;  // 高度检测参数
    BoomDetectionParams boom_detection;      // 臂架检测参数
    WeldingDirectionParams welding_direction;// 焊接方向检测参数
};

/**
 * @brief 读取算法配置YAML文件的通用函数
 * @param yaml_file_path YAML文件绝对路径
 * @return 解析后的完整算法配置结构体
 * @throw std::runtime_error 文件不存在/字段缺失/类型错误时抛出异常
 */
WeldingConfig readWeldingConfigFromYaml(const std::string &yaml_file_path);


/**
 *
 * 相机参数
 * */
// 定义6自由度位姿结构体（对应camera/tool节点）
struct Pose6D {
    double x;   // 平移X
    double y;   // 平移Y
    double z;   // 平移Z
    double rx;  // 旋转X（弧度）
    double ry;  // 旋转Y（弧度）
    double rz;  // 旋转Z（弧度）
};

// 定义完整的配置结构体（映射整个YAML文件）
struct CameraConfig {
    Pose6D camera;                  // 相机位姿配置
    Pose6D tool;                    // 工具位姿配置
    double y_min;                   // Y轴最小值
    double y_max;                   // Y轴最大值
    double z_min;                   // Z轴最小值
    double z_max;                   // Z轴最大值
    double percentile_low;          // 低分位数阈值
    double percentile_high;         // 高分位数阈值
    int number_points_threshold;    // 点云数量阈值（整型）
    int no_detection_count_threshold; // 无检测计数阈值（整型）
    double y_min_f;                 // Y轴精细最小值
    double y_max_f;                 // Y轴精细最大值
};

/**
 * @brief 相机参数YAML文件读取函数（解析检测配置）
 * @param yaml_file_path YAML文件的绝对路径
 * @return 解析后的完整配置结构体
 * @throw std::runtime_error 文件不存在/字段缺失/类型错误时抛出异常
 */
CameraConfig readCameraConfigFromYaml(const std::string &yaml_file_path);


#endif  // YAML_READER__YAML_READER_HPP_
