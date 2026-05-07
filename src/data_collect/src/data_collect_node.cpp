//
// Created by huang on 2026/2/28.
//
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <cstdlib>
#include <mutex>
#include <atomic>
#include <thread>
#include <exception>
#include <ament_index_cpp/get_package_share_directory.hpp>

// 自定义消息（替换为你的实际消息路径）
//#include "weld_interface/msg/joint_pos.hpp"
#include "weld_interface/msg/tcp_pos.hpp"
//#include "robot_control_msgs/srv/srv_rgbd.hpp"
#include "weld_interface/msg/line_coeffs.hpp"
#include "weld_interface/msg/fanuc_robot_info.hpp"
#include "weld_interface/msg/data_collect_status.hpp"
#include "weld_interface/srv/set_collection_task.hpp"

#include "../../weld_interface/include/weld_interface/topic_configs.h"
#include "../../weld_interface/include/weld_interface/service_configs.h"
#include "file_reader/json.hpp"
#include "file_reader/yaml_reader.h"

#define MAX_SAVE_DATA 50000

namespace fs = std::filesystem;

namespace {

std::string resolve_nodemanage_yaml_path() {
    const char* yaml_path = std::getenv("AUTOCOVER_NODEMANAGE_YAML");
    if (yaml_path != nullptr && yaml_path[0] != '\0') {
        return yaml_path;
    }
    try {
        return ament_index_cpp::get_package_share_directory("data_collect_bringup") + "/config/nodemanage.yaml";
    } catch (const std::exception&) {
        return "config/nodemanage.yaml";
    }
}

std::string build_register_value_group_name(bool has_register_value, int register_value) {
    if (has_register_value) {
        return std::to_string(register_value);
    }
    return "unknown";
}

}

// 工具函数：确保目录存在
void ensure_dir(const std::string& path) {
    if (!fs::exists(path)) {
        fs::create_directories(path);
    }
}

// 获取当前日期（YYYY-MM-DD）
std::string get_current_date() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&now_c);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d");
    return oss.str();
}

// 获取当前时间（HH-MM-SS）
std::string get_current_daytime_str() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&now_c);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%H-%M-%S");
    return oss.str();
}

std::string get_current_datetime_str() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&now_c);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S%z");
    return oss.str();
}

// 获取当前系统时间戳（毫秒）
static int64_t get_current_timestamp_ms() {
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();
    // 转换为微秒精度
    auto now_us = std::chrono::time_point_cast<std::chrono::microseconds>(now);
    // 获取从纪元开始的微秒数
    return now_us.time_since_epoch().count()/1000;
}

// 保存PointCloud2为PLY文件
bool save_ply_file(const sensor_msgs::msg::PointCloud2::SharedPtr& pc2msg, const std::string& savepath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc2msg, *cloud);

    if (cloud->empty()) {
        RCLCPP_WARN(rclcpp::get_logger("data_collect_node"), "[pc2_to_ply] Received empty point cloud");
        return false;
    }

    pcl::PLYWriter writer;
    return writer.write(savepath, *cloud, false) == 0; // false=ASCII格式
}

class DataCollectNode : public rclcpp::Node {
public:
    DataCollectNode() : Node("data_collect_node") {
        std::string yaml_path = resolve_nodemanage_yaml_path();
        RCLCPP_INFO(this->get_logger(), "[DataCollect] Using nodemanage yaml: %s.", yaml_path.c_str());
        ROS2YamlReader reader(yaml_path);
        auto params = reader.readDataCollectNodeParams();

        save_dir_root_ = params.save_dir_root;
        ensure_dir(save_dir_root_);
        RCLCPP_INFO(this->get_logger(), "[DataCollect] Saving data to %s.", save_dir_root_.c_str());

        image_save_interval_ = params.image_save_interval;
        image_log_save_interval_ = params.image_log_save_interval;
        height_log_save_interval_ = params.height_log_save_interval;
        fix_scan_interval_ = params.fix_scan_interval;
        if (image_save_interval_ <= 0) image_save_interval_ = 1;
        if (image_log_save_interval_ <= 0) image_log_save_interval_ = 1;
        if (height_log_save_interval_ <= 0) height_log_save_interval_ = 1;
        if (fix_scan_interval_ <= 0) fix_scan_interval_ = 1;
        auto_save_flag_ = params.auto_save_flag == 0?false:true;
        target_register_index_ = params.target_register_index;
        current_target_register_value_ = 0;
        has_target_register_value_ = false;
        detect_flag1_.store(0);
        detect_flag2_.store(0);
        arc_lost_ticks_.store(0);
        timer_thread_stop_.store(false);
        RCLCPP_INFO(this->get_logger(), "[DataCollect] Auto save data flag : %d.", params.auto_save_flag);
        RCLCPP_INFO(this->get_logger(), "[DataCollect] Target register index : %d.", target_register_index_);

        save_dir_root_ = this->declare_parameter<std::string>("save_dir_root", save_dir_root_);
        image_save_interval_ = this->declare_parameter<int>("image_save_interval", image_save_interval_);
        image_log_save_interval_ = this->declare_parameter<int>("image_log_save_interval", image_log_save_interval_);
        height_log_save_interval_ = this->declare_parameter<int>("height_log_save_interval", height_log_save_interval_);
        fix_scan_interval_ = this->declare_parameter<int>("fix_scan_interval", fix_scan_interval_);
        auto_save_flag_ = this->declare_parameter<int>("auto_save_flag", auto_save_flag_ ? 1 : 0) != 0;
        target_register_index_ = this->declare_parameter<int>("target_register_index", target_register_index_);
        normalize_runtime_settings();

        // 计数器初始化
        reset_counters();

        // 运行模式（默认休眠）
        run_mode_.store(false);

        // CVBridge
        cv_bridge_ptr_ = std::make_shared<cv_bridge::CvImage>();

        // 订阅话题
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
                IMAGE_TOPIC_NAME, 1, std::bind(&DataCollectNode::cb_save_image, this, std::placeholders::_1));

        sub_fix_ply_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                POINT_CLOUD_TOPIC_NAME, 1, std::bind(&DataCollectNode::cb_save_pcd, this, std::placeholders::_1));

        sub_tool_pose_ = this->create_subscription<weld_interface::msg::TcpPos>(
                TCP_PUBLISH_TOPIC_NAME, 1, std::bind(&DataCollectNode::cb_save_tool_pose, this, std::placeholders::_1));

        sub_image_log_ = this->create_subscription<sensor_msgs::msg::Image>(
                SAM_DETECT_IMAGE_TOPIC_NAME, 1, std::bind(&DataCollectNode::cb_save_image_log, this, std::placeholders::_1));

        sub_estimated_line_ = this->create_subscription<weld_interface::msg::LineCoeffs>(
                ESTIMATED_LINE_TOPIC_NAME, 1, std::bind(&DataCollectNode::cb_save_estimated_line, this, std::placeholders::_1));

        sub_height_log_ = this->create_subscription<sensor_msgs::msg::Image>(
                DEBUG_HEIGHT_IMG_TOPIC_NAME, 1, std::bind(&DataCollectNode::cb_save_height_log, this, std::placeholders::_1));

        sub_fanuc_robot_info_ = this->create_subscription<weld_interface::msg::FanucRobotInfo>(
                FANUC_ROBOT_INFO_TOPIC_NAME, 1, std::bind(&DataCollectNode::cb_save_fanuc_robot_info, this, std::placeholders::_1));

        sub_target_register_value_ = this->create_subscription<std_msgs::msg::Int32>(
                FANUC_TARGET_REGISTER_VALUE_TOPIC_NAME, 10, std::bind(&DataCollectNode::cb_target_register_value, this, std::placeholders::_1));

        status_pub_ = this->create_publisher<weld_interface::msg::DataCollectStatus>(
                DATA_COLLECT_STATUS_TOPIC_NAME, rclcpp::QoS(1).transient_local());
        status_timer_ = this->create_wall_timer(
                std::chrono::seconds(1), std::bind(&DataCollectNode::publish_status, this));

        // 注册服务
        srv_mode_activate_ = this->create_service<std_srvs::srv::Empty>(
                DATA_COLLECT_ACTIVATE_SRV_NAME, std::bind(&DataCollectNode::data_collect_activate, this,
                                                   std::placeholders::_1, std::placeholders::_2));

        srv_mode_deactivate_ = this->create_service<std_srvs::srv::Empty>(
                DATA_COLLECT_DEACTIVATE_SRV_NAME, std::bind(&DataCollectNode::data_collect_deactivate, this,
                                                     std::placeholders::_1, std::placeholders::_2));

        srv_set_task_ = this->create_service<weld_interface::srv::SetCollectionTask>(
                DATA_COLLECT_SET_TASK_SRV_NAME, std::bind(&DataCollectNode::set_collection_task,
                                                     this, std::placeholders::_1, std::placeholders::_2));

        parameter_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&DataCollectNode::update_runtime_parameters, this, std::placeholders::_1));

        // timers
        timer_thread_ = std::thread(&DataCollectNode::timerThreadLoop, this);

        RCLCPP_INFO(this->get_logger(), "[DataCollect] init success.");
        publish_status();
    }

    ~DataCollectNode() override {
        timer_thread_stop_.store(true);
        if (timer_thread_.joinable()) {
            timer_thread_.join();
        }
    }

    void enable_data_collect(){
        std::lock_guard<std::mutex> lock(run_mutex_);
        if (run_mode_.load()) {
            return;
        }

        // 创建存储目录
        save_date_ = get_current_date();
        std::string timestamp = get_current_daytime_str();
        std::string register_group_dir = save_dir_root_ + "/" + build_register_value_group_name(
                has_target_register_value_, current_target_register_value_);
        std::string root_dir = register_group_dir + "/" + save_date_ + "/" + timestamp;
        current_save_dir_ = root_dir;
        collection_started_at_ = get_current_datetime_str();
        collection_ended_at_.clear();
        last_error_.clear();
        if (!has_target_register_value_) {
            RCLCPP_WARN(this->get_logger(),
                        "[DataCollect] No target register value received yet for R[%d], using fallback directory 'unknown'.",
                        target_register_index_);
        }
        RCLCPP_INFO(this->get_logger(), "[DataCollect] current save dir %s.", root_dir.c_str());

        // 创建子目录
        save_dir_camera_ = root_dir + "/camera";
        save_dir_camera_log_ = root_dir + "/camera_log";
        save_dir_height_log_ = root_dir + "/height_log";
        save_dir_point_cloud_ = root_dir + "/scan_point_cloud";
        save_dir_robot_state_ = root_dir + "/robot_state";
        save_dir_fanuc_robot_info_ = root_dir + "/fanuc_robot_info";

        save_dir_camera_depth_ = root_dir + "/camera_depth";
        save_dir_camera_depth_log_ = root_dir + "/camera_depth_log";
        save_dir_welding_state_ = root_dir + "/welding_state";
        save_dir_control_cmd_ = root_dir + "/control_cmd";
        save_dir_state_type_ = root_dir + "/state_type";

        std::vector<std::string> folders = {
                save_dir_camera_, save_dir_camera_depth_, save_dir_point_cloud_,
                save_dir_robot_state_, save_dir_welding_state_, save_dir_control_cmd_,
                save_dir_state_type_, save_dir_camera_depth_log_, save_dir_camera_log_,
                save_dir_height_log_, save_dir_fanuc_robot_info_
        };
        for (const auto& folder : folders) {
            ensure_dir(folder);
        }

        // 重置计数器
        reset_counters();
        save_collection_metadata(root_dir);
        write_collection_manifest("running");

        run_mode_.store(true);
        arc_lost_ticks_.store(0);
        RCLCPP_INFO(this->get_logger(), "[DataCollect] Data collection activated. Saving to %s.", root_dir.c_str());
    }

    void normalize_runtime_settings() {
        if (image_save_interval_ <= 0) image_save_interval_ = 1;
        if (image_log_save_interval_ <= 0) image_log_save_interval_ = 1;
        if (height_log_save_interval_ <= 0) height_log_save_interval_ = 1;
        if (fix_scan_interval_ <= 0) fix_scan_interval_ = 1;
        ensure_dir(save_dir_root_);
    }

    rcl_interfaces::msg::SetParametersResult update_runtime_parameters(
            const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        std::lock_guard<std::mutex> lock(run_mutex_);
        for (const auto& parameter : parameters) {
            const std::string& name = parameter.get_name();
            if (name == "save_dir_root") {
                save_dir_root_ = parameter.as_string();
            } else if (name == "image_save_interval") {
                if (parameter.as_int() <= 0) {
                    result.successful = false;
                    result.reason = "image_save_interval must be greater than 0";
                    return result;
                }
                image_save_interval_ = static_cast<int>(parameter.as_int());
            } else if (name == "image_log_save_interval") {
                if (parameter.as_int() <= 0) {
                    result.successful = false;
                    result.reason = "image_log_save_interval must be greater than 0";
                    return result;
                }
                image_log_save_interval_ = static_cast<int>(parameter.as_int());
            } else if (name == "height_log_save_interval") {
                if (parameter.as_int() <= 0) {
                    result.successful = false;
                    result.reason = "height_log_save_interval must be greater than 0";
                    return result;
                }
                height_log_save_interval_ = static_cast<int>(parameter.as_int());
            } else if (name == "fix_scan_interval") {
                if (parameter.as_int() <= 0) {
                    result.successful = false;
                    result.reason = "fix_scan_interval must be greater than 0";
                    return result;
                }
                fix_scan_interval_ = static_cast<int>(parameter.as_int());
            } else if (name == "auto_save_flag") {
                auto_save_flag_ = parameter.as_int() != 0;
            } else if (name == "target_register_index") {
                target_register_index_ = static_cast<int>(parameter.as_int());
                has_target_register_value_ = false;
            }
        }

        normalize_runtime_settings();
        RCLCPP_INFO(this->get_logger(), "[DataCollect] Runtime parameters updated.");
        return result;
    }

    // 激活数据采集
    void data_collect_activate(
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> res) {
        (void)req;
        (void)res;
        enable_data_collect();
    }

    void disable_data_collect(){
        std::lock_guard<std::mutex> lock(run_mutex_);
        if (!run_mode_.load()) {
            return;
        }
        run_mode_.store(false);
        arc_lost_ticks_.store(0);
        collection_ended_at_ = get_current_datetime_str();
        write_collection_manifest("completed");
        RCLCPP_INFO(this->get_logger(), "[DataCollect] Data collection deactivated.");
    }

    // 停用数据采集
    void data_collect_deactivate(
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> res) {
        (void)req;
        (void)res;
        disable_data_collect();
    }

    void set_collection_task(
            const std::shared_ptr<weld_interface::srv::SetCollectionTask::Request> req,
            std::shared_ptr<weld_interface::srv::SetCollectionTask::Response> res) {
        std::lock_guard<std::mutex> lock(run_mutex_);
        task_id_ = req->task_id;
        workpiece_id_ = req->workpiece_id;
        weld_seam_id_ = req->weld_seam_id;
        operator_name_ = req->operator_name;
        shift_ = req->shift;
        notes_ = req->notes;

        if (!current_save_dir_.empty()) {
            write_collection_manifest(run_mode_.load() ? "running" : "completed");
        }

        res->success = true;
        res->message = "collection task updated";
        RCLCPP_INFO(this->get_logger(), "[DataCollect] Collection task updated: task_id=%s, workpiece_id=%s, weld_seam_id=%s.",
                    task_id_.c_str(), workpiece_id_.c_str(), weld_seam_id_.c_str());
    }

    //判断开始采集数据
    void timerCallback(){
        // Fanuc weld_detect may jitter around arc start/end. Do not stop collection
        // on a single inactive sample; otherwise collection repeatedly restarts and
        // the effective save frequency drops sharply.
        const bool arc_active = detect_flag1_.load() != 0 || detect_flag2_.load() != 0;
        if(arc_active){
            arc_lost_ticks_.store(0);
            if(!run_mode_.load()){
                printf("Start data collect!\n");
                enable_data_collect();
            }
            return;
        }else{
            if(run_mode_.load()){
                int arc_lost_ticks = arc_lost_ticks_.fetch_add(1) + 1;
                if(arc_lost_ticks >= ARC_LOST_HOLD_TICKS){
                    printf("Stop data collect!\n");
                    disable_data_collect();
                }
            }
        }
    }

    // 独立线程的循环函数（核心：精准定时+调用回调）
    void timerThreadLoop() {
        // 循环条件：ROS2未退出 + 线程运行标志为true
        while (rclcpp::ok() && !timer_thread_stop_.load()) {
            if(auto_save_flag_== false){
                break;
            }
            // ========== 调用原有timerCallback逻辑 ==========
            this->timerCallback();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        // 线程退出日志
        RCLCPP_INFO(this->get_logger(), "独立定时器线程退出");
    }

    // 重置计数器
    void reset_counters() {
        image_total_counter_ = 0;
        image_log_total_counter_ = 0;
        height_log_total_counter_ = 0;
        fix_scan_total_counter_ = 0;

        image_save_counter_ = 0;
        image_log_save_counter_ = 0;
        height_log_save_counter_ = 0;
        fix_scan_save_counter_ = 0;
        tool_pose_save_counter_ = 0;
        estimated_line_save_counter_ = 0;
        fanuc_info_save_counter_ = 0;
        manifest_update_counter_ = 0;
    }

    void save_collection_metadata(const std::string& root_dir) {
        std::string metadata_path = root_dir + "/collection_meta.txt";
        std::ofstream f(metadata_path, std::ios::out | std::ios::trunc);
        if (!f.is_open()) {
            RCLCPP_WARN(this->get_logger(), "[DataCollect] Failed to write metadata file: %s", metadata_path.c_str());
            return;
        }

        f << "target_register_index=" << target_register_index_ << std::endl;
        if (has_target_register_value_) {
            f << "target_register_value=" << current_target_register_value_ << std::endl;
        } else {
            f << "target_register_value=unknown" << std::endl;
        }
        f << "task_id=" << task_id_ << std::endl;
        f << "workpiece_id=" << workpiece_id_ << std::endl;
        f << "weld_seam_id=" << weld_seam_id_ << std::endl;
        f << "operator_name=" << operator_name_ << std::endl;
        f << "shift=" << shift_ << std::endl;
        f.close();
    }

    void write_collection_manifest(const std::string& status) {
        if (current_save_dir_.empty()) {
            return;
        }

        nlohmann::json manifest;
        manifest["schema_version"] = 1;
        manifest["software"]["name"] = "weld_data_collect";
        manifest["software"]["ros_package"] = "data_collect";
        manifest["status"] = status;
        manifest["started_at"] = collection_started_at_;
        manifest["ended_at"] = collection_ended_at_.empty() ? nlohmann::json(nullptr) : nlohmann::json(collection_ended_at_);
        manifest["save_dir"] = current_save_dir_;
        manifest["save_dir_root"] = save_dir_root_;
        manifest["auto_save"] = auto_save_flag_;
        manifest["task"]["task_id"] = task_id_;
        manifest["task"]["workpiece_id"] = workpiece_id_;
        manifest["task"]["weld_seam_id"] = weld_seam_id_;
        manifest["task"]["operator_name"] = operator_name_;
        manifest["task"]["shift"] = shift_;
        manifest["task"]["notes"] = notes_;
        manifest["target_register"]["index"] = target_register_index_;
        manifest["target_register"]["has_value"] = has_target_register_value_;
        manifest["target_register"]["value"] = has_target_register_value_
                ? nlohmann::json(current_target_register_value_)
                : nlohmann::json(nullptr);
        manifest["sampling"]["image_save_interval"] = image_save_interval_;
        manifest["sampling"]["image_log_save_interval"] = image_log_save_interval_;
        manifest["sampling"]["height_log_save_interval"] = height_log_save_interval_;
        manifest["sampling"]["fix_scan_interval"] = fix_scan_interval_;
        manifest["counts"]["image"] = image_save_counter_;
        manifest["counts"]["image_log"] = image_log_save_counter_;
        manifest["counts"]["height_log"] = height_log_save_counter_;
        manifest["counts"]["point_cloud"] = fix_scan_save_counter_;
        manifest["counts"]["tool_pose"] = tool_pose_save_counter_;
        manifest["counts"]["estimated_line"] = estimated_line_save_counter_;
        manifest["counts"]["fanuc_info"] = fanuc_info_save_counter_;
        manifest["topics"]["image"] = IMAGE_TOPIC_NAME;
        manifest["topics"]["point_cloud"] = POINT_CLOUD_TOPIC_NAME;
        manifest["topics"]["tool_pose"] = TCP_PUBLISH_TOPIC_NAME;
        manifest["topics"]["fanuc_info"] = FANUC_ROBOT_INFO_TOPIC_NAME;
        manifest["last_error"] = last_error_;

        const std::string manifest_path = current_save_dir_ + "/manifest.json";
        std::ofstream f(manifest_path, std::ios::out | std::ios::trunc);
        if (!f.is_open()) {
            last_error_ = "Failed to write manifest: " + manifest_path;
            RCLCPP_WARN(this->get_logger(), "[DataCollect] %s", last_error_.c_str());
            return;
        }
        f << manifest.dump(2) << std::endl;
    }

    void publish_status() {
        if (!status_pub_) {
            return;
        }
        std::lock_guard<std::mutex> lock(run_mutex_);

        weld_interface::msg::DataCollectStatus status;
        status.header.stamp = this->now();
        status.header.frame_id = "data_collect_node";
        status.running = run_mode_.load();
        status.auto_save = auto_save_flag_;
        status.current_save_dir = current_save_dir_;
        status.target_register_index = target_register_index_;
        status.target_register_value = current_target_register_value_;
        status.has_target_register_value = has_target_register_value_;
        status.image_count = image_save_counter_;
        status.image_log_count = image_log_save_counter_;
        status.height_log_count = height_log_save_counter_;
        status.point_cloud_count = fix_scan_save_counter_;
        status.tool_pose_count = tool_pose_save_counter_;
        status.estimated_line_count = estimated_line_save_counter_;
        status.fanuc_info_count = fanuc_info_save_counter_;
        status.task_id = task_id_;
        status.workpiece_id = workpiece_id_;
        status.weld_seam_id = weld_seam_id_;
        status.operator_name = operator_name_;
        status.shift = shift_;
        status.notes = notes_;
        status.last_error = last_error_;
        status_pub_->publish(status);

        if (status.running && ++manifest_update_counter_ >= 5) {
            manifest_update_counter_ = 0;
            write_collection_manifest("running");
        }
    }

    void set_last_error(const std::string& message) {
        last_error_ = message;
        RCLCPP_WARN(this->get_logger(), "[DataCollect] %s", message.c_str());
    }

    // 回调：保存场景图像
    void cb_save_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!run_mode_.load() || image_save_counter_ > MAX_SAVE_DATA) return;

        if (image_total_counter_ % image_save_interval_ == 0) {
            int64_t timestamp = get_current_timestamp_ms();
            //std::string timestamp = std::to_string(msg->header.stamp.sec * 1000000 + msg->header.stamp.nanosec / 1000);
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                std::string image_path = save_dir_camera_ + "/" + std::to_string(timestamp) + ".jpg";
                if (cv::imwrite(image_path, cv_ptr->image)) {
                    image_save_counter_++;
                } else {
                    set_last_error("Failed to write image: " + image_path);
                }
            } catch (const std::exception& e) {
                set_last_error(std::string("Failed to convert image: ") + e.what());
            }
        }
        image_total_counter_++;
    }

    // 回调：保存熔池分割日志图像
    void cb_save_image_log(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!run_mode_.load() || image_log_save_counter_ > MAX_SAVE_DATA) return;
        if (image_log_total_counter_ % image_log_save_interval_ == 0) {
            int64_t timestamp = get_current_timestamp_ms();
            //std::string timestamp = std::to_string(msg->header.stamp.sec * 1000000 + msg->header.stamp.nanosec / 1000);
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                std::string image_path = save_dir_camera_log_ + "/" + std::to_string(timestamp) + ".jpg";
                if (cv::imwrite(image_path, cv_ptr->image)) {
                    image_log_save_counter_++;
                } else {
                    set_last_error("Failed to write image log: " + image_path);
                }
            } catch (const std::exception& e) {
                set_last_error(std::string("Failed to convert image log: ") + e.what());
            }
        }
        image_log_total_counter_++;
    }

    // 回调：保存估计直线参数
    void cb_save_estimated_line(const weld_interface::msg::LineCoeffs::SharedPtr msg) {
        if (!run_mode_.load()) return;
        int64_t timestamp = get_current_timestamp_ms();
        std::string line_log_path = save_dir_camera_ + "/line_log.csv";

        double a = msg->a;
        double b = msg->b;
        double c = msg->c;

        int x0 = (a != 0) ? static_cast<int>(-c / a) : 0;
        int y0 = 0;
        int y1 = 1024;
        int x1 = (a != 0) ? static_cast<int>(-(b * y1 + c) / a) : 0;

        std::ofstream f(line_log_path, std::ios::app);
        if (f.is_open()) {
            f << timestamp << "," << x0 << "," << y0 << "," << x1 << "," << y1 << std::endl;
            f.close();
            estimated_line_save_counter_++;
        } else {
            set_last_error("Failed to write estimated line log: " + line_log_path);
        }
    }

    // 回调：保存高度日志图像
    void cb_save_height_log(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!run_mode_.load() || height_log_save_counter_ > MAX_SAVE_DATA) return;
        if (height_log_total_counter_ % height_log_save_interval_ == 0) {
            int64_t timestamp = get_current_timestamp_ms();
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                std::string image_path = save_dir_height_log_ + "/" + std::to_string(timestamp) + ".jpg";
                if (cv::imwrite(image_path, cv_ptr->image)) {
                    height_log_save_counter_++;
                } else {
                    set_last_error("Failed to write height log: " + image_path);
                }
            } catch (const std::exception& e) {
                set_last_error(std::string("Failed to convert height log: ") + e.what());
            }
        }
        height_log_total_counter_++;
    }

    // 回调：保存线扫点云
    void cb_save_pcd(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!run_mode_.load() || fix_scan_save_counter_ > MAX_SAVE_DATA) return;

        if (fix_scan_total_counter_ % fix_scan_interval_ == 0) {
            int64_t timestamp = get_current_timestamp_ms();
            std::string pcd_path = save_dir_point_cloud_ + "/" + std::to_string(timestamp) + ".ply"; // 修正后缀为PLY
            if (save_ply_file(msg, pcd_path)) {
                fix_scan_save_counter_++;
            } else {
                set_last_error("Failed to write point cloud: " + pcd_path);
            }
        }
        fix_scan_total_counter_++;
    }

    // 回调：保存工具位姿
    void cb_save_tool_pose(const weld_interface::msg::TcpPos::SharedPtr msg) {
        if (!run_mode_.load()) return;

        int64_t timestamp = get_current_timestamp_ms();
        std::string tool_pose_path = save_dir_robot_state_ + "/tool_pose.csv";

        std::ofstream f(tool_pose_path, std::ios::app);
        if (f.is_open()) {
            f << timestamp << ","
              << msg->x << "," << msg->y << "," << msg->z << ","
              << msg->rx << "," << msg->ry << "," << msg->rz << std::endl;
            f.close();
            tool_pose_save_counter_++;
        } else {
            set_last_error("Failed to write tool pose: " + tool_pose_path);
        }
    }

    // 回调：保存速度控制指令
    void save_speed_cmd(const weld_interface::msg::TcpPos::SharedPtr msg) {
        if (!run_mode_.load()) return;

        int64_t timestamp = get_current_timestamp_ms();
        std::string speed_cmd_path = save_dir_control_cmd_ + "/control_speed.csv";

        std::ofstream f(speed_cmd_path, std::ios::app);
        if (f.is_open()) {
            f << timestamp << ","
              << msg->x << "," << msg->y << "," << msg->z << ","
              << msg->rx << "," << msg->ry << "," << msg->rz << std::endl;
            f.close();
        } else {
            set_last_error("Failed to write speed command: " + speed_cmd_path);
        }
    }

    // 回调：保存Fanuc机器人信息
    void cb_save_fanuc_robot_info(const weld_interface::msg::FanucRobotInfo::SharedPtr msg) {
        if(msg != nullptr){
            detect_flag1_.store(msg->weld_detect1);
            detect_flag2_.store(msg->weld_detect2);
        }
        if (!run_mode_.load()) return;
        int64_t timestamp = get_current_timestamp_ms();
        std::string fanuc_info_path = save_dir_fanuc_robot_info_ + "/fanuc_robot_info.csv";
        std::ofstream f(fanuc_info_path, std::ios::app);
        if (f.is_open()) {
            f << timestamp << ","
              << msg->main_pgm << "," << msg->cur_pgm << "," << msg->cur_seq << ","
              << msg->ncstatus << "," << msg->mode << ","
              << msg->voltage1 << "," << msg->current1 << "," << msg->wire_speed1 << "," << msg->weld_detect1 << ","
              << msg->voltage2 << "," << msg->current2 << "," << msg->wire_speed2 << "," << msg->weld_detect2 << ","
              << msg->alarm << "," << msg->alarm_msg << ","
              << msg->emg << "," << msg->override << "," << msg->weld_enable << std::endl;
            f.close();
            fanuc_info_save_counter_++;
        } else {
            set_last_error("Failed to write Fanuc info: " + fanuc_info_path);
        }
    }

    void cb_target_register_value(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg == nullptr) {
            return;
        }
        std::lock_guard<std::mutex> lock(run_mutex_);
        current_target_register_value_ = msg->data;
        has_target_register_value_ = true;
    }

private:
    // 运行模式
    std::atomic_bool run_mode_;

    // 存储目录
    std::string save_dir_root_;
    std::string save_date_;
    std::string current_save_dir_;
    std::string collection_started_at_;
    std::string collection_ended_at_;
    std::string last_error_;
    std::string task_id_;
    std::string workpiece_id_;
    std::string weld_seam_id_;
    std::string operator_name_;
    std::string shift_;
    std::string notes_;
    std::string save_dir_camera_;
    std::string save_dir_camera_log_;
    std::string save_dir_height_log_;
    std::string save_dir_camera_depth_;
    std::string save_dir_camera_depth_log_;
    std::string save_dir_point_cloud_;
    std::string save_dir_robot_state_;
    std::string save_dir_welding_state_;
    std::string save_dir_control_cmd_;
    std::string save_dir_state_type_;
    std::string save_dir_fanuc_robot_info_;

    // 存储间隔
    int image_save_interval_;
    int image_log_save_interval_;
    int height_log_save_interval_;
    int fix_scan_interval_;
    bool auto_save_flag_;
    int target_register_index_;
    int current_target_register_value_;
    bool has_target_register_value_;

    // 计数器
    int image_total_counter_;
    int image_log_total_counter_;
    int height_log_total_counter_;
    int fix_scan_total_counter_;

    int image_save_counter_;
    int image_log_save_counter_;
    int height_log_save_counter_;
    int fix_scan_save_counter_;
    int tool_pose_save_counter_;
    int estimated_line_save_counter_;
    int fanuc_info_save_counter_;
    int manifest_update_counter_;

    // CVBridge
    std::shared_ptr<cv_bridge::CvImage> cv_bridge_ptr_;

    // 订阅器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_fix_ply_;
    rclcpp::Subscription<weld_interface::msg::TcpPos>::SharedPtr sub_tool_pose_;
    rclcpp::Subscription<weld_interface::msg::TcpPos>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_log_;
    rclcpp::Subscription<weld_interface::msg::LineCoeffs>::SharedPtr sub_estimated_line_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_height_log_;
    rclcpp::Subscription<weld_interface::msg::FanucRobotInfo>::SharedPtr sub_fanuc_robot_info_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_target_register_value_;
    rclcpp::Publisher<weld_interface::msg::DataCollectStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // 服务端
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_mode_activate_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_mode_deactivate_;
    rclcpp::Service<weld_interface::srv::SetCollectionTask>::SharedPtr srv_set_task_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    std::thread timer_thread_;
    std::atomic_bool timer_thread_stop_;
    std::mutex run_mutex_;

    std::atomic_int detect_flag1_,detect_flag2_;
    std::atomic_int arc_lost_ticks_;
    static constexpr int ARC_LOST_HOLD_TICKS = 50; // 20 ms * 50 = 1 s

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataCollectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
