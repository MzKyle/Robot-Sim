//
// Created by huang on 2026/1/14.
//

#include <RVC/RVC.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
//#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <pcl/filters/passthrough.h>
// 自定义消息需确保已生成ROS2版本
#include "weld_interface/srv/scan3d.hpp"
#include "weld_interface/msg/tcp_pos.hpp"
#include <std_msgs/msg/float64.hpp>
#include "weld_interface/srv/update_height.hpp"
#include "weld_interface/srv/update_range.hpp"
#include "weld_interface/msg/height.hpp"

#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.h>  // ROS 包: tf2_eigen
#include <cstdlib>
#include <exception>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "weld_interface/topic_configs.h"
#include "weld_interface/service_configs.h"
#include "file_reader/yaml_reader.h"

// 替换ROS1的tf为ROS2的tf2
using tf2::Transform;
using tf2::Quaternion;
using tf2::Matrix3x3;
using tf2::Vector3;

std::mutex g_mutex;
RVC::X2* camera_ptr = NULL;

RVC::X2::CaptureOptions FixScan_opt;
RVC::X2::CaptureOptions SwingLineScan_opt;
Transform base_tcp;

const int img_width = 450;
const int img_height = 450;

double target_height = 0.0;

static pcl::PointCloud<pcl::PointXYZ>::Ptr PointMap2CloudPoint(RVC::PointMap& pm)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->height = pm.GetSize().height;
    cloud->width = pm.GetSize().width;
    cloud->is_dense = false;
    cloud->resize(cloud->height * cloud->width);
    const unsigned int pm_sz = cloud->height * cloud->width;
    const double* pm_data = pm.GetPointDataConstPtr();
    for (int i = 0; i < pm_sz; i++, pm_data += 3)
    {
        cloud->points[i].x = pm_data[0];
        cloud->points[i].y = pm_data[1];
        cloud->points[i].z = pm_data[2];
    }
    return cloud;
}


// ========== 新增：点云从相机坐标系转换到TCP坐标系的函数 ==========
static void transformCloudToTCP(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                                const Transform& tcp_camera) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    const tf2::Matrix3x3& R = tcp_camera.getBasis();
    const tf2::Vector3& p = tcp_camera.getOrigin();

    T.linear() <<
        R[0][0], R[0][1], R[0][2],
        R[1][0], R[1][1], R[1][2],
        R[2][0], R[2][1], R[2][2];

    T.translation() << p.x(), p.y(), p.z();

    Eigen::Matrix4d  matrix = T.matrix().cast<double>();

    pcl::transformPointCloud(*cloud_in, *cloud_out, matrix);  // 输入输出为同一对象

    
}
// ===============================================================

cv::Mat img2mat(RVC::Image img)
{
    cv::Mat mat;
    if (img.IsValid())
    {
        const size_t img_w = img.GetSize().width, img_h = img.GetSize().height;
        bool is_color = img.GetType() == RVC::ImageType::Mono8 ? false : true;

        if (is_color == false)
        {
            mat = cv::Mat(img.GetSize().height, img.GetSize().width, CV_8UC1, img.GetDataPtr());
        }
    }

    return mat;
}

static bool start_fix_scan = false;
double y_min, y_max;
double z_min, z_max, default_z_min, default_z_max;
int number_points_threshold, no_detection_count_threshold;
double percentile_low, percentile_high;
CameraConfig current_camera_config;
std::string current_camera_config_path;

namespace {

std::string resolve_share_path(const std::string& package_name, const std::string& relative_path)
{
    if (relative_path.empty() || relative_path.front() == '/') {
        return relative_path;
    }

    try {
        return ament_index_cpp::get_package_share_directory(package_name) + "/" + relative_path;
    } catch (const std::exception&) {
        return relative_path;
    }
}

const std::string DEFAULT_CAMERA_TCP_CONFIG = resolve_share_path("camera_3d_driver", "config/cameratcp.yaml");
const std::string DEFAULT_NODEMANAGE_YAML = resolve_share_path("data_collect_bringup", "config/nodemanage.yaml");

}  // namespace

static void apply_camera_config(const CameraConfig& config, Transform& tcp_camera)
{
    current_camera_config = config;
    tcp_camera.setOrigin(Vector3(config.camera.x, config.camera.y, config.camera.z));

    Quaternion quat;
    quat.setRPY(config.camera.rx, config.camera.ry, config.camera.rz);
    tcp_camera.setRotation(quat);

    y_min = config.y_min;
    y_max = config.y_max;
    default_z_min = config.z_min;
    z_min = default_z_min;
    default_z_max = config.z_max;
    z_max = default_z_max;
    no_detection_count_threshold = config.no_detection_count_threshold;
    percentile_low = config.percentile_low;
    percentile_high = config.percentile_high;
    number_points_threshold = config.number_points_threshold;

    RCLCPP_INFO(rclcpp::get_logger("camera_driver_3d"), "load camera_3d config: y_min=%f y_max=%f,  z_min=%f z_max=%f", y_min, y_max, z_min, z_max);
    RCLCPP_INFO(rclcpp::get_logger("camera_driver_3d"), "load camera_3d config: number_points_threshold=%d no_detection_count_threshold=%d",
                number_points_threshold, no_detection_count_threshold);
    RCLCPP_INFO(rclcpp::get_logger("camera_driver_3d"), "load camera_3d config: percentile_low=%f percentile_high=%f", percentile_low, percentile_high);
}

static bool load_config_file(const std::string& cfg_file, Transform& tcp_camera)
{
    if (cfg_file.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_driver_3d"), "Camera config path is empty.");
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("camera_driver_3d"), "use config file : %s", cfg_file.c_str());
    try {
        CameraConfig config = readCameraConfigFromYaml(cfg_file);
        current_camera_config_path = cfg_file;
        apply_camera_config(config, tcp_camera);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_driver_3d"), "Failed to read camera config: %s", e.what());
        return false;
    }
    return true;
}

// ROS2服务回调函数（替换ROS1的srv格式）
bool _start_fix_scan(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                     std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("camera_driver_3d"), "start fix scan");
    start_fix_scan = true;
    camera_ptr->StartFixedLineScan(FixScan_opt);
    return true;
}

bool _stop_fix_scan(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    start_fix_scan = false;
    RCLCPP_INFO(rclcpp::get_logger("camera_driver_3d"), "stop fix scan");
    camera_ptr->StopFixedLineScan();
    return true;
}

bool load_config(Transform& tcp_camera)
{
    const char* env_nodemanage_path = std::getenv("AUTOCOVER_NODEMANAGE_YAML");
    const std::string nodemanage_path =
            (env_nodemanage_path != nullptr && env_nodemanage_path[0] != '\0')
            ? env_nodemanage_path
            : DEFAULT_NODEMANAGE_YAML;

    std::string cfg_file = DEFAULT_CAMERA_TCP_CONFIG;

    try {
        ROS2YamlReader reader(nodemanage_path);
        auto camera_3d_params = reader.readCameraDriver3DParams();
        if (!camera_3d_params.cfg.empty()) {
            cfg_file = resolve_share_path("camera_3d_driver", camera_3d_params.cfg);
        }
        RCLCPP_INFO(rclcpp::get_logger("camera_driver_3d"), "Read cfg from nodemanage.yaml: %s", cfg_file.c_str());
    } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("camera_driver_3d"), "Failed to read from nodemanage.yaml: %s, using default: %s", e.what(), cfg_file.c_str());
    }

    return load_config_file(cfg_file, tcp_camera);
}

weld_interface::msg::TcpPos _scan_pose;

// ROS2 3D扫描服务回调
bool call_scan(const std::shared_ptr<weld_interface::srv::Scan3d::Request> req,
               std::shared_ptr<weld_interface::srv::Scan3d::Response> res)
{
    if (start_fix_scan)
    {
        camera_ptr->StopFixedLineScan();
    }

    if (camera_ptr->Capture(SwingLineScan_opt))
    {
        RVC::PointMap pm = camera_ptr->GetPointMap();
        RVC::Image img = camera_ptr->GetImage(RVC::CameraID_Left);
        cv::Mat cv_img = img2mat(img);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = PointMap2CloudPoint(pm);

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = rclcpp::Clock().now(); // ROS2时间戳
        cloud_msg.header.frame_id = "camera";

        if (cv_img.empty() == false)
        {
            cv_bridge::CvImage cv_bridge_image;
            cv_bridge_image.header.stamp = rclcpp::Clock().now();
            cv_bridge_image.header.frame_id = "camera_frame";
            cv_bridge_image.encoding = "mono8";
            cv_bridge_image.image = cv_img;

            sensor_msgs::msg::Image ros_image;
            cv_bridge_image.toImageMsg(ros_image);
            res->image = ros_image;
        }

        res->points = cloud_msg;
        _scan_pose.x = base_tcp.getOrigin().x();
        _scan_pose.y = base_tcp.getOrigin().y();
        _scan_pose.z = base_tcp.getOrigin().z();

        double roll, pitch, yaw;
        Matrix3x3(base_tcp.getRotation()).getRPY(roll, pitch, yaw);
        _scan_pose.rx = roll;
        _scan_pose.ry = pitch;
        _scan_pose.rz = yaw;
    }

    if (start_fix_scan)
    {
        camera_ptr->StartFixedLineScan(FixScan_opt);
    }

    return true;
}

pcl::PointCloud<pcl::PointXYZRGB> generateChooseColoredCloud(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;

    colored_cloud.width = cloud.width;
    colored_cloud.height = cloud.height;
    colored_cloud.is_dense = cloud.is_dense;
    colored_cloud.points.resize(cloud.points.size());

    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        colored_cloud.points[i].x = cloud.points[i].x;
        colored_cloud.points[i].y = cloud.points[i].y;
        colored_cloud.points[i].z = cloud.points[i].z;
        colored_cloud.points[i].r = 255;
    }
    return colored_cloud;
}

static pcl::PassThrough<pcl::PointXYZ> pass;

bool filter_point(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double value_min, double value_max)
{
    pass.setInputCloud(source);
    pass.setFilterFieldName(name);
    pass.setFilterLimits(value_min, value_max);
    pass.setFilterLimitsNegative(false);
    pass.filter(*out);

    return true;
}

// ROS2订阅回调函数
void jog_callback(const weld_interface::msg::TcpPos::SharedPtr msg)
{
    // 米和弧度
    base_tcp.setOrigin(Vector3(msg->x, msg->y, msg->z));

    Quaternion quat;
    quat.setRPY(msg->rx, msg->ry, msg->rz);
    base_tcp.setRotation(quat);
}

// 更新目标高度服务回调（ROS2）
bool updateTargetHeightCallback(const std::shared_ptr<weld_interface::srv::UpdateHeight::Request> req,
                                std::shared_ptr<weld_interface::srv::UpdateHeight::Response> res)
{
    target_height = req->height;
    RCLCPP_INFO(rclcpp::get_logger("camera_driver_3d"), "Successfully updated target height of camera 3d node: %f", target_height);
    return true;
}

void drawDebugHeight(double estimated_height, double target_height, pcl::PointCloud<pcl::PointXYZ>& cloud, cv::Mat& out_img)
{
    auto height_to_pixel = [&](double z) -> int {
        double ratio = (z_max - z) / (z_max - z_min);
        int pixel = static_cast<int>(ratio * img_height);
        if (pixel < 0) pixel = 0;
        if (pixel >= img_height) pixel = img_height - 1;
        return pixel;
    };

    auto width_to_pixel = [&](double y) -> int {
        double ratio = (y_max - y) / (y_max - y_min);
        int pixel = static_cast<int>(ratio * img_width);
        if (pixel < 0) pixel = 0;
        if (pixel >= img_width) pixel = img_width - 1;
        return pixel;
    };

    for (const auto& pt : cloud.points) {
        if (std::isnan(pt.y) || std::isnan(pt.z)) continue;

        int x_pixel = width_to_pixel(pt.y);
        int y_pixel = height_to_pixel(pt.z);
        cv::circle(out_img, cv::Point(x_pixel, y_pixel), 2, cv::Scalar(0, 255, 0), -1);
    }

    int target_y = height_to_pixel(-target_height);
    cv::line(out_img, cv::Point(0, target_y), cv::Point(img_width - 1, target_y), cv::Scalar(0, 0, 255), 2);
    cv::putText(out_img, "target_height", cv::Point(10, target_y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

    int estimated_y = height_to_pixel(-estimated_height);
    cv::line(out_img, cv::Point(0, estimated_y), cv::Point(img_width - 1, estimated_y), cv::Scalar(255, 0, 0), 2);
    cv::putText(out_img, "estimated_height", cv::Point(10, estimated_y + 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

    char target_text[64], estimated_text[64];
    snprintf(target_text, sizeof(target_text), "target: %.2f mm", target_height * 1000.0);
    snprintf(estimated_text, sizeof(estimated_text), "estimated: %.2f mm", estimated_height * 1000.0);
    cv::putText(out_img, target_text, cv::Point(200, target_y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    cv::putText(out_img, estimated_text, cv::Point(200, estimated_y + 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
}

void filter_by_percentile(std::vector<double>& in_values, std::vector<double>& out_values, double percentile_low, double percentile_high)
{
    if (percentile_low >= percentile_high) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_driver_3d"), "Please make sure percentile_low < percentile_high: %f, %f", percentile_low, percentile_high);
        return;
    }
    if (percentile_low < 0.0 || percentile_low > 1.0 || percentile_high < 0.0 || percentile_high > 1.0) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_driver_3d"), "Please make sure percentile_low and percentile_high are between 0.0 and 1.0: %f, %f", percentile_low, percentile_high);
        return;
    }

    if (in_values.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_driver_3d"), "Please make sure in_values is not empty");
        return;
    }

    out_values.clear();

    std::sort(in_values.begin(), in_values.end());
    size_t n = in_values.size();
    size_t index_low = static_cast<size_t>(n * percentile_low);
    size_t index_high = static_cast<size_t>(n * percentile_high);

    if (index_low >= n) index_low = n - 1;
    if (index_high >= n) index_high = n - 1;
    double value_low = in_values[index_low];
    double value_high = in_values[index_high];

    for (const auto& v : in_values) {
        if (v >= value_low && v <= value_high) {
            out_values.push_back(v);
        }
    }
}

// 更新Z范围服务回调（ROS2）
bool updateCroppingZRangeCallback(const std::shared_ptr<weld_interface::srv::UpdateRange::Request> req,
                                  std::shared_ptr<weld_interface::srv::UpdateRange::Response> res)
{
    z_min = default_z_min + req->delta;
    z_max = default_z_max + req->delta;
    RCLCPP_INFO(rclcpp::get_logger("camera_driver_3d"), "Successfully updated cropping z range: (%f, %f)", z_min, z_max);
    return true;
}

bool reloadCamera3DConfigCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> res,
                                  Transform* tcp_camera)
{
    (void)req;
    if (tcp_camera == nullptr) {
        res->success = false;
        res->message = "tcp_camera is null";
        return true;
    }

    const bool ok = current_camera_config_path.empty()
            ? load_config(*tcp_camera)
            : load_config_file(current_camera_config_path, *tcp_camera);
    if (!ok) {
        res->success = false;
        res->message = "failed to reload cameratcp.yaml";
        return true;
    }

    res->success = true;
    res->message = "camera 3d config reloaded";
    RCLCPP_INFO(rclcpp::get_logger("camera_driver_3d"), "Reloaded camera 3d config from yaml.");
    return true;
}

static void declare_camera_config_parameters(const rclcpp::Node::SharedPtr& node, const CameraConfig& config)
{
    node->declare_parameter<double>("camera.x", config.camera.x);
    node->declare_parameter<double>("camera.y", config.camera.y);
    node->declare_parameter<double>("camera.z", config.camera.z);
    node->declare_parameter<double>("camera.rx", config.camera.rx);
    node->declare_parameter<double>("camera.ry", config.camera.ry);
    node->declare_parameter<double>("camera.rz", config.camera.rz);
    node->declare_parameter<double>("tool.x", config.tool.x);
    node->declare_parameter<double>("tool.y", config.tool.y);
    node->declare_parameter<double>("tool.z", config.tool.z);
    node->declare_parameter<double>("tool.rx", config.tool.rx);
    node->declare_parameter<double>("tool.ry", config.tool.ry);
    node->declare_parameter<double>("tool.rz", config.tool.rz);
    node->declare_parameter<double>("y_min", config.y_min);
    node->declare_parameter<double>("y_max", config.y_max);
    node->declare_parameter<double>("z_min", config.z_min);
    node->declare_parameter<double>("z_max", config.z_max);
    node->declare_parameter<double>("y_min_f", config.y_min_f);
    node->declare_parameter<double>("y_max_f", config.y_max_f);
    node->declare_parameter<double>("percentile_low", config.percentile_low);
    node->declare_parameter<double>("percentile_high", config.percentile_high);
    node->declare_parameter<int>("number_points_threshold", config.number_points_threshold);
    node->declare_parameter<int>("no_detection_count_threshold", config.no_detection_count_threshold);
}

static rcl_interfaces::msg::SetParametersResult update_camera_runtime_parameters(
        const std::vector<rclcpp::Parameter>& parameters,
        Transform* tcp_camera,
        bool* publish_tf)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    if (tcp_camera == nullptr || publish_tf == nullptr) {
        result.successful = false;
        result.reason = "camera runtime pointer is null";
        return result;
    }

    CameraConfig next_config = current_camera_config;
    for (const auto& parameter : parameters) {
        const std::string& name = parameter.get_name();
        if (name == "cfg") {
            if (!load_config_file(parameter.as_string(), *tcp_camera)) {
                result.successful = false;
                result.reason = "failed to load cfg";
                return result;
            }
            next_config = current_camera_config;
        } else if (name == "publish_tf") {
            *publish_tf = parameter.as_bool();
        } else if (name == "camera.x") {
            next_config.camera.x = parameter.as_double();
        } else if (name == "camera.y") {
            next_config.camera.y = parameter.as_double();
        } else if (name == "camera.z") {
            next_config.camera.z = parameter.as_double();
        } else if (name == "camera.rx") {
            next_config.camera.rx = parameter.as_double();
        } else if (name == "camera.ry") {
            next_config.camera.ry = parameter.as_double();
        } else if (name == "camera.rz") {
            next_config.camera.rz = parameter.as_double();
        } else if (name == "tool.x") {
            next_config.tool.x = parameter.as_double();
        } else if (name == "tool.y") {
            next_config.tool.y = parameter.as_double();
        } else if (name == "tool.z") {
            next_config.tool.z = parameter.as_double();
        } else if (name == "tool.rx") {
            next_config.tool.rx = parameter.as_double();
        } else if (name == "tool.ry") {
            next_config.tool.ry = parameter.as_double();
        } else if (name == "tool.rz") {
            next_config.tool.rz = parameter.as_double();
        } else if (name == "y_min") {
            next_config.y_min = parameter.as_double();
        } else if (name == "y_max") {
            next_config.y_max = parameter.as_double();
        } else if (name == "z_min") {
            next_config.z_min = parameter.as_double();
        } else if (name == "z_max") {
            next_config.z_max = parameter.as_double();
        } else if (name == "y_min_f") {
            next_config.y_min_f = parameter.as_double();
        } else if (name == "y_max_f") {
            next_config.y_max_f = parameter.as_double();
        } else if (name == "percentile_low") {
            next_config.percentile_low = parameter.as_double();
        } else if (name == "percentile_high") {
            next_config.percentile_high = parameter.as_double();
        } else if (name == "number_points_threshold") {
            next_config.number_points_threshold = static_cast<int>(parameter.as_int());
        } else if (name == "no_detection_count_threshold") {
            next_config.no_detection_count_threshold = static_cast<int>(parameter.as_int());
        }
    }

    if (next_config.y_min >= next_config.y_max) {
        result.successful = false;
        result.reason = "y_min must be less than y_max";
        return result;
    }
    if (next_config.z_min >= next_config.z_max) {
        result.successful = false;
        result.reason = "z_min must be less than z_max";
        return result;
    }
    if (next_config.percentile_low < 0.0 || next_config.percentile_high > 1.0 ||
        next_config.percentile_low >= next_config.percentile_high) {
        result.successful = false;
        result.reason = "percentile range must satisfy 0 <= low < high <= 1";
        return result;
    }

    apply_camera_config(next_config, *tcp_camera);
    RCLCPP_INFO(rclcpp::get_logger("camera_driver_3d"), "Runtime camera 3d parameters updated.");
    return result;
}

int main(int argc, char* argv[])
{
    // ROS2节点初始化
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("camera_driver_3d");

    // Initialize RVC X system.
    RVC::SystemInit();

    // Scan all USB RVC X Camera devices.
    RVC::Device devices[10];
    size_t actual_size = 0;
    SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::All);

    // Find whether any RVC X Camera is connected or not.
    if (actual_size == 0)
    {
        std::cout << "Can not find any RVC X Camera!" << std::endl;
        return -1;
    }

    RVC::DeviceInfo info;
    devices[0].GetDeviceInfo(&info);
    if (info.support_x2 == false)
    {
        std::cout << "The camera does not support the x2 function!" << std::endl;
        RVC::SystemShutdown();
        return -1;
    }

    if ((info.support_capture_mode & RVC::CaptureMode_SwingLineScan) == false)
    {
        std::cout << "The camera does not support SwingLineScan Mode!" << std::endl;
        RVC::SystemShutdown();
        return -1;
    }


    // Create a RVC X Camera and choose use left side camera.

    RVC::X2 x2 = RVC::X2::Create(devices[0]);

    // 固定线扫配置
    FixScan_opt.capture_mode = RVC::CaptureMode_FixedLineScan;
    FixScan_opt.line_scanner_exposure_time_us = 300;
    FixScan_opt.projector_brightness = 100;
    FixScan_opt.gain_3d = 0;
    FixScan_opt.line_scanner_laser_position = 65536 / 2;
    FixScan_opt.line_scanner_min_distance = 400;
    FixScan_opt.line_scanner_max_distance = 1000;

    // 非固定线扫配置
    SwingLineScan_opt.capture_mode = RVC::CaptureMode_SwingLineScan;
    SwingLineScan_opt.line_scanner_scan_time_ms = 2500;
    SwingLineScan_opt.line_scanner_exposure_time_us = 300;
    SwingLineScan_opt.line_scanner_min_distance = 400;
    SwingLineScan_opt.line_scanner_max_distance = 800;
    SwingLineScan_opt.correspond2d = true;
    SwingLineScan_opt.exposure_time_2d = 12;
    SwingLineScan_opt.gain_2d = 1.0;
    SwingLineScan_opt.gamma_2d = 0.53;

    x2.Open();
    camera_ptr = &x2;

    // Test RVC X Camera is opened or not.
    if (!x2.IsOpen())
    {
        std::cout << "RVC X Camera is not opened!" << std::endl;
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return 1;
    }

    std::cout << "start " << std::endl;

    // ROS2频率控制（30Hz）
    rclcpp::Rate loop_rate(30);
    auto points_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(FIXED_SCAN_TOPIC_NAME, 1);
    //auto estimated_height_pub = node->create_publisher<weld_interface::msg::Height>(ESTIMATED_HEIGHT_TOPIC_NAME, 1);
    auto debug_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(FIXED_SCAN_ALL_TOPIC_NAME, 1);
    auto debug_height_pub = node->create_publisher<sensor_msgs::msg::Image>(DEBUG_HEIGHT_IMG_TOPIC_NAME, 1);
    auto pose_pub = node->create_publisher<weld_interface::msg::TcpPos>(SCAN_POSE_TOPIC_NAME, 1);

    // ========== 新增：TCP坐标系原始点云发布器 ==========
    auto tcp_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(POINT_CLOUD_TOPIC_NAME, 1);
    // ===================================================

    // ROS2订阅者创建
    auto sub_jog = node->create_subscription<weld_interface::msg::TcpPos>(
            TCP_PUBLISH_TOPIC_NAME, 1, std::bind(&jog_callback, std::placeholders::_1));

    // ROS2服务创建
    auto start_scan_srv = node->create_service<std_srvs::srv::Empty>(
            START_FIX_SCAN_SRV_NAME, std::bind(&_start_fix_scan, std::placeholders::_1, std::placeholders::_2));
    auto stop_scan_srv = node->create_service<std_srvs::srv::Empty>(
            STOP_FIX_SCAN_SRV_NAME, std::bind(&_stop_fix_scan, std::placeholders::_1, std::placeholders::_2));
    auto scan_3d_srv = node->create_service<weld_interface::srv::Scan3d>(
            SCAN_3D_SRV_NAME, std::bind(&call_scan, std::placeholders::_1, std::placeholders::_2));
    auto update_target_height_srv = node->create_service<weld_interface::srv::UpdateHeight>(
            UPDATE_CAMERA_3D_NODE_TARGET_HEIGHT_SRV_NAME, std::bind(&updateTargetHeightCallback, std::placeholders::_1, std::placeholders::_2));
    Transform tcp_camera;
    if (!load_config(tcp_camera)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load initial camera 3d config.");
        x2.Close();
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        rclcpp::shutdown();
        return 1;
    }
    node->declare_parameter<std::string>("cfg", current_camera_config_path);
    declare_camera_config_parameters(node, current_camera_config);

    auto update_cropping_z_range_srv = node->create_service<weld_interface::srv::UpdateRange>(
            UPDATE_CAMERA_3D_NODE_CROPPING_Z_RANGE_SRV_NAME, std::bind(&updateCroppingZRangeCallback, std::placeholders::_1, std::placeholders::_2));
    auto reload_config_srv = node->create_service<std_srvs::srv::Trigger>(
            RELOAD_CAMERA_3D_CONFIG_SRV_NAME,
            std::bind(&reloadCamera3DConfigCallback, std::placeholders::_1, std::placeholders::_2, &tcp_camera));

    // ROS2 TF广播器
    tf2_ros::TransformBroadcaster tf_broadcaster(node);

    // 参数获取（ROS2）
    bool publish_tf = node->declare_parameter<bool>("publish_tf", true);
    auto parameter_callback_handle = node->add_on_set_parameters_callback(
            std::bind(&update_camera_runtime_parameters, std::placeholders::_1, &tcp_camera, &publish_tf));
    (void)parameter_callback_handle;

    pcl::PointCloud<pcl::PointXYZ>::Ptr base_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    cv_bridge::CvImage cv_bridge_image;
    bool no_detection_count = 0;

    while (rclcpp::ok())
    {
        RVC::PointMap pm;
        if (start_fix_scan)
        {
            if (x2.GetFixedLineScanPointMap(pm))
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera = PointMap2CloudPoint(pm); // 相机坐标系原始点云

                // ========== 核心修改：转换为TCP坐标系原始点云并发布 ==========
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tcp_raw(new pcl::PointCloud<pcl::PointXYZ>);
                transformCloudToTCP(cloud_camera, cloud_tcp_raw, tcp_camera); // 坐标变换

                // 发布TCP坐标系原始点云（无裁剪）
                sensor_msgs::msg::PointCloud2 tcp_cloud_msg;
                pcl::toROSMsg(*cloud_tcp_raw, tcp_cloud_msg);
                tcp_cloud_msg.header.stamp = node->get_clock()->now();
                tcp_cloud_msg.header.frame_id = "tcp"; // 明确标注为TCP坐标系
                tcp_cloud_pub->publish(tcp_cloud_msg);
            }
        }

        // ROS2 TF变换发布
        if (publish_tf)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = node->get_clock()->now();
            t.header.frame_id = "tcp";
            t.child_frame_id = "camera";

            t.transform.translation.x = tcp_camera.getOrigin().x();
            t.transform.translation.y = tcp_camera.getOrigin().y();
            t.transform.translation.z = tcp_camera.getOrigin().z();

            Quaternion q = tcp_camera.getRotation();
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_broadcaster.sendTransform(t);
        }
        pose_pub->publish(_scan_pose);

        loop_rate.sleep();
        rclcpp::spin_some(node);
    }

    x2.StopFixedLineScan();
    x2.Close();
    RVC::X2::Destroy(x2);
    RVC::SystemShutdown();

    rclcpp::shutdown();
    return 0;
}
