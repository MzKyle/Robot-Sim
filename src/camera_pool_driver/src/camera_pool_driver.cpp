#include "CameraApi.h" //相机SDK的API头文件

#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
// ROS2核心头文件
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/image.hpp"
// ROS2的cv_bridge头文件
#include "cv_bridge/cv_bridge.h"

#include "weld_interface/topic_configs.h"
#include "weld_interface/service_configs.h"

using namespace cv;

unsigned char *g_pRgbBuffer; // 处理后数据缓存区

struct Camera2DSettings
{
    int trigger_mode{2};
    int strobe_polarity{0};
    int saturation{64};
    int gamma{106};
    double exposure_time{4.3};
    int analog_gain{64};
    double frame_rate{60.0};
};

static void log_camera_status(const rclcpp::Logger& logger, const char* name, int status)
{
    if (status != CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_WARN(logger, "%s failed, status=%d", name, status);
    }
}

static Camera2DSettings declare_camera_settings(const rclcpp::Node::SharedPtr& node)
{
    Camera2DSettings settings;
    settings.trigger_mode = node->declare_parameter<int>("trigger_mode", settings.trigger_mode);
    settings.strobe_polarity = node->declare_parameter<int>("strobe_polarity", settings.strobe_polarity);
    settings.saturation = node->declare_parameter<int>("saturation", settings.saturation);
    settings.gamma = node->declare_parameter<int>("gamma", settings.gamma);
    settings.exposure_time = node->declare_parameter<double>("exposure_time", settings.exposure_time);
    settings.analog_gain = node->declare_parameter<int>("analog_gain", settings.analog_gain);
    settings.frame_rate = node->declare_parameter<double>("frame_rate", settings.frame_rate);
    if (settings.frame_rate <= 0.0)
    {
        settings.frame_rate = 60.0;
    }
    return settings;
}

static void apply_camera_settings(int hCamera, const rclcpp::Logger& logger, const Camera2DSettings& settings)
{
    log_camera_status(logger, "CameraSetTriggerMode", CameraSetTriggerMode(hCamera, settings.trigger_mode));
    log_camera_status(logger, "CameraSetStrobePolarity", CameraSetStrobePolarity(hCamera, settings.strobe_polarity));
    log_camera_status(logger, "CameraSetSaturation", CameraSetSaturation(hCamera, settings.saturation));
    log_camera_status(logger, "CameraSetGamma", CameraSetGamma(hCamera, settings.gamma));
    log_camera_status(logger, "CameraSetExposureTime", CameraSetExposureTime(hCamera, settings.exposure_time));
    log_camera_status(logger, "CameraSetAnalogGain", CameraSetAnalogGain(hCamera, settings.analog_gain));
    RCLCPP_INFO(
        logger,
        "2D camera settings: trigger_mode=%d strobe_polarity=%d saturation=%d gamma=%d exposure_time=%.3f analog_gain=%d frame_rate=%.2f",
        settings.trigger_mode,
        settings.strobe_polarity,
        settings.saturation,
        settings.gamma,
        settings.exposure_time,
        settings.analog_gain,
        settings.frame_rate);
}

int main(int argc, char *argv[])
{
    int iCameraCounts = 1;
    int iStatus = -1;
    tSdkCameraDevInfo tCameraEnumList;
    int hCamera;
    tSdkCameraCapbility tCapability; // 设备描述信息
    tSdkFrameHead sFrameInfo;
    BYTE *pbyBuffer;

    // ========== ROS2 初始化 ==========
    rclcpp::init(argc, argv);
    // 创建ROS2节点
    auto node = rclcpp::Node::make_shared("camera_node");

    // ========== 相机初始化逻辑（保持不变） ==========
    CameraSdkInit(1);

    // 枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("state = %d\n", iStatus);

    printf("count = %d\n", iCameraCounts);
    // 没有连接设备
    if (iCameraCounts == 0)
    {
        return -1;
    }

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

    // 初始化失败
    printf("state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        return -1;
    }

    // 获得相机的特性描述结构体
    CameraGetCapability(hCamera, &tCapability);

    // 分配图像缓存区
    g_pRgbBuffer = (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);

    // 启动相机采集
    CameraPlay(hCamera);

    // 图像格式设置
    if (tCapability.sIspCapacity.bMonoSensor)
    {
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    }
    else
    {
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }

    Camera2DSettings camera_settings = declare_camera_settings(node);
    apply_camera_settings(hCamera, node->get_logger(), camera_settings);

    auto parameter_callback_handle = node->add_on_set_parameters_callback(
        [&](const std::vector<rclcpp::Parameter>& parameters) {
            Camera2DSettings next_settings = camera_settings;
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto& parameter : parameters)
            {
                const std::string& name = parameter.get_name();
                if (name == "trigger_mode")
                {
                    next_settings.trigger_mode = parameter.as_int();
                }
                else if (name == "strobe_polarity")
                {
                    next_settings.strobe_polarity = parameter.as_int();
                }
                else if (name == "saturation")
                {
                    next_settings.saturation = parameter.as_int();
                }
                else if (name == "gamma")
                {
                    next_settings.gamma = parameter.as_int();
                }
                else if (name == "exposure_time")
                {
                    next_settings.exposure_time = parameter.as_double();
                }
                else if (name == "analog_gain")
                {
                    next_settings.analog_gain = parameter.as_int();
                }
                else if (name == "frame_rate")
                {
                    next_settings.frame_rate = parameter.as_double();
                    if (next_settings.frame_rate <= 0.0)
                    {
                        result.successful = false;
                        result.reason = "frame_rate must be greater than 0";
                        return result;
                    }
                }
            }

            camera_settings = next_settings;
            apply_camera_settings(hCamera, node->get_logger(), camera_settings);
            return result;
        });
    (void)parameter_callback_handle;

    // ROS2日志输出
    RCLCPP_INFO(node->get_logger(), "width:%d height:%d",
                tCapability.sResolutionRange.iWidthMax,
                tCapability.sResolutionRange.iHeightMax);

    // ========== ROS2 发布者创建 ==========
    // 创建图像发布者，队列大小设为1
    auto image_pub = node->create_publisher<sensor_msgs::msg::Image>(IMAGE_TOPIC_NAME, 1);

    // ========== 主循环 ==========
    while (rclcpp::ok())
    {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
        {
            if(CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo) == CAMERA_STATUS_SUCCESS){
                // 创建OpenCV图像矩阵
                cv::Mat matImage(
                        cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
                        sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                        g_pRgbBuffer);

                // ========== ROS2 CV-Bridge 转换 ==========
                cv_bridge::CvImage cv_bridge_image;
                // 设置时间戳（ROS2使用内置的时间接口）
                cv_bridge_image.header.stamp = node->get_clock()->now();
                cv_bridge_image.header.frame_id = "camera_frame";
                // 根据图像类型设置编码格式
                cv_bridge_image.encoding = (sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8) ? "mono8" : "bgr8";
                cv_bridge_image.image = matImage;

                // 将CV图像转换为ROS2图像消息并发布
                sensor_msgs::msg::Image ros_image;
                cv_bridge_image.toImageMsg(ros_image);
                image_pub->publish(ros_image);

                // 释放图像缓冲区
                CameraReleaseImageBuffer(hCamera, pbyBuffer);
            }
            else{
                printf("CameraImageProcess exe failed!\n");
            }
        }

        // ROS2自旋（处理回调，这里无回调但建议保留）
        rclcpp::spin_some(node);
        // 频率控制
        rclcpp::Rate loop_rate(camera_settings.frame_rate);
        loop_rate.sleep();
    }

    // ========== 资源释放 ==========
    CameraUnInit(hCamera);
    free(g_pRgbBuffer);

    // ROS2退出
    rclcpp::shutdown();

    return 0;
}
