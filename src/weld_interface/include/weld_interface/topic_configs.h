//
// Created by huang on 2026/2/27.
//

#ifndef ROS2_MAIN_CONFIGS_H
#define ROS2_MAIN_CONFIGS_H

//TOPICS定义：
//auto_cover topics
#define TCP_PUBLISH_TOPIC_NAME "/tool_pos" //机器人发布TOPIC TcpPos.msg
#define ROTOB_COMMAND_TOPIC_NAME "/robot_command" //用于向机器人发送控制指令 RobotCommand.msg

#define ESTIMATED_LINE_TOPIC_NAME "/estimated_line" //估计焊缝线系数订阅：接收视觉系统估计的焊缝线位置 LineCoeffs.msg
#define ESTIMATED_HEIGHT_TOPIC_NAME "/estimated_height" // 估计焊缝高度订阅：接收视觉系统估计的焊缝高度信息 Height.msg
#define ESTIMATED_LIFT_DROP_TOPIC_NAME "/estimated_lift_drop" //订阅焊缝隆起/下降信息 LiftDrop.msg
#define IMAGE_TOPIC_NAME "/image_topic" //订阅2D相机图像话题（触发焊缝分割） sensor_msgs::msg::Image
#define POINT_CLOUD_TOPIC_NAME "/tcp_cloud_raw" //订阅3D点云话题（触发高度估计） sensor_msgs::msg::PointCloud2

//camera topics
#define FIXED_SCAN_TOPIC_NAME "/fixed_scan" // 发布经过处理后的固定扫描点云数据（通常是筛选后的有效点云） sensor_msgs::msg::PointCloud2
#define FIXED_SCAN_ALL_TOPIC_NAME "/fixed_scan_all" // 发布完整的原始扫描点云数据，用于调试和可视化分析 sensor_msgs::msg::PointCloud2
#define DEBUG_HEIGHT_IMG_TOPIC_NAME "/debug_height_img" // 创建调试高度图像发布器 sensor_msgs::msg::Image
#define SCAN_POSE_TOPIC_NAME "/scan_pose" // 发布扫描时的机器人TCP位置和姿态 TcpPos.msg

//fanuc_robot topics
#define FANUC_ROBOT_INFO_TOPIC_NAME "/fanuc_robot_info" //机器人信息 FanucRobotInfo.msg
#define FANUC_TARGET_REGISTER_VALUE_TOPIC_NAME "/fanuc_target_register_value" //目标寄存器当前值 std_msgs::msg::Int32
#define FANUC_WELD_REGISTER_INFO_TOPIC_NAME "/fanuc_weld_register_info" //焊接寄存器信息 weld_interface::msg::WeldRegisterInfo
#define DATA_COLLECT_STATUS_TOPIC_NAME "/data_collect_status" //数据采集状态 weld_interface::msg::DataCollectStatus
#define DATA_COLLECT_QUALITY_TOPIC_NAME "/data_collect_quality" //采集质量评估状态 weld_interface::msg::CollectionQuality

//ros_bridge topics
#define SELECT_POINTCLOUD_TOPIC_NAME "/select_pointcloud" // 订阅高度图像 sensor_msgs::msg::PointCloud2
#define SAM_DETECT_IMAGE_TOPIC_NAME "/sam_detect_image" //订阅分割图像 sensor_msgs::msg::Image

#endif //ROS2_MAIN_CONFIGS_H
