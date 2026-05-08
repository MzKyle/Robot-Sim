#include "file_reader/json_reader.h"
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>

int main(int argc, char** argv)
{
    // 初始化ROS2节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("json_reader_test");

    const char* env_path = std::getenv("WELD_JSON_CONFIG_PATH");
    if (env_path == nullptr || env_path[0] == '\0') {
        RCLCPP_WARN(node->get_logger(), "WELD_JSON_CONFIG_PATH is not set, skipping json_reader_test.");
        rclcpp::shutdown();
        return 0;
    }

    std::string config_path = env_path;

    try {
        // 调用通用读取函数
        AddressRootConfig config = read_json_config(config_path);

        // 打印解析结果（验证）
        RCLCPP_INFO(node->get_logger(), "Read %lu AddressManageConfig items", config.AddressManageConfig.size());
        for (const auto& manage_item : config.AddressManageConfig) {
            RCLCPP_INFO(node->get_logger(), "DeviceID: %s", manage_item.DeviceID.c_str());
            RCLCPP_INFO(node->get_logger(), "AddressItemList size: %lu", manage_item.AddressItemList.size());
            for (const auto& addr_item : manage_item.AddressItemList) {
                RCLCPP_INFO(node->get_logger(), "  - Name: %s, Explain: %s", addr_item.Name.c_str(), addr_item.Explain.c_str());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error reading config: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}