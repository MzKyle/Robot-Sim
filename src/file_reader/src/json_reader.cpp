#include "file_reader/json_reader.h"

#include <fstream>
#include <rclcpp/rclcpp.hpp>

// 实现AddressItem的反序列化
void from_json(const json& j, AddressItem& item)
{
    // 严格解析每个字段，若字段缺失会抛出异常
    j.at("Id").get_to(item.Id);
    j.at("Address").get_to(item.Address);
    j.at("StartPosition").get_to(item.StartPosition);
    j.at("AddressLen").get_to(item.AddressLen);
    j.at("RegisterType").get_to(item.RegisterType);
    j.at("BlockNumber").get_to(item.BlockNumber);
    j.at("BlockType").get_to(item.BlockType);
    j.at("Explain").get_to(item.Explain);
    j.at("Name").get_to(item.Name);
    j.at("ConfigTime").get_to(item.ConfigTime);
    j.at("Offset").get_to(item.Offset);
    j.at("Authority").get_to(item.Authority);
    j.at("Unit").get_to(item.Unit);
    j.at("ValueType").get_to(item.ValueType);
    j.at("ExchangeByte").get_to(item.ExchangeByte);
    j.at("TransformType").get_to(item.TransformType);
    j.at("Express").get_to(item.Express);
    j.at("ReadMin").get_to(item.ReadMin);
    j.at("ReadMax").get_to(item.ReadMax);
    j.at("TransformMin").get_to(item.TransformMin);
    j.at("TransformMax").get_to(item.TransformMax);
    j.at("Precision").get_to(item.Precision);
    j.at("PrecisionType").get_to(item.PrecisionType);
    j.at("ScanInterval").get_to(item.ScanInterval);
    j.at("CollectionType").get_to(item.CollectionType);
    j.at("IsSaveLastValue").get_to(item.IsSaveLastValue);
}

// 实现AddressManageConfigItem的反序列化
void from_json(const json& j, AddressManageConfigItem& item)
{
    j.at("DeviceID").get_to(item.DeviceID);
    j.at("AddressItemList").get_to(item.AddressItemList);
}

// 实现AddressRootConfig的反序列化
void from_json(const json& j, AddressRootConfig& config)
{
    j.at("AddressManageConfig").get_to(config.AddressManageConfig);
}

// 通用读取函数实现
AddressRootConfig read_json_config(const std::string& file_path)
{
    // 1. 打开文件
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open JSON config file: " + file_path);
    }

    // 2. 解析JSON
    json json_data;
    try {
        file >> json_data;
    } catch (const json::parse_error& e) {
        // 修复：直接抛出std::runtime_error，避开parse_error构造兼容性问题
        std::string error_msg = "JSON parse error in file " + file_path +
                                " (error code: " + std::to_string(e.id) + "): " + e.what();
        throw std::runtime_error(error_msg); // 改用通用的runtime_error
    }

    // 3. 映射到结构体并返回
    AddressRootConfig config = json_data.get<AddressRootConfig>();
    RCLCPP_INFO(rclcpp::get_logger("json_reader"), "Successfully read and parsed JSON config: %s", file_path.c_str());
    return config;
}