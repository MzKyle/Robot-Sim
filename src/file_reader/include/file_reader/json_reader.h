#ifndef JSON_CONFIG_READER__JSON_CONFIG_READER_HPP_
#define JSON_CONFIG_READER__JSON_CONFIG_READER_HPP_

//#include <nlohmann/json.hpp>
#include "json.hpp"
#include <string>
#include <vector>
#include <stdexcept>

// 简化JSON命名空间
using json = nlohmann::json;

// 定义与JSON对应的结构体（严格匹配字段名和类型）
struct AddressItem
{
    std::string Id;
    std::string Address;
    std::string StartPosition;
    std::string AddressLen;
    std::string RegisterType;
    std::string BlockNumber;
    std::string BlockType;
    std::string Explain;
    std::string Name;
    uint64_t ConfigTime;
    std::string Offset;
    std::string Authority;
    std::string Unit;
    std::string ValueType;
    std::string ExchangeByte;
    std::string TransformType;
    std::string Express;
    std::string ReadMin;
    std::string ReadMax;
    std::string TransformMin;
    std::string TransformMax;
    std::string Precision;
    std::string PrecisionType;
    std::string ScanInterval;
    std::string CollectionType;
    bool IsSaveLastValue;
};

struct AddressManageConfigItem
{
    std::string DeviceID;
    std::vector<AddressItem> AddressItemList;
};

struct AddressRootConfig
{
    std::vector<AddressManageConfigItem> AddressManageConfig;
};

// JSON反序列化（将JSON对象映射到结构体）
void from_json(const json& j, AddressItem& item);
void from_json(const json& j, AddressManageConfigItem& item);
void from_json(const json& j, AddressRootConfig& config);

// 通用读取函数：传入文件路径，返回解析后的AddressRootConfig结构体
// 异常：文件不存在/读取失败抛出std::runtime_error；JSON格式错误抛出nlohmann::json::parse_error
AddressRootConfig read_json_config(const std::string& file_path);

#endif  // JSON_CONFIG_READER__JSON_CONFIG_READER_HPP_