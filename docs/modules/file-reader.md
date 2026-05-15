# 配置读取工具

`file_reader` 提供 YAML 和 JSON 的读取辅助能力，主要服务于启动入口、配置同步和历史数据读取。

## 主要职责

- 读取和解析配置文件。
- 为 launch、采集节点和 UI 提供统一的数据入口。
- 减少各个节点重复解析配置的代码。

## 使用场景

- `data_collect_bringup` 读取默认配置。
- `data_collect_ui` 读取并保存参数页中的配置。
