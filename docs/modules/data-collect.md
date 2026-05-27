# 采集核心

`data_collect` 是采集保存模块，负责将相机图像、点云、机器人位姿、设备状态和质量评估信息写入磁盘，并维护采集元数据。焊接和 Fanuc 信息作为当前 adapter 保留。

## 主要职责

- 接收采集状态和外部数据流。
- 根据采样间隔保存图像、点云、日志和状态文件。
- 写入 `manifest.json` 和兼容旧流程的 `meta.json`。
- 发布通用采集状态到 `/acquisition/status`，并保留旧 `/data_collect_status`。

## 与 UI 的关系

- UI 通过服务控制采集开始、停止和任务录入。
- UI 订阅状态话题，展示当前保存目录和统计数量。
- UI 可直接修改后端统一配置文件。
- 新项目优先使用 `/task/set_context` 或 `/acquisition/set_task` 写入任务信息；旧 UI 仍可使用 `/data_collect_set_task`。

## 关键输出目录

- `camera/`、`camera_log/`
- `height_log/`、`camera_depth/`、`camera_depth_log/`
- `scan_point_cloud/`
- `robot_state/`
- `welding_state/`、`control_cmd/`、`state_type/`
- `fanuc_robot_info/`
