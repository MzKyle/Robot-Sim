# 采集核心

`data_collect` 是工作空间的核心模块，负责将相机图像、点云、机器人位姿、Fanuc 状态、焊接寄存器和质量评估信息写入磁盘，并维护采集元数据。

## 主要职责

- 接收采集状态和外部数据流。
- 根据采样间隔保存图像、点云、日志和状态文件。
- 写入 `manifest.json` 和兼容旧流程的 `meta.json`。
- 发布采集状态到 `/data_collect_status`。

## 与 UI 的关系

- UI 通过服务控制采集开始、停止和任务录入。
- UI 订阅状态话题，展示当前保存目录和统计数量。
- UI 可直接修改后端统一配置文件。

## 关键输出目录

- `camera/`、`camera_log/`
- `height_log/`、`camera_depth/`、`camera_depth_log/`
- `scan_point_cloud/`
- `robot_state/`
- `welding_state/`、`control_cmd/`、`state_type/`
- `fanuc_robot_info/`
