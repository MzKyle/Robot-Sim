# 仿真方案

本方案以 gz sim 8 为主，保留 mock 节点作为备用链路。

## 目标

- 不依赖真实 2D 相机、3D 相机和 Fanuc 控制器也能启动后端。
- 让 `data_collect`、`data_collect_ui` 和现有 ROS 接口保持不变。
- 用 gz sim 8 数据验证采集保存、状态显示、服务调用和参数修改流程。
- 当渲染环境不可用时，退回 mock 图像、点云和机器人状态链路。

## 当前仿真包

- `data_collect_sim`
- gz sim 8 场景加载 `panda_weld_arm`
- 2D 仿真节点或 gz 相机桥接发布 `/image_topic`
- 3D 仿真节点或 gz 相机桥接发布 `/tcp_cloud_raw`、`/fixed_scan`、`/fixed_scan_all`、`/scan_pose`、`/debug_height_img`
- 仿真机器人节点发布 `/tool_pos`、`/fanuc_robot_info`、`/fanuc_target_register_value`、`/fanuc_weld_register_info`
- 可单独启动 `weld_cell.world.sdf`

## 启动顺序

1. 编译工作空间。
2. 启动仿真 bringup。
3. 启动 Qt 前端。
4. 在前端里验证采集启停、任务写入、参数修改和历史查看。

## 现阶段验收

- `/data_collect_status` 可以持续更新。
- 前端可以收到 `/fanuc_robot_info`、`/fanuc_target_register_value` 和 `/fanuc_weld_register_info`。
- `/image_topic`、`/tcp_cloud_raw` 和 `/tool_pos` 有稳定数据。
- `data_collect` 可以落盘并生成 `manifest.json`。
- `start_fix_scan`、`stop_fix_scan`、`/fanuc_register_read` 和 `/reload_camera_3d_config` 可用。

## 下一步

- 把 mock 链路进一步收敛到统一的仿真配置入口。
- 增加更多故障注入场景，用于回归测试。
- 按现场需要微调 gz sim 8 模型和相机位姿。