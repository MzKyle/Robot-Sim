# 仿真入口

`data_collect_sim` 是焊接数据采集工作空间的 gz sim 8 仿真入口包。它把真实采集接口复刻成两条可切换的数据链路：一条是 mock 测试链，继续发布随机图像、点云和机器人状态；另一条是 gz sim 8 链路，直接从 Panda 机械臂和挂载相机读取仿真数据。

## 主要职责

- 启动 gz sim 8 和 `weld_cell.world.sdf` 场景。
- 挂载 `panda_weld_arm` 模型，并通过 `panda_joint_demo_node` 驱动关节演示。
- 通过 `ros_gz_bridge` 把 gz 图像、点云、TF 和关节控制桥接到 ROS 2。
- 提供 `sim_camera_2d_node`、`sim_camera_3d_node`、`sim_fanuc_robot_node` 和 `tf_to_tcp_node` 作为 mock 或兼容链路。
- 保持 `/image_topic`、`/tcp_cloud_raw`、`/tool_pos`、`/fanuc_robot_info`、`/fanuc_target_register_value`、`/fanuc_weld_register_info` 等后端接口不变。

## 启动开关

`data_collect_sim.launch.py` 提供的主要开关如下：

- `use_gazebo`：是否启动 gz sim 8。
- `use_gz_sensors`：是否使用 gz 传感器桥接图像和点云。
- `use_sim_camera_2d`：是否启用 mock 2D 相机。
- `use_sim_camera_3d`：是否启用 mock 3D 相机。
- `use_sim_fanuc`：是否启用仿真机器人状态节点。
- `use_tf_to_tcp`：是否启用 TF 到 TCP 的转换节点。
- `use_gz_joint_control`：是否启用关节控制桥接和演示节点。

**默认值**：launch 文件中默认设置为 `use_gazebo=true`、`use_gz_sensors=true`、`use_sim_camera_2d=true`、`use_sim_camera_3d=true`。当 `use_gz_sensors=true` 时，ros_gz_bridge 会被启用，mock 摄像头节点仅在 `use_gz_sensors=false` 时启动。

## 关键话题

- `/image_topic`：gz 相机或 mock 2D 相机图像。
- `/tcp_cloud_raw`：gz RGBD 点云或 mock 3D 点云。
- `/tool_pos`：真实末端位姿或仿真 TCP 位姿。
- `/fanuc_robot_info`、`/fanuc_target_register_value`、`/fanuc_weld_register_info`：仿真机器人状态和寄存器信息。
- `/start_fix_scan`、`/stop_fix_scan`：固定扫描控制。
- `/scan_3d`：3D 单次扫描服务。

## 适用场景

- 没有真实相机或机器人时联调后端和 UI。
- 验证 gz sim 8 场景、桥接配置和关节控制。
- 做回归测试时切换到 mock 链路快速启动。