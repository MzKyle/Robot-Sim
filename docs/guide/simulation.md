# 仿真方案（详尽指南）

本文档对齐 `src/data_collect_sim` 包，说明如何使用 gz sim 8（默认）或 mock 链路来验证后端采集流程及前端显示。

## 目标

- 在没有真实 2D/3D 相机与 Fanuc 控制器的环境下，能完整启动后端并验证数据采集流程。
- 保持对后端接口的兼容性：`data_collect`、`data_collect_ui` 和 `weld_interface` 无需修改。

## 相关源码位置

- 仿真包：[src/data_collect_sim](src/data_collect_sim/README.md)
- 世界文件：[src/data_collect_sim/worlds/weld_cell.world.sdf](src/data_collect_sim/worlds/weld_cell.world.sdf)
- Panda 模型：[src/data_collect_sim/models/panda_weld_arm](src/data_collect_sim/models/panda_weld_arm)
- 启动文件：[src/data_collect_sim/launch/data_collect_sim.launch.py](src/data_collect_sim/launch/data_collect_sim.launch.py)
- 单独世界启动：[src/data_collect_sim/launch/gazebo_world.launch.py](src/data_collect_sim/launch/gazebo_world.launch.py)

## 说明（包内功能概览）

- `panda_weld_arm`：Panda 机械臂的 SDF、mesh、材质与末端相机挂载。
- 世界文件 `weld_cell.world.sdf` 会在启动时 include `model://panda_weld_arm`，加载机械臂和工位场景。
- 提供两条数据链路：
	- mock 链路：本地 ROS 节点发布随机/静态的图像、点云与机器人状态，便于快速联调。
	- gz sim 8 链路：通过 ros_gz_bridge 或包内脚本，将 gz sim 8 的相机与 link pose 转换为后端接口话题。
- 提供节点与脚本：`sim_camera_2d_node`、`sim_camera_3d_node`、`tf_to_tcp_node`、`sim_fanuc_robot_node`、`panda_joint_demo_node` 等。

## 启动前准备

1. 安装 ROS 2（本仓库使用 Humble 作为示例）。
2. 确保系统上已安装 gz sim 8 / gazebo（与 ros_gz_bridge 兼容）。
3. 在工作空间根目录执行：

```bash
cd /home/kyle/sany/weld_data_collect_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select data_collect_sim data_collect_bringup data_collect data_collect_ui weld_interface
source install/setup.bash
```

## 启动示例

- 默认（启动 gz sim 8 并根据参数选择传感器链）：

```bash
ros2 launch data_collect_sim data_collect_sim.launch.py
```

- 只使用 mock 测试链（快速联调后端）：

```bash
ros2 launch data_collect_sim data_collect_sim.launch.py \
	use_gz_sensors:=false \
	use_sim_camera_2d:=true \
	use_sim_camera_3d:=true
```

- 使用 gz sim 8 原生相机与点云（需要渲染环境）：

```bash
ros2 launch data_collect_sim data_collect_sim.launch.py \
	use_gz_sensors:=true \
	use_sim_camera_2d:=false \
	use_sim_camera_3d:=false
```

- 如果只想单独打开世界（不带后端节点）：

```bash
ros2 launch data_collect_sim gazebo_world.launch.py
```

说明：按 `src/data_collect_sim/launch/data_collect_sim.launch.py` 的默认参数，`use_gazebo=true`、`use_gz_sensors=true`，因此默认会启用 gz 仿真与 ros_gz_bridge 的传感器桥接；mock 摄像头节点仅在 `use_gz_sensors=false` 时启动。若需要只保留 gz 机械臂运动但使用 mock 图像，可以启用 `use_gz_joint_control:=true` 并设置 `use_gz_sensors:=false`。

## 关键 ROS2 话题与接口

- `/image_topic` — 末端 2D 相机图像（sensor_msgs/Image）。
- `/tcp_cloud_raw` — 末端 RGB-D 点云（sensor_msgs/PointCloud2）。
- `/tool_pos` — 由仿真 link pose 转换来的末端 TCP 位姿（geometry_msgs/PoseStamped 或自定义结构，参见 `tf_to_tcp_node` 实现）。
- `/fanuc_robot_info`、`/fanuc_target_register_value`、`/fanuc_weld_register_info` — 仿真机器人状态与寄存器信息（与后端原有话题保持兼容）。

内部 gz sim 话题示例（仅供调试）：

- `/panda_weld_arm/pool_camera/image`
- `/panda_weld_arm/tcp_rgbd/points`

关节控制桥接：

- ROS 侧使用 `/panda_weld_arm/joint/panda_joint*/cmd_pos`，再桥到 gz 的 `/model/panda_weld_arm/joint/panda_joint*/0/cmd_pos`。

## 配置文件与桥接

- ros_gz_bridge 配置文件位于 `src/data_collect_sim/config/`（例如 `ros_gz_bridge_sensors.yaml`、`ros_gz_bridge_tf.yaml`、`ros_gz_bridge_joints.yaml`），用于定义 topic <-> bridge 映射。
- 节点管理配置 `nodemanage_sim.yaml` 保存了默认启停策略。

## 校验与调试命令

```bash
gz sdf -k src/data_collect_sim/models/panda_weld_arm/model.sdf
ros2 topic hz /image_topic
ros2 topic hz /tcp_cloud_raw
ros2 topic echo /tool_pos --once
```

在 RViz 中将 `Fixed Frame` 设为 `world`，增加 `PointCloud2` 显示 `/tcp_cloud_raw`，检查点云是否出现。

## 输出与验收要点

- `/data_collect_status` 能持续更新，前端能接收机器人与寄存器信息。
- `/image_topic`、`/tcp_cloud_raw` 与 `/tool_pos` 为稳定数据流。
- `data_collect` 节点能将采集数据落盘并生成 `manifest.json`。

## 故障回退（fallback）

当渲染或 gz 环境不可用时，默认回退到 mock 链路，mock 节点会发布可用于后端验证的随机或静态图像与点云。适用于 CI / 无 GPU 环境的快速回归测试。

## 后续建议

- 将 mock 链路统一到单一仿真配置入口（减少启动参数组合）。
- 增加故障注入场景（延迟、丢包、传感器噪声）用于回归测试。
- 根据现场需要微调相机位姿与世界模型以提高数据真实性。

更多细节请参考包内 README：[src/data_collect_sim/README.md](src/data_collect_sim/README.md)