# 新机械臂 Gazebo + ROS2 仿真场景改造

## Summary
- 使用根目录 `/home/kyle/sany/weld_data_collect_ws/model` 的新 Panda SDF/mesh/texture 作为 `data_collect_sim` 的唯一 Gazebo 机械臂模型。
- 保持后端 ROS2 接口不变：`/image_topic`、`/tcp_cloud_raw`、`/tool_pos`、`/fanuc_robot_info` 等继续可用。
- 按用户确认删除旧 `fanuc_m20i` 仿真资产，不再保留昨天的旧模型链路。

## Key Changes
- 将新模型安装到 `src/data_collect_sim/models/panda_weld_arm/`，更新 `model.config` 和 `model.sdf` 名称为稳定的 `panda_weld_arm`，避免继续暴露旧 Fanuc 模型名。
- 在 Panda SDF 内新增 `camera_mount` link，并用 fixed joint 固定到 `panda_link8` 末端；默认沿用旧相机安装偏置 `0.04 0 0.13` 和朝向 `0 pi 0`，让相机朝向工件区域。
- 在 `camera_mount` 上添加 Gazebo 原生 `camera` 和 `rgbd_camera`，Gazebo topic 使用 `/panda_weld_arm/pool_camera/image` 与 `/panda_weld_arm/tcp_rgbd/points`。
- 给 Panda 模型启用 `PosePublisher`，发布 link pose 到 `/model/panda_weld_arm/pose`；通过 `ros_gz_bridge` 桥到 ROS `/tf`。
- 更新 bridge 配置：把 Gazebo 图像、点云、TF、7 个 Panda 关节命令桥到 ROS2；ROS2 图像/点云 topic 仍映射到 `/image_topic`、`/tcp_cloud_raw`。
- 更新启动文件：`gazebo_world.launch.py` spawn 新模型；`data_collect_sim.launch.py` 关节控制参数改为 `panda_joint1..panda_joint7`，`tf_to_tcp_node` 默认监听 `world -> camera_mount` 或 `world -> panda_link8` 后发布 `/tool_pos`。
- 删除旧资产和旧默认逻辑：移除 `src/data_collect_sim/models/fanuc_m20i`、Fanuc xacro 依赖链、`spawn_fanuc_m20i_node` 的旧默认使用路径，并重命名/替换为通用 `spawn_sim_model_node` 或 `spawn_panda_weld_arm_node`。

## Public Interfaces
- 保持不变：`/image_topic` 为 `sensor_msgs/msg/Image`，`/tcp_cloud_raw` 为 `sensor_msgs/msg/PointCloud2`，`/tool_pos` 为 `weld_interface/msg/TcpPos`。
- Gazebo 内部 topic 改为 Panda 命名，但不要求后端消费这些内部 topic。
- 关节命令内部改为 `/panda_weld_arm/panda_joint{1..7}/cmd_pos`，只影响仿真控制，不影响数据采集后端。

## Test Plan
- 静态校验：运行 `gz sdf -k src/data_collect_sim/models/panda_weld_arm/model.sdf`，确认 SDF 有效。
- 构建校验：运行 `colcon build --symlink-install --packages-select data_collect_sim data_collect_bringup data_collect weld_interface`。
- 启动校验：运行 `ros2 launch data_collect_sim data_collect_sim.launch.py use_gz_sensors:=true use_tf_to_tcp:=true use_gz_joint_control:=true`。
- 数据校验：确认 `ros2 topic hz /image_topic`、`ros2 topic hz /tcp_cloud_raw`、`ros2 topic echo /tool_pos --once` 都有数据。
- 可视化校验：Gazebo 中只出现新 Panda 机械臂，末端相机随关节运动；旧 Fanuc 模型不再加载。

## Assumptions
- 新模型根目录 `model/` 是最终资产来源，会被复制进 `data_collect_sim/models/panda_weld_arm/` 并纳入安装。
- 后端仍按现有 ROS2 topic 消费数据，不迁移到 `panda_*` ROS topic。
- 相机默认固定到 `panda_link8`；如果实际末端工具应挂在 `panda_hand` 或其他 link，后续只调整 fixed joint parent 和偏置即可。
