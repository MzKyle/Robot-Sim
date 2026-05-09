# data_collect_sim

`data_collect_sim` 是焊接数据采集工作空间的仿真入口包，目标是先让后端和 Qt 前端在没有真实设备的情况下完成联调。

## 当前内容

- 2D 相机仿真节点，发布 `/image_topic`。
- 3D 相机仿真节点，发布 `/tcp_cloud_raw`、`/fixed_scan`、`/fixed_scan_all`、`/scan_pose`。
- Fanuc 机器人仿真节点，发布 `/tool_pos`、`/fanuc_robot_info`、`/fanuc_target_register_value`、`/fanuc_weld_register_info`。
- 仿真配置文件 `config/nodemanage_sim.yaml`。
- 仿真场景骨架 `worlds/weld_cell.world.sdf`。
- `gazebo_world.launch.py` 可以单独打开 Gazebo Sim 场景，并在启动后显式 spawn 基于官方 `fanuc_m20ib_support/m20ib25` URDF 的 Fanuc M-20iB/25 模型。
- `tf_to_tcp_node`：从 Gazebo TF 发布器读取工具位姿，发布 `/tool_pos` 和 `/fanuc_robot_info`，用于让场景驱动数据驱动后端。
- `spawn_fanuc_m20i_node`：默认通过官方 `m20ib25` xacro 生成 URDF，并使用 `ros_gz_sim create` spawn 到场景中；旧的手写 SDF 仅保留为非默认 fallback。
- 官方 URDF 的 `camera_mount` 现在可以直接挂原生 Gazebo 传感器，不再依赖 `libros_gz_camera.so` / `libros_gz_depth_camera.so`。
- Gazebo 机器人已取消 `<static>true</static>`，6 个关节通过 Gazebo `JointPositionController` 接收位置命令。
- `joint_state_to_gz_joint_cmd_node`：订阅 `/joint_states`，把 `joint_state_publisher_gui` 的 6 个关节位置转成 Gazebo joint position command。
- `robot_description_publisher_node`：发布 `/robot_description`，供 `joint_state_publisher` / `joint_state_publisher_gui` 读取官方 Fanuc URDF。

- 当前 `urdf/` 目录内置了从官方 `fanuc_m20ib_support` / `fanuc_resources` 迁移来的 xacro 资源，`models/fanuc_m20i/meshes` 内保留官方 `m20ib25` visual / collision mesh；如果后续要继续和 ROS-Industrial 原包同步，优先以 `src/fanuc/` 为上游来源。

## 启动方式

```bash
cd /home/kyle/sany/weld_data_collect_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select data_collect_sim data_collect_bringup data_collect data_collect_ui weld_interface
source install/setup.bash
ros2 launch data_collect_sim data_collect_sim.launch.py
```

单独打开 Gazebo Sim 场景：

```bash
ros2 launch data_collect_sim gazebo_world.launch.py
```

这个启动方式会自动加载官方 URDF Fanuc 模型。默认不会加载 Gazebo 传感器，因此图像和点云默认仍由纯 ROS 仿真节点提供。

手动 spawn 模型：

```bash
gz model -f /home/kyle/sany/weld_data_collect_ws/install/data_collect_sim/share/data_collect_sim/fanuc_m20i/model.sdf -m fanuc_m20i -w weld_cell
```

然后再启动前端：

```bash
ros2 run data_collect_ui data_collect_ui
```

## Gazebo 传感器与仿真节点切换

默认情况下，`data_collect_sim.launch.py` 会打开 Gazebo Sim，并默认启用纯 ROS 相机仿真节点发布 `/image_topic` 和 `/tcp_cloud_raw`：

```bash
ros2 launch data_collect_sim data_collect_sim.launch.py
```

如果需要关闭纯 ROS 相机仿真节点，可以显式指定：

```bash
ros2 launch data_collect_sim data_collect_sim.launch.py \
	use_sim_camera_2d:=false \
	use_sim_camera_3d:=false
```

如果需要改成原生 Gazebo 2D / 3D 传感器输出，并通过 `ros_gz_bridge` 接到 ROS 2，使用：

```bash
ros2 launch data_collect_sim data_collect_sim.launch.py \
	use_gz_sensors:=true
```

此时会自动发生这几件事：

- 不启动 `sim_camera_2d_node`
- 不启动 `sim_camera_3d_node`
- 启动 `ros_gz_bridge parameter_bridge`
- 将 Gazebo 原生 2D 图像桥接到 `/image_topic`
- 将 Gazebo 原生 3D 点云桥接到 `/tcp_cloud_raw`

Gazebo 原生相机挂在官方 Fanuc `tool0` 后的 `camera_mount` 上。`camera_mount` 相对 `tool0` 的位姿是 `xyz="0.04 0 0.13"`，`rpy="0 3.141592653589793 0"`；Gazebo 相机光轴沿 sensor frame 的 `+X` 方向，这个姿态让相机默认朝向工件台面。场景里放了红色方块、绿色圆柱和蓝色球作为相机调试目标。

## 用 joint_state_publisher_gui 控制 Gazebo 机械臂

第一次使用前确认 GUI 包已安装：

```bash
sudo apt install ros-humble-joint-state-publisher-gui
```

启动 Gazebo 原生相机、GUI 关节滑条和 Gazebo 关节位置控制：

```bash
cd /home/kyle/sany/weld_data_collect_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select data_collect_sim
source install/setup.bash
ros2 launch data_collect_sim data_collect_sim.launch.py \
	use_gz_sensors:=true \
	use_joint_state_gui:=true \
	use_gz_joint_control:=true \
	use_sim_fanuc:=false \
	use_tf_to_tcp:=true
```

这个模式下的数据链路是：

- `joint_state_publisher_gui` 发布 `/joint_states`
- `joint_state_to_gz_joint_cmd_node` 发布 `/fanuc_m20i/joint_*/cmd_pos`
- `ros_gz_bridge` 把这些 ROS 位置命令桥到 Gazebo
- Gazebo 的 6 个 `JointPositionController` 驱动官方 Fanuc URDF 关节
- `camera_mount` 固连在 `tool0` 后面，所以 2D 图像 `/image_topic` 和 3D 点云 `/tcp_cloud_raw` 会随着末端姿态变化

常用可视化：

```bash
ros2 run rqt_image_view rqt_image_view /image_topic
rviz2
```

RViz 中 `Fixed Frame` 选 `world`，添加 `PointCloud2` 显示 `/tcp_cloud_raw`。默认会启动 `robot_state_publisher` 和普通 `joint_state_publisher` 来发布 TF；开启 `use_joint_state_gui:=true` 时由 GUI 取代普通 joint state publisher。

## 现阶段说明

当前默认路径已经不再加载 `libros_gz_camera.so` / `libros_gz_depth_camera.so`，因此正常启动不应再出现这两个缺失报错。只有在 `use_gz_sensors:=true` 时，才会启用官方 URDF 上的原生 Gazebo `camera` / `rgbd_camera` 传感器，并通过 `ros_gz_bridge` 输出到 ROS 2。
