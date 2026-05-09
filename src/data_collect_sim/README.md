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

ros_gz 传感器接入说明（示例）:

 - 方案：在模型 SDF 中为相机挂载 `sensor` 并使用 ros_gz 的 camera/depth plugin，将数据桥接到 ROS 2 topics。下面是一个示例传感器片段（替换为你实际使用的 plugin 名称与参数）：

```xml
<sensor name="camera_sensor" type="camera">
	<pose>0 0 0 0 0 0</pose>
	<camera>
		<horizontal_fov>1.047</horizontal_fov>
		<image>
			<width>640</width>
			<height>480</height>
			<format>R8G8B8</format>
		</image>
		<clip>
			<near>0.1</near>
			<far>100.0</far>
		</clip>
	</camera>
	<plugin name="ros_gz_camera_plugin" filename="libros_gz_camera.so">
		<!-- plugin-specific params: topic name, frame id, qos, etc. -->
	</plugin>
</sensor>
```

 - 在启用 `ros_gz` 桥 (ros_gz_bridge/ros_gz) 后，plugin 会将 `sensor_msgs/Image` / `sensor_msgs/PointCloud2` 等发布到 ROS 2。确保 topic 名称与 `data_collect` 后端期望的 topic 对齐（例如 `/image_topic`, `/tcp_cloud_raw`）。

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

这个启动方式会自动加载官方 URDF Fanuc 模型。默认不会加载 Gazebo 相机插件，因此不会再出现 `libros_gz_camera.so` / `libros_gz_depth_camera.so` 缺失报错；图像和点云默认由纯 ROS 仿真节点提供。

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

若后续需要重新启用 Gazebo 模型内的相机插件，可以打开：

```bash
ros2 launch data_collect_sim data_collect_sim.launch.py \
	enable_gz_camera_plugins:=true \
	use_sim_camera_2d:=false \
	use_sim_camera_3d:=false
```

若 Gazebo 传感器输出走 Gazebo Transport，而不是 ROS 直出，可以再加上桥接：

```bash
ros2 launch data_collect_sim data_collect_sim.launch.py \
	enable_gz_camera_plugins:=true \
	use_sim_camera_2d:=false \
	use_sim_camera_3d:=false \
	use_gz_bridge:=true
```

## 现阶段说明

当前默认路径已经不再加载 Gazebo 相机插件，因此正常启动不应再出现 `libros_gz_camera.so` / `libros_gz_depth_camera.so` 缺失报错。只有在显式打开 `enable_gz_camera_plugins:=true` 时，才会重新尝试加载这些插件。
