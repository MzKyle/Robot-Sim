# data_collect_sim

`data_collect_sim` 是焊接数据采集工作空间的仿真入口包，目标是先让后端和 Qt 前端在没有真实设备的情况下完成联调。

## 当前内容

- 2D 相机仿真节点，发布 `/image_topic`。
- 3D 相机仿真节点，发布 `/tcp_cloud_raw`、`/fixed_scan`、`/fixed_scan_all`、`/scan_pose`。
- Fanuc 机器人仿真节点，发布 `/tool_pos`、`/fanuc_robot_info`、`/fanuc_target_register_value`、`/fanuc_weld_register_info`。
- 仿真配置文件 `config/nodemanage_sim.yaml`。
- 仿真场景骨架 `worlds/weld_cell.world.sdf`。
- `gazebo_world.launch.py` 可以单独打开 Gazebo Sim 场景（含 Fanuc 简化模型）。
- `tf_to_tcp_node`：从 Gazebo TF 发布器读取工具位姿，发布 `/tool_pos` 和 `/fanuc_robot_info`，用于让场景驱动数据驱动后端。
- `spawn_fanuc_m20i_node`：通过 `gz model -f ... -m ... -w ...` 将模型 `share/data_collect_sim/fanuc_m20i/model.sdf` spawn 到场景中（可替换为更真实的 Fanuc SDF/URDF）。

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

 - 如果你已有 Fanuc M-20i 的完整 URDF/SDF，替换 `models/fanuc_m20i/model.sdf` 即可；或者将真实模型放到 `share/data_collect_sim/models/fanuc_m20i/` 下并在 launch 中调整参数。

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

这个启动方式会自动加载 Fanuc 简化模型（带 2D 熔池相机 + 3D 线扫点云相机）；如果你只看到地面和桌子，检查 `GZ_SIM_RESOURCE_PATH` 是否包含 `data_collect_sim/models`。

手动 spawn 模型：

```bash
gz model -f /home/kyle/sany/weld_data_collect_ws/install/data_collect_sim/share/data_collect_sim/fanuc_m20i/model.sdf -m fanuc_m20i -w weld_cell
```

然后再启动前端：

```bash
ros2 run data_collect_ui data_collect_ui
```

## Gazebo 传感器与仿真节点切换

默认情况下，`data_collect_sim.launch.py` 会打开 Gazebo Sim，并使用模型里的传感器发布 `/image_topic` 和 `/tcp_cloud_raw`。
如果需要回退到接口级仿真（纯 ROS 节点生成图像/点云），可以在启动时打开开关：

```bash
ros2 launch data_collect_sim data_collect_sim.launch.py \
	use_sim_camera_2d:=true \
	use_sim_camera_3d:=true
```

若你的 Gazebo 传感器输出走 Gazebo Transport，而不是 ROS 直出，可以加上桥接：

```bash
ros2 launch data_collect_sim data_collect_sim.launch.py use_gz_bridge:=true
```

## 现阶段说明

这一步先完成接口级仿真，后续可以再把这三个节点替换成真正的 Gazebo 传感器和 M-20i 机器人模型。这样后端和前端的测试不会被仿真实现细节拖住。