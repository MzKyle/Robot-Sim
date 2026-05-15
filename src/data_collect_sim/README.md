# data_collect_sim

`data_collect_sim` 是焊接数据采集工作空间的 gz sim 8 仿真入口包。这里保留两条可选的数据链路：一条是简单的 mock 测试链，继续像以前一样发布随机生成的图像和点云；另一条是 gz sim 8 仿真链，直接读取与相机挂接的真实仿真数据。两条路都保持后端接口不变。

## 当前内容

- `models/panda_weld_arm/`：Panda 机械臂 SDF、mesh 和 PBR 材质。
- `worlds/weld_cell.world.sdf`：gz sim 8 焊接工位场景、工件台和相机调试目标。
- `worlds/weld_cell.world.sdf`：直接 include `model://panda_weld_arm`，场景启动即加载机械臂。
- gz sim 8 原生相机：可选发布 `/image_topic` 和 `/tcp_cloud_raw` 到 ROS 2 后端，侧装在末端并朝向 TCP。
- `PosePublisher` + `tf_to_tcp_node`：从 gz sim 8 link pose 生成 `/tool_pos` 和 `/fanuc_robot_info`，保持后端接口不变。
- `panda_joint_demo_node`：发布 7 轴 Panda 关节位置命令，让末端相机随机械臂运动。
- 纯 ROS 相机仿真节点作为 mock 测试来源保留，用于快速验证后端链路。

## 启动方式

```bash
cd /home/kyle/sany/weld_data_collect_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select data_collect_sim data_collect_bringup data_collect data_collect_ui weld_interface
source install/setup.bash
ros2 launch data_collect_sim data_collect_sim.launch.py
```

 默认启动会打开 gz sim 8 并加载 `panda_weld_arm`，launch 文件的默认参数为 `use_gazebo=true`、`use_gz_sensors=true`。因此默认会启用 gz 仿真与 ros_gz_bridge 的传感器桥接，仿真链路作为默认数据源；当你将 `use_gz_sensors:=false` 时，才会回退到 mock 相机链以便快速联调后端。

两条路的选择方式：

```bash
# 1. mock 测试链：随机图像/点云，适合快速联调后端
ros2 launch data_collect_sim data_collect_sim.launch.py \
  use_gz_sensors:=false \
  use_sim_camera_2d:=true \
  use_sim_camera_3d:=true

# 2. gz sim 8 仿真链：读取与模型相机绑定的实际仿真数据
ros2 launch data_collect_sim data_collect_sim.launch.py \
  use_gz_sensors:=true \
  use_sim_camera_2d:=false \
  use_sim_camera_3d:=false
```

如果你希望保留 gz sim 8 机械臂运动但仍使用 mock 图像，也可以把 `use_gz_joint_control:=true` 保持开启，只关闭 `use_gz_sensors`。

单独打开 gz sim 8 场景：

```bash
ros2 launch data_collect_sim gazebo_world.launch.py
```

## ROS 2 数据接口

后端接口保持不变：

- `/image_topic`：gz sim 8 末端 2D 相机图像。
- `/tcp_cloud_raw`：gz sim 8 末端 RGBD 点云。
- `/tool_pos`：由 `world -> panda_weld_arm/panda_link8` TF 转换得到的真实末端 TCP 位姿；`camera_mount` 仅表示相机安装 frame。
- `/fanuc_robot_info`、`/fanuc_target_register_value`、`/fanuc_weld_register_info`：仿真机器人状态和寄存器信息，继续兼容现有后端。

gz sim 8 内部 topic 使用 Panda 命名，例如 `/panda_weld_arm/pool_camera/image`、`/panda_weld_arm/tcp_rgbd/points`。关节控制在 ROS 侧使用 `/panda_weld_arm/joint/panda_joint*/cmd_pos`，再桥到 gz sim 8 默认的 `/model/panda_weld_arm/joint/panda_joint*/0/cmd_pos`，这样既能兼容 ROS 命名规则，也和 gz 自带的 Joint position controller 面板保持一致。

## 常用校验

```bash
gz sdf -k src/data_collect_sim/models/panda_weld_arm/model.sdf
ros2 topic hz /image_topic
ros2 topic hz /tcp_cloud_raw
ros2 topic echo /tool_pos --once
```

RViz 中 `Fixed Frame` 选 `world`，添加 `PointCloud2` 显示 `/tcp_cloud_raw`。gz sim 8 中应只看到新的 Panda 机械臂，不再加载旧 Fanuc 仿真模型。

## 可选 fallback

mock 路就是 fallback 路，适合没有稳定渲染环境、但又要验证 `/image_topic`、`/tcp_cloud_raw`、`/tool_pos` 和后端采集逻辑时使用。
