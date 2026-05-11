# data_collect_sim

`data_collect_sim` 是焊接数据采集工作空间的 Gazebo 仿真入口包。当前默认场景使用 `panda_weld_arm` 机械臂模型，默认通过纯 ROS 2 的 2D/RGBD 备份相机把仿真数据接到现有后端话题；需要时也可以切回 Gazebo 相机 bridge。

## 当前内容

- `models/panda_weld_arm/`：Panda 机械臂 SDF、mesh 和 PBR 材质。
- `worlds/weld_cell.world.sdf`：焊接工位场景、工件台和相机调试目标。
- `worlds/weld_cell.world.sdf`：直接 include `model://panda_weld_arm`，场景启动即加载机械臂。
- Gazebo 原生相机：可选发布 `/image_topic` 和 `/tcp_cloud_raw` 到 ROS 2 后端。
- `PosePublisher` + `tf_to_tcp_node`：从 Gazebo link pose 生成 `/tool_pos` 和 `/fanuc_robot_info`，保持后端接口不变。
- `panda_joint_demo_node`：发布 7 轴 Panda 关节位置命令，让末端相机随机械臂运动。
- 纯 ROS 相机仿真节点作为默认图像/点云来源保留。

## 启动方式

```bash
cd /home/kyle/sany/weld_data_collect_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select data_collect_sim data_collect_bringup data_collect data_collect_ui weld_interface
source install/setup.bash
ros2 launch data_collect_sim data_collect_sim.launch.py
```

默认启动会打开 Gazebo、加载 `panda_weld_arm`、启动纯 ROS 的 2D/RGBD 备份相机、启动末端 TF 到 `/tool_pos` 的转换，并启动后端采集节点。Gazebo 相机桥默认关闭；如果你的渲染环境可用，可以手动切回 `use_gz_sensors:=true`。

单独打开 Gazebo 场景：

```bash
ros2 launch data_collect_sim gazebo_world.launch.py
```

## ROS 2 数据接口

后端接口保持不变：

- `/image_topic`：Gazebo 末端 2D 相机图像。
- `/tcp_cloud_raw`：Gazebo 末端 RGBD 点云。
- `/tool_pos`：由 `world -> panda_weld_arm/camera_mount` TF 转换得到的 TCP 位姿。
- `/fanuc_robot_info`、`/fanuc_target_register_value`、`/fanuc_weld_register_info`：仿真机器人状态和寄存器信息，继续兼容现有后端。

Gazebo 内部 topic 使用 Panda 命名，例如 `/panda_weld_arm/pool_camera/image`、`/panda_weld_arm/tcp_rgbd/points`。关节控制在 ROS 侧使用 `/panda_weld_arm/joint/panda_joint*/cmd_pos`，再桥到 Gazebo 默认的 `/model/panda_weld_arm/joint/panda_joint*/0/cmd_pos`，这样既能兼容 ROS 命名规则，也和 Gazebo 自带的 Joint position controller 面板保持一致。

## 常用校验

```bash
gz sdf -k src/data_collect_sim/models/panda_weld_arm/model.sdf
ros2 topic hz /image_topic
ros2 topic hz /tcp_cloud_raw
ros2 topic echo /tool_pos --once
```

RViz 中 `Fixed Frame` 选 `world`，添加 `PointCloud2` 显示 `/tcp_cloud_raw`。Gazebo 中应只看到新的 Panda 机械臂，不再加载旧 Fanuc 仿真模型。

## 可选 fallback

如需临时回到纯 ROS 合成相机，不使用 Gazebo 相机数据：

```bash
ros2 launch data_collect_sim data_collect_sim.launch.py \
  use_gz_sensors:=false \
  use_sim_camera_2d:=true \
  use_sim_camera_3d:=true
```
