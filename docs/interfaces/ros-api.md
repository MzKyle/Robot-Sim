# ROS API

## Launch

| 命令 | 说明 |
| --- | --- |
| `ros2 launch robot_sim_bringup sim.launch.py` | 主仿真入口 |
| `ros2 launch robot_sim_bringup sensor_receivers.launch.py` | 单独启动 receiver |
| `ros2 launch robot_sim_bringup record_bag.launch.py` | 录制 rosbag |
| `ros2 launch robot_sim_bringup distributed_local.launch.py` | 本机分布式模拟 |

## 常用参数

| 参数 | 示例 | 说明 |
| --- | --- | --- |
| `sim_profile` | `panda` | 内置 profile 名称 |
| `sim_profile_file` | `/path/to/custom.yaml` | 外部 profile |
| `sim_mode` | `mock` / `light` / `full` | 仿真模式 |
| `sensor_overrides` | `camera=true,lidar=false` | 传感器覆盖 |
| `rviz` | `true` / `false` / `auto` | RViz 开关 |
| `use_moveit` | `true` / `false` / `auto` | MoveIt 开关 |
| `headless` | `true` / `false` / `auto` | Gazebo GUI 开关 |
| `use_gripper` | `true` / `false` | 是否启动带 `enabled_by: use_gripper` 的 controller |

`distributed_local.launch.py` 另提供 `rqt_graph`；`sensor_receivers.launch.py` 使用
`layout` 参数而不是 `sim_mode`。精确参数以 `ros2 launch <pkg> <file> --show-args` 为准。

## Topics

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` | 机器人关节状态 |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB 图像 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB 相机内参 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 深度图 |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 深度相机内参 |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | RGBD 点云 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D lidar |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | 3D lidar 点云 |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | receiver 健康状态；仅在单独启动 receiver 后出现 |

## Actions

| Action | 说明 |
| --- | --- |
| `/arm_controller/follow_joint_trajectory` | 主轨迹控制器 action |
| `/move_action` | MoveIt MoveGroup action |

## Interfaces

- `robot_task_interfaces`：任务上下文消息和服务类型声明。
- `simulation_interfaces`：仿真场景消息和服务类型声明。

当前仓库没有为这些 service 类型安装 server；它们不是 `run_case` 的控制 API。
