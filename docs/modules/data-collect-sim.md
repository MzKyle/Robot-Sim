# 仿真平台

仿真入口已统一到 `robot_sim_bringup`，旧 `data_collect_sim` 包已移除。

## 主要职责

- 启动 gz sim 8 场景和 Panda 机器人。
- 在 Gazebo 模式中通过 `gz_ros2_control/GazeboSimSystem` 接入 `ros2_control`。
- 通过按组开关控制 RGB、depth、lidar 和 IMU，避免无关传感器消耗性能。
- 提供 mock、light、full 三档模式覆盖快速控制验证、轻量仿真和完整感知仿真。

## 启动参数

- `sim_mode`：`mock`、`light` 或 `full`，默认 `light`。
- `enable_camera`：RGB 图像和 CameraInfo。
- `enable_depth`：深度图和点云。
- `enable_lidar`：2D scan 和 3D lidar 点云。
- `enable_imu`：IMU。
- `use_moveit`、`rviz`、`headless`、`use_sim_time`：均支持 `auto|true|false`。

## 常见命令

```bash
ros2 launch robot_sim_bringup sim.launch.py
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=mock
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=full
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=light enable_camera:=true
```
