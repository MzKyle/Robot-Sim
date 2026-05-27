# 仿真平台

仿真入口已统一到 `robot_sim_bringup`，旧 `data_collect_sim` 包已移除。

## 主要职责

- 启动 gz sim 8 场景和 Panda 机器人。
- 在 Gazebo 模式中通过 `gz_ros2_control/GazeboSimSystem` 接入 `ros2_control`。
- 通过 `sensor_overrides` 按组控制 RGB、depth、lidar 和 IMU，避免无关传感器消耗性能。
- 提供 mock、light、full 三档模式覆盖快速控制验证、轻量仿真和完整感知仿真。

## 启动参数

- `sim_mode`：`mock`、`light` 或 `full`，默认 `light`。
- `sim_profile`、`sim_profile_file`：选择内置或外部仿真 profile。
- `sensor_overrides`：覆盖传感器组，例如 `camera=true,depth=false`。
- `use_moveit`、`rviz`、`headless`、`use_sim_time`：均支持 `auto|true|false`。

## 常见命令

```bash
ros2 launch robot_sim_bringup sim.launch.py
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=mock
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=full
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=light sensor_overrides:=camera=true
```
