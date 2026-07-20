# 仿真方案

`robot_sim_bringup` 通过 `sim_profile` 和 `sim_mode` 组合启动仿真。

## 模式

| 模式 | Gazebo | 控制链 | 传感器 | MoveIt/RViz | 场景 |
| --- | --- | --- | --- | --- | --- |
| `mock` | 否 | `mock_components/GenericSystem` | 否 | 默认关闭 | 快速验证 launch、controller 和 action |
| `light` | 是 | `gz_ros2_control/GazeboSimSystem` | 默认关闭 | 默认关闭 | 日常控制链调试 |
| `full` | 是 | `gz_ros2_control/GazeboSimSystem` | 默认开启 | 默认开启 | 传感器、规划和演示 |

表中的“传感器”指 Gazebo sensor 与 ROS bridge topic。健康诊断 receiver 是独立进程，
`sim.launch.py` 不会自动启动；需要时运行 `sensor_receivers.launch.py`。

## Profile

内置 profile：

```text
panda
fanuc_m20id12l
fanuc_m20id12l_industrial_cell
```

启动示例：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=panda sim_mode:=light
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l sim_mode:=full
```

外部 profile：

```bash
ros2 launch robot_sim_bringup sim.launch.py \
  sim_profile_file:=/path/to/custom_robot.yaml \
  sim_mode:=light
```

## 传感器开关

`full` 默认开启 profile 声明的传感器。可以用 `sensor_overrides` 覆盖：

```bash
ros2 launch robot_sim_bringup sim.launch.py \
  sim_mode:=light \
  sensor_overrides:=camera=true,depth=false,lidar=true,imu=true
```

## MoveIt

`full` 默认开启 MoveIt 与 RViz：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l sim_mode:=full
```

自动检查规划执行：

```bash
scripts/sim_smoke_test.sh --profile fanuc_m20id12l --mode full --with-moveit --timeout 120
```

如果只想快速验证控制链和 MoveIt，可先用 mock：

```bash
ros2 run robot_sim_bringup run_case --case empty_motion --mode mock --no-rosbag --output-dir robot_sim_runs
```

## 场景

场景由 `robot_sim_scenarios` 生成：

- `worlds/base/`：base world。
- `assets/`：桌子、目标物、障碍物等可复用 SDF。
- `world_presets/`：组合 base world 与 assets 的启动 preset。
- `scenes/`：可复用 scene library YAML。

## 验收用例

validation case 使用 `run_case` 执行，并生成报告、metrics、日志和 rosbag：

```bash
ros2 run robot_sim_bringup run_case --case empty_motion --output-dir robot_sim_runs
ros2 run robot_sim_bringup run_case --case industrial_fixture_to_pallet --output-dir robot_sim_runs
ros2 run robot_sim_bringup run_case --case industrial_obstacle_clearance --output-dir robot_sim_runs
```

更多说明见 [快速上手](quick-start.md) 和 [测试验收](../workflow/testing.md)。
