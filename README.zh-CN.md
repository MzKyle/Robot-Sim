# robot_sim

[English](README.md) | [简体中文](README.zh-CN.md)

<p align="center">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-00A6A6" />
  <img alt="Gazebo Harmonic" src="https://img.shields.io/badge/Gazebo-Harmonic%20%7C%20gz%20sim%208-F57C00" />
  <img alt="MoveIt2" src="https://img.shields.io/badge/Planning-MoveIt2-2D6CDF" />
  <a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache--2.0-blue" /></a>
</p>

![robot_sim](docs/assets/cover.svg)

`robot_sim` 是一个面向 ROS 2 机器人的仿真与验收工具集，用来检查机器人模型、控制器、MoveIt 配置、传感器、场景和任务流程能否稳定协同运行。

它不是只打开一次 Gazebo 的 demo，而是面向可重复的仿真验证。你可以启动交互式仿真、运行验收用例、收集运行产物，也可以为自己的机器人生成外部仿真包模板。

通用 `schema: 4` ROS 2 pipeline 验证已经拆到同级项目 `robot_validation`。当前仓库专注维护 `schema: 3` 机器人仿真域。

## 你可以用它做什么

| 目标 | 用法 |
| --- | --- |
| 启动机器人仿真 | 用 Panda 或 Fanuc profile 启动 Gazebo，可按需开启 MoveIt 和传感器 |
| 执行验收检查 | 运行 validation case，生成日志、指标、报告和可选 rosbag |
| 验证工业单元 | 检查障碍避让、fixture-to-pallet、规划目标和焊接集成干运行 |
| 测试传感器流程 | 运行相机、深度、激光雷达、IMU、标定和传送带分拣场景 |
| 接入自己的机器人 | 生成外部机器人包模板，补充 profile、scene 和 validation case |

## 环境要求

- Ubuntu，并已 source `/opt/ros/humble`
- Gazebo Harmonic / `gz sim 8`
- ROS 2 Humble 对应的 MoveIt 2 与 ros2_control
- `colcon`、`rosdep` 和常规 ROS 2 构建工具

完整环境检查见 [docs/guide/prerequisites.md](docs/guide/prerequisites.md)。

## 快速上手

```bash
git clone https://github.com/MzKyle/robot_sim.git robot_sim
cd robot_sim

source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic

colcon build --symlink-install \
  --allow-overriding gz_ros2_control \
  --packages-select \
    gz_ros2_control \
    robot_sim_description robot_sim_control robot_sim_scenarios \
    robot_sim_moveit_config \
    robot_sim_sensor_camera robot_sim_sensor_depth robot_sim_sensor_lidar robot_sim_sensor_imu \
    robot_sim_bringup robot_task_interfaces simulation_interfaces

source install/setup.bash
```

先运行一个不依赖 Gazebo 的快速验收用例：

```bash
ros2 run robot_sim_bringup run_case \
  --case empty_motion \
  --mode mock \
  --no-rosbag \
  --output-dir robot_sim_runs \
  --timeout 120
```

运行结果会写入 `robot_sim_runs/` 下的时间戳目录，包含生效配置、日志、指标和报告。

需要 Gazebo 时启动交互式仿真：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=panda sim_mode:=light
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l_industrial_cell sim_mode:=full
```

仿真模式：

| 模式 | 用途 |
| --- | --- |
| `mock` | 不启动 Gazebo，用于快速检查运行链路和产物生成 |
| `light` | 启动 headless Gazebo 和 ros2_control，默认关闭传感器 |
| `full` | 默认开启 Gazebo、MoveIt/RViz、bridge 和传感器 |

## 内置示例

机器人 profile：

- `panda`
- `fanuc_m20id12l`
- `fanuc_m20id12l_industrial_cell`

验收用例：

| Case | 检查内容 |
| --- | --- |
| `empty_motion` | 最小 MoveIt 规划与执行链路 |
| `industrial_obstacle_clearance` | Fanuc 工业障碍避让 |
| `industrial_fixture_to_pallet` | fixture-to-pallet 工业运动 |
| `industrial_planning_goal` | 工业目标点规划 smoke |
| `panda_pick_place` | Panda 桌面 pick-place 规划 |
| `sensor_calibration` | 多视角传感器标定流程 |
| `conveyor_sorting` | 传送带分拣流程 |
| `weld_pre_positioning_scan_and_move` | 焊前 3D 定位与 MoveIt jog |
| `weld_2d_lateral_correction_dry_run` | 2D 焊缝纠偏干运行 |

## 接入自己的机器人

使用 scaffold 命令生成符合 `robot_sim/` 目录约定的外部 ROS 包：

```bash
ros2 run robot_sim_bringup scaffold_robot \
  --package my_robot_sim \
  --robot-name my_robot \
  --output /tmp \
  --planning-group manipulator \
  --tool-link tool0 \
  --joint-names joint_1 joint_2 joint_3 joint_4 joint_5 joint_6 \
  --sensor-set camera,depth,lidar,imu \
  --with-gripper true
```

外部包会按以下路径被发现：

```text
share/<pkg>/robot_sim/profiles/*.yaml
share/<pkg>/robot_sim/validation_cases/*.yaml
share/<pkg>/robot_sim/scenes/*.yaml
```

## 文档

- 用户安装与运行指南：[docs/guide/quick-start.md](docs/guide/quick-start.md)
- 开发者系统总览：[docs/README.md](docs/README.md)
- 配置参考：[docs/configuration/settings.md](docs/configuration/settings.md)
- 架构说明：[docs/architecture/README.md](docs/architecture/README.md)
- 常见问题：[docs/faq/troubleshooting.md](docs/faq/troubleshooting.md)

## Debian 安装包

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
bash packaging/build_deb.sh
sudo apt install ./dist/robot-sim_0.1.0-1_amd64.deb
```

安装后的常用命令：

```bash
robot-sim-check
robot-sim run-case --case industrial_fixture_to_pallet
robot-sim migrate-config --input old.yaml --output new.yaml
robot-sim scaffold-robot --package my_robot_sim --robot-name my_robot --output /tmp --joint-names joint_1 joint_2 joint_3 joint_4 joint_5 joint_6
robot-sim sim_profile:=panda sim_mode:=light
```

## 许可证

`robot_sim` 使用 [Apache License 2.0](LICENSE)。
