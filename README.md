# robot_sim

<p align="center">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-00A6A6" />
  <img alt="Gazebo Harmonic" src="https://img.shields.io/badge/Gazebo-Harmonic%20%7C%20gz%20sim%208-F57C00" />
  <img alt="MoveIt2" src="https://img.shields.io/badge/Planning-MoveIt2-2D6CDF" />
  <a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache--2.0-blue" /></a>
</p>

`robot_sim` 是一个面向工业机器人项目的仿真验收与回归测试平台。它把机器人 profile、Gazebo 工况场景、ros2_control、MoveIt2、传感器桥接和 validation case 串成一条可重复执行的验收链路：一次命令启动仿真、检查控制器/TF/传感器、执行规划任务、录 rosbag，并生成可交付的报告。

项目当前内置 Panda 与 Fanuc M-20iD/12L，可直接用于空场运动、障碍避让、fixture-to-pallet、pick-place、传感器标定和 conveyor sorting 等典型任务的仿真验收。

## 适合谁用

- 机器人应用工程师：验证新机器人模型、控制器、MoveIt 配置和工况场景是否能跑通。
- 仿真与算法团队：把场景、任务和 pass/fail 指标固化为可回归的用例。
- 项目交付团队：用 `report.html`、`metrics.json`、rosbag 和日志说明一次仿真验收到底通过了什么。
- CI 维护者：把轻量 mock/full smoke 和工业 validation case 放进定时回归。

`robot_sim` 不是 Web UI，也不是生产控制器。它的核心目标是让机器人仿真从“能打开 demo”变成“能被验收、能被复跑、能定位失败”。

## 快速上手

### 1. 准备环境

需要 Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic、MoveIt2、`colcon` 和 `rosdep`。完整依赖见 [docs/guide/prerequisites.md](docs/guide/prerequisites.md)。

```bash
git clone https://github.com/MzKyle/robot_sim.git robot_sim
cd robot_sim

source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
```

### 2. 构建工作空间

```bash
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

### 3. 跑第一个验收用例

```bash
ros2 run robot_sim_bringup run_case \
  --case empty_motion \
  --output-dir robot_sim_runs \
  --timeout 120
```

运行完成后会生成独立目录：

```text
robot_sim_runs/<UTC timestamp>_empty_motion_panda/
  manifest.json
  metrics.json
  report.md
  report.html
  robot.urdf
  effective_case.yaml
  effective_profile.yaml
  logs/
  rosbag/
```

打开最新报告：

```bash
latest_run="$(ls -td robot_sim_runs/*_empty_motion_panda | head -1)"
xdg-open "${latest_run}/report.html"
```

报告会告诉你：哪一步失败、controller 是否 active、TF 是否完整、每个传感器 topic 频率、MoveIt 是否成功、执行耗时、日志和 rosbag 在哪里。

### 4. 启动交互式仿真

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=panda sim_mode:=light
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l_industrial_cell sim_mode:=full
```

`light` 适合日常控制链调试；`full` 会启动 Gazebo、传感器、MoveIt 和 RViz，适合完整仿真和人工检查。

## 已内置能力

| 能力 | 当前状态 |
| --- | --- |
| 仿真模式 | `mock`、`light`、`full` 三档 |
| 机器人 profile | Panda、Fanuc M-20iD/12L、Fanuc 工业单元 |
| 场景库 | `debug_empty`、`industrial_cell`、`tabletop_pick_place`、`conveyor_sorting`、`shelf_bin_picking` |
| 传感器 | RGB、深度、点云、2D LaserScan、3D lidar、IMU |
| 验收指标 | 启动、controller active、joint state、TF、sensor Hz、MoveIt 规划/执行、控制误差、TCP clearance |
| 配置契约 | `schema: 3` + JSON Schema，覆盖 `sim_profile`、`scene`、`world_preset`、`validation_case` |
| 报告产物 | `manifest.json`、日志、URDF、rosbag、`metrics.json`、`report.md`、`report.html` |
| 可扩展性 | 外部 profile/case/scene package 发现，机器人接入 scaffold，v2 到 v3 迁移工具 |

## 内置验收用例

| Case | Profile | Scene | 任务族 | 默认执行方式 |
| --- | --- | --- | --- | --- |
| `empty_motion` | `panda` | `debug_empty` | `empty_motion` | MoveIt plan + execute |
| `industrial_obstacle_clearance` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | `obstacle_clearance` | MoveIt plan + execute |
| `industrial_fixture_to_pallet` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | `fixture_to_pallet` | MoveIt plan + execute |
| `industrial_planning_goal` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | `obstacle_clearance` | MoveIt plan + execute |
| `panda_pick_place` | `panda` | `tabletop_pick_place` | `pick_place` | 规划验收，`task.moveit.execute: false` |
| `sensor_calibration` | `panda` | `tabletop_pick_place` | `sensor_calibration` | 规划/传感器验收，`task.moveit.execute: false` |
| `conveyor_sorting` | `panda` | `conveyor_sorting` | `conveyor_sorting` | 规划/业务事件验收，`task.moveit.execute: false` |

工业 Fanuc 用例和 `empty_motion` 会实际执行轨迹；Panda 的 pick-place、sensor calibration、conveyor sorting 当前默认做规划、TF、传感器和业务步骤级验收，适合在不引入额外 Gazebo 插件或视觉库的前提下做回归。

## 常用命令

运行验收：

```bash
ros2 run robot_sim_bringup run_case --case industrial_fixture_to_pallet --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case industrial_obstacle_clearance --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case panda_pick_place --scene-variant extra_workpieces
```

校验 profile：

```bash
ros2 run robot_sim_bringup profile_lint --profile panda --mode full --require-moveit --require-receivers
ros2 run robot_sim_bringup profile_lint --profile fanuc_m20id12l_industrial_cell --mode full --require-moveit --require-receivers
```

生成新机器人接入模板：

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

迁移旧配置：

```bash
ros2 run robot_sim_bringup migrate_config --input old.yaml --output new.yaml
```

Deb 安装后：

```bash
robot-sim-check
robot-sim run-case --case industrial_fixture_to_pallet
robot-sim scaffold-robot --package my_robot_sim --robot-name my_robot --output /tmp --joint-names joint_1 joint_2 joint_3 joint_4 joint_5 joint_6
robot-sim sim_profile:=panda sim_mode:=light
```

## 接入自己的机器人

推荐不要把所有机器人都塞进本仓库。外部 ROS package 使用标准路径即可被发现：

```text
share/<pkg>/robot_sim/profiles/*.yaml
share/<pkg>/robot_sim/validation_cases/*.yaml
share/<pkg>/robot_sim/scenes/*.yaml
```

运行外部配置：

```bash
ros2 run robot_sim_bringup run_case \
  --profile-package my_robot_sim \
  --profile my_robot \
  --case-package my_robot_sim \
  --case smoke_empty_motion
```

显式文件路径也支持：

```bash
ros2 run robot_sim_bringup run_case \
  --profile-file /path/to/profile.yaml \
  --case /path/to/case.yaml \
  --scene /path/to/scene.yaml
```

## 文档

- [快速上手](docs/guide/quick-start.md)
- [环境依赖](docs/guide/prerequisites.md)
- [仿真运行](docs/guide/simulation.md)
- [测试验收](docs/workflow/testing.md)
- [配置说明](docs/configuration/settings.md)
- [日志与产物](docs/logging/data-storage.md)
- [Deb 打包与 Release](docs/guide/package-install.md)
- [产品路线图](docs/roadmap.md)
- [故障排查](docs/faq/troubleshooting.md)

## License

本项目使用 [Apache License 2.0](LICENSE)。第三方资源说明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
