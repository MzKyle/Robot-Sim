# robot_sim

[English](README.md) | [简体中文](README.zh-CN.md)

<p align="center">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-00A6A6" />
  <img alt="Gazebo Harmonic" src="https://img.shields.io/badge/Gazebo-Harmonic%20%7C%20gz%20sim%208-F57C00" />
  <img alt="MoveIt2" src="https://img.shields.io/badge/Planning-MoveIt2-2D6CDF" />
  <a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache--2.0-blue" /></a>
</p>

![robot_sim](docs/assets/cover.svg)

`robot_sim` 是一个工业机器人仿真验收与回归测试平台，面向 ROS 2 Humble、
Gazebo Harmonic、ros2_control、MoveIt2 和仿真传感器。它把机器人 profile、工况
scene、任务用例、外部 ROS2 模块、日志、指标和 rosbag 串成一条可重复执行的验收链路。

[阅读完整文档](docs/README.md)

## 快速上手

1. 准备 Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic、MoveIt2、`colcon` 和
   `rosdep`。完整依赖见 [环境依赖](docs/guide/prerequisites.md)。
2. 拉取并构建工作空间：

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

3. 跑第一个验收用例：

```bash
ros2 run robot_sim_bringup run_case \
  --case empty_motion \
  --output-dir robot_sim_runs \
  --timeout 120
```

4. 打开报告：

```bash
latest_run="$(ls -td robot_sim_runs/*_empty_motion_panda | head -1)"
xdg-open "${latest_run}/report.html"
```

每次运行都会生成一个独立产物目录，里面包含 `manifest.json`、最终生效的 YAML、
`robot.urdf`、日志、rosbag、`metrics.json`、`report.md` 和 `report.html`。

## 这个项目能做什么

- 以 `mock`、`light`、`full` 三种模式启动 Panda 或 Fanuc M-20iD/12L 仿真。
- 验收 controller 状态、joint state、TF 完整性、传感器 topic 频率、MoveIt
  规划/执行、目标误差、控制误差和 TCP clearance。
- 运行空场运动、障碍避让、fixture-to-pallet、pick-place、传感器标定、传送带分拣
  和外部模块验收用例。
- 使用 scene variant 和参数化配置生成可复现的工业测试工况。
- 通过 adapter 接入外部 ROS2 模块，当前支持 TCP pose、`/scan_3d`、MoveIt pose jog、
  合成焊缝视觉和连续纠偏相关服务。
- 输出可人工复查、可交付归档、可 CI 检查的完整运行产物。
- 用 scaffold 生成外部机器人接入包，不需要把所有机器人配置都塞进本仓库。

## 内置验收用例

| Case | Profile | Scene | 用途 |
| --- | --- | --- | --- |
| `empty_motion` | `panda` | `debug_empty` | 最小 MoveIt 规划与执行验收 |
| `industrial_obstacle_clearance` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | Fanuc 工业障碍避让，应用 planning-scene objects |
| `industrial_fixture_to_pallet` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | fixture-to-pallet 工业运动验收 |
| `industrial_planning_goal` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | 工业目标点规划 smoke 验收 |
| `panda_pick_place` | `panda` | `tabletop_pick_place` | pick-place 规划验收，默认不执行轨迹 |
| `sensor_calibration` | `panda` | `tabletop_pick_place` | 多视角传感器标定流程验收 |
| `conveyor_sorting` | `panda` | `conveyor_sorting` | 传送带分拣流程验收 |
| `weld_pre_positioning_scan_and_move` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | 外部焊前 3D 定位，dataset `/scan_3d` + MoveIt jog |
| `weld_2d_lateral_correction_dry_run` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | 外部 2D 纠偏干运行，使用合成视觉 topic |

## 仿真与验收流程

- `sim.launch.py` 用于打开交互式仿真：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=panda sim_mode:=light
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l_industrial_cell sim_mode:=full
```

- `run_case` 用于执行完整验收闭环：

```bash
ros2 run robot_sim_bringup run_case --case industrial_fixture_to_pallet --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case industrial_obstacle_clearance --output-dir robot_sim_runs --timeout 120
```

- 外部模块验收需要先 source 外部工作空间：

```bash
source /home/kyle/sany/ROS2_Motion_Planner/install/setup.bash
ros2 run robot_sim_bringup run_case --case weld_pre_positioning_scan_and_move --output-dir robot_sim_runs --timeout 180
```

焊前定位用例会优先读取 `/home/kyle/sany/data/3dcamera_2d_img`。如果有同名 `.npz`
点云，就用真实图片和真实点云回放；如果只有图片，就生成确定性的合成点云；如果目录中
没有可用数据，则回退到内置 replay 采集记录。

## 配置模型

`robot_sim` 使用 `schema: 3` YAML 契约，并通过 JSON Schema 校验：

| 配置类型 | 描述内容 |
| --- | --- |
| `sim_profile` | 机器人描述、控制、MoveIt、传感器、bridge、world、layout 和 capability |
| `scene` | 完整工况，包括区域、对象、workspace、参数、variant 和 generator |
| `world_preset` | legacy/base world 资产组合 |
| `validation_case` | 启动参数、场景、任务族、planning scene、期望指标、adapter 和产物 |

外部 package 使用以下路径即可被发现：

```text
share/<pkg>/robot_sim/profiles/*.yaml
share/<pkg>/robot_sim/validation_cases/*.yaml
share/<pkg>/robot_sim/suites/*.yaml
share/<pkg>/robot_sim/data_sources/*.yaml
share/<pkg>/robot_sim/adapters/*.yaml
```

旧的 `robot_sim/validation_suites` 路径仍然兼容。内置机器人示例位于
`examples/robot_arm`，焊接集成位于 `integrations/welding`。

## 机器人接入模板

生成一个外部机器人仿真包骨架：

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

## Debian 安装包

本地构建并安装 deb：

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
robot-sim scaffold-robot --package my_robot_sim --robot-name my_robot --output /tmp --joint-names joint_1 joint_2 joint_3 joint_4 joint_5 joint_6
robot-sim scaffold-system --package my_robot_sim --name minimal_system --output /tmp
robot-sim scaffold-case --package my_robot_sim --name smoke_case --system minimal_system --output /tmp
robot-sim scaffold-suite --package my_robot_sim --name smoke_suite --case smoke_case --output /tmp
robot-sim scaffold-adapter --package my_robot_sim --name smoke_adapter --output /tmp
robot-sim sim_profile:=panda sim_mode:=light
```

## 文档

- [文档首页](docs/README.md)
- [快速上手](docs/guide/quick-start.md)
- [仿真运行](docs/guide/simulation.md)
- [外部模块接入](docs/guide/external-modules.md)
- [测试验收](docs/workflow/testing.md)
- [配置说明](docs/configuration/settings.md)
- [日志与产物](docs/logging/data-storage.md)
- [Deb 打包与 Release](docs/guide/package-install.md)
- [产品路线图](docs/roadmap.md)
- [故障排查](docs/faq/troubleshooting.md)

## 许可证

`robot_sim` 使用 [Apache License 2.0](LICENSE)。第三方资源说明见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
