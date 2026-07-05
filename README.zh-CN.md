# robot_sim

[English](README.md) | [简体中文](README.zh-CN.md)

<p align="center">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-00A6A6" />
  <img alt="Gazebo Harmonic" src="https://img.shields.io/badge/Gazebo-Harmonic%20%7C%20gz%20sim%208-F57C00" />
  <img alt="MoveIt2" src="https://img.shields.io/badge/Planning-MoveIt2-2D6CDF" />
  <a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache--2.0-blue" /></a>
</p>

![robot_sim](docs/assets/cover.svg)

`robot_sim` 现在专注 `schema: 3` 机器人仿真与验收链路：Gazebo、MoveIt、
ros2_control、机器人 profile、scene、validation case、传感器和 legacy 焊接/FANUC 集成。

通用 `schema: 4` ROS2 pipeline 验证已经拆到同级项目 `../robot_validation`。

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

运行 v3 验收用例：

```bash
ros2 run robot_sim_bringup run_case \
  --case empty_motion \
  --output-dir robot_sim_runs \
  --timeout 120
```

启动交互式仿真：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=panda sim_mode:=light
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l_industrial_cell sim_mode:=full
```

## 内置验收用例

| Case | Profile | Scene | 用途 |
| --- | --- | --- | --- |
| `empty_motion` | `panda` | `debug_empty` | 最小 MoveIt 规划与执行验收 |
| `industrial_obstacle_clearance` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | Fanuc 工业障碍避让 |
| `industrial_fixture_to_pallet` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | fixture-to-pallet 工业运动验收 |
| `industrial_planning_goal` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | 工业目标点规划 smoke |
| `panda_pick_place` | `panda` | `tabletop_pick_place` | pick-place 规划验收 |
| `sensor_calibration` | `panda` | `tabletop_pick_place` | 多视角传感器标定流程 |
| `conveyor_sorting` | `panda` | `conveyor_sorting` | 传送带分拣流程 |
| `weld_pre_positioning_scan_and_move` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | 外部焊前 3D 定位 + MoveIt jog |
| `weld_2d_lateral_correction_dry_run` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | 外部 2D 焊缝纠偏干运行 |

## 配置模型

`robot_sim` 校验以下 v3 YAML 契约：

| 配置类型 | 描述内容 |
| --- | --- |
| `sim_profile` | 机器人描述、控制、MoveIt、传感器、bridge、world、layout 和 capability |
| `scene` | 工况区域、对象、workspace、参数、variant 和 generator |
| `world_preset` | legacy/base world 资产组合 |
| `validation_case` | 启动参数、场景、任务族、planning scene、期望指标、adapter 和产物 |

外部机器人包路径：

```text
share/<pkg>/robot_sim/profiles/*.yaml
share/<pkg>/robot_sim/validation_cases/*.yaml
share/<pkg>/robot_sim/scenes/*.yaml
```

如果把 `schema: 4` case 传给 `robot_sim_bringup run_case`，命令会失败并提示改用
`robot_validation`。

## 机器人接入模板

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
robot-sim sim_profile:=fanuc_m20id12l sim_mode:=full
```

## 许可证

`robot_sim` 使用 [Apache License 2.0](LICENSE)。
