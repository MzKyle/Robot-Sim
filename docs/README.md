# robot_sim 文档

![robot_sim](assets/cover.svg)

`robot_sim` 是工业机器人仿真验收与回归测试平台。它的核心工作流是：选择一个
机器人 `sim_profile`，加载一个工况 `scene`，执行一个 `validation_case`，然后
输出可复查的报告、指标、日志和 rosbag。

这个项目不是单纯打开 Gazebo 的 demo，也不是生产机器人控制器。它面向工程验收：
回答“机器人模型、控制链、TF、传感器、MoveIt、业务模块和场景任务是否能稳定跑通”。

## 适用场景

| 场景 | 典型用法 |
| --- | --- |
| 新机器人接入 | 用 `scaffold_robot` 生成外部 ROS package，补齐 description、control、MoveIt、sensor、profile 和 smoke case |
| 日常仿真调试 | 用 `sim.launch.py` 的 `mock`、`light`、`full` 模式分别检查 launch、控制链和完整仿真 |
| 工业验收 | 用 `run_case` 执行障碍避让、fixture-to-pallet、pick-place、传感器标定、传送带分拣等 case |
| 外部模块验收 | 用 `module_validation` 接入定位、纠偏、视觉、分拣等外部 ROS2 模块 |
| 回归测试 | 在 CI 或定时任务中保存 `robot_sim_runs/`，对比 `metrics.json` 和 `report.html` |
| 交付排查 | 通过 `manifest.json`、step log、adapter log、rosbag 和报告定位失败步骤 |

## 当前能力

| 能力域 | 已实现内容 |
| --- | --- |
| 仿真模式 | `mock`、`light`、`full` |
| 内置机器人 | Panda、Fanuc M-20iD/12L、Fanuc 工业单元 |
| 场景库 | 空场、工业单元、桌面抓取、传送带分拣、货架料箱 |
| 配置契约 | `schema: 3` + `kind`，覆盖 `sim_profile`、`scene`、`world_preset`、`validation_case` |
| 标准任务族 | `empty_motion`、`obstacle_clearance`、`fixture_to_pallet`、`pick_place`、`sensor_calibration`、`conveyor_sorting` |
| 外部模块任务 | `module_validation`，支持启动外部 launch/command、adapter、服务动作和 topic 断言 |
| Adapter | TF 到 TCP pose、MoveIt pose service、`/scan_3d` dataset/replay、合成焊缝视觉、loop motion services |
| 验收指标 | 启动、controller active、joint state、TF、sensor Hz、MoveIt、控制误差、目标误差、TCP clearance |
| 产物 | manifest、effective YAML、URDF、日志、rosbag、metrics、Markdown/HTML 报告 |
| 扩展 | 外部 package 发现、scene 参数/variant/generator、机器人模板生成、v2 到 v3 配置迁移 |

## 一次 `run_case` 会做什么

```text
run_case
  -> 读取并校验 case/profile/scene
  -> 应用 CLI 覆盖、scene variant、scene 参数和传感器覆盖
  -> 创建独立 robot_sim_runs/<timestamp>_<case>_<profile>/ 目录
  -> 写入 manifest、effective_case、effective_profile
  -> profile lint、URDF 渲染和 check_urdf
  -> 启动 Gazebo、ros2_control、MoveIt、bridge、传感器 receiver
  -> 等待 spawn、joint state、controller active、trajectory action、sensor topic、TF
  -> 按 task.type 分发标准任务族 runner
  -> 可选启动外部模块和仿真 adapter，执行服务/topic 验收
  -> 录制 rosbag
  -> 汇总 metrics.json、validation_metrics.json、report.md、report.html
  -> 清理进程；失败也保留可复查产物
```

## 内置用例一览

| Case | Profile | Scene | Task | 执行方式 |
| --- | --- | --- | --- | --- |
| `empty_motion` | `panda` | `debug_empty` | `empty_motion` | MoveIt plan + execute |
| `industrial_obstacle_clearance` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | `obstacle_clearance` | MoveIt plan + execute |
| `industrial_fixture_to_pallet` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | `fixture_to_pallet` | MoveIt plan + execute |
| `industrial_planning_goal` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | `obstacle_clearance` | MoveIt plan + execute |
| `panda_pick_place` | `panda` | `tabletop_pick_place` | `pick_place` | 规划验收，默认不执行轨迹 |
| `sensor_calibration` | `panda` | `tabletop_pick_place` | `sensor_calibration` | 规划/传感器验收，默认不执行轨迹 |
| `conveyor_sorting` | `panda` | `conveyor_sorting` | `conveyor_sorting` | 规划/业务事件验收，默认不执行轨迹 |
| `weld_pre_positioning_scan_and_move` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | `module_validation` | 焊前 3D 定位，dataset `/scan_3d` + MoveIt jog |
| `weld_2d_lateral_correction_dry_run` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | `module_validation` | 2D 纠偏干运行，合成视觉 topic |

## 快速导航

- [快速上手](guide/quick-start.md)：从源码构建并跑第一个 `empty_motion` 报告。
- [环境依赖](guide/prerequisites.md)：ROS、Gazebo、MoveIt、Python 依赖和常见系统配置。
- [仿真运行](guide/simulation.md)：`mock`、`light`、`full` 模式、profile、传感器和 MoveIt。
- [外部模块接入](guide/external-modules.md)：`module_validation`、adapter、焊前定位和 2D 纠偏参考接入。
- [测试验收](workflow/testing.md)：单元测试、profile lint、smoke、validation case 和报告指标。
- [配置说明](configuration/settings.md)：`schema: 3` 配置结构、scene 参数、validation case 字段。
- [日志与产物](logging/data-storage.md)：`robot_sim_runs/` 目录结构、rosbag、报告和清理策略。
- [ROS API](interfaces/ros-api.md)：launch、topic、service、action 和命令入口。
- [Deb 打包与 Release](guide/package-install.md)：本地 deb 构建、安装和发布。
- [产品路线图](roadmap.md)：已落地阶段与后续计划。
- [故障排查](faq/troubleshooting.md)：常见构建、Gazebo、MoveIt、controller、sensor 问题。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `src/core/robot_sim_bringup/` | 仿真入口、profile loader、schema 校验、lint、run_case、module runner、scaffold 和 launch |
| `src/core/robot_sim_description/` | Panda/Fanuc 模型、xacro、mesh、传感器挂载和 ros2_control 标签 |
| `src/core/robot_sim_control/` | controller 配置 |
| `src/core/robot_sim_moveit_config/` | MoveIt2、SRDF、kinematics、OMPL、controller 和 RViz 配置 |
| `src/core/robot_sim_scenarios/` | scene library、world preset、assets、schema 和 SDF 生成 |
| `src/sensors/` | camera、depth、lidar、imu receiver |
| `src/interfaces/` | 通用仿真和任务接口 |
| `src/vendor/gz_ros2_control/` | Gazebo Harmonic 的 ros2_control overlay |
| `docs/` | 文档站点 |
| `packaging/` | Debian 打包脚本 |

## 推荐命令

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic

colcon build --symlink-install --allow-overriding gz_ros2_control --packages-select \
  gz_ros2_control \
  robot_sim_description robot_sim_control robot_sim_scenarios robot_sim_moveit_config \
  robot_sim_sensor_camera robot_sim_sensor_depth robot_sim_sensor_lidar robot_sim_sensor_imu \
  robot_sim_bringup robot_task_interfaces simulation_interfaces

source install/setup.bash

ros2 run robot_sim_bringup run_case --case empty_motion --output-dir robot_sim_runs --timeout 120
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l_industrial_cell sim_mode:=full
```
