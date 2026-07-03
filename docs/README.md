# robot_sim 文档

> 工业机器人仿真验收与回归测试平台，面向 ROS 2 Humble、Gazebo Harmonic、ros2_control 和 MoveIt2。

`robot_sim` 的核心工作流是：选择一个机器人 profile，加载一个工况 scene，执行一个 validation case，然后输出可复查的验收产物。它适合把“模型能否启动、控制器是否正常、TF/传感器/MoveIt 是否达标、规划任务是否成功”固化成可以反复运行的工程检查。

## 适用场景

| 场景 | 用法 |
| --- | --- |
| 新机器人接入 | 用 scaffold 生成外部 package，补齐 description、control、MoveIt、sensor 和 smoke case |
| 日常仿真调试 | 用 `sim.launch.py` 的 `mock`、`light`、`full` 模式分别验证 launch、控制链和完整仿真 |
| 工业验收 | 用 `run_case` 执行 fixture-to-pallet、障碍避让、pick-place 等 validation case |
| 外部模块验收 | 用 `module_validation` 接入定位、纠偏、视觉、分拣等外部 ROS2 模块 |
| 回归测试 | 在 CI 或定时任务中保存 `robot_sim_runs/`，通过 `metrics.json` 和 `report.html` 比较结果 |
| 交付排查 | 通过 `logs/sim.launch.log`、step log、rosbag 和报告定位失败步骤 |

## 当前能力

| 能力域 | 已实现内容 |
| --- | --- |
| 配置契约 | `schema: 3` + `kind`，覆盖 `sim_profile`、`scene`、`world_preset`、`validation_case` |
| 内置机器人 | Panda、Fanuc M-20iD/12L、Fanuc 工业单元 |
| 场景库 | 空场、工业单元、桌面抓取、传送带分拣、货架料箱 |
| 任务族 | `empty_motion`、`obstacle_clearance`、`fixture_to_pallet`、`pick_place`、`sensor_calibration`、`conveyor_sorting`、`module_validation` |
| 验收指标 | 启动、controller active、joint state、TF、sensor Hz、MoveIt、控制误差、目标误差、TCP clearance |
| 产物 | manifest、effective YAML、URDF、日志、rosbag、metrics、Markdown/HTML 报告 |
| 扩展 | 外部 package 发现、scene 参数/variant/generator、机器人模板生成、配置迁移 |

## 一次验收会做什么

```text
run_case
  -> 读取并校验 case/profile/scene
  -> 应用 CLI 覆盖、scene variant 和 scene 参数
  -> 创建独立 run artifact 目录
  -> profile lint、URDF 渲染和 check_urdf
  -> 启动 Gazebo/MoveIt/传感器链路
  -> 等待 spawn、controller、joint state、TF、sensor topic
  -> 按 task.type 分发标准任务族 runner
  -> 可选启动外部模块和仿真 adapter，执行服务/topic 验收
  -> 录制 rosbag，生成 metrics.json、report.md、report.html
  -> 失败也保留 manifest、日志和报告
```

第一次使用建议从 [快速上手](guide/quick-start.md) 开始；需要深入配置时阅读 [配置说明](configuration/settings.md) 和 [测试验收](workflow/testing.md)。

## 快速导航

- [快速上手](guide/quick-start.md)
- [环境依赖](guide/prerequisites.md)
- [仿真运行](guide/simulation.md)
- [外部模块接入](guide/external-modules.md)
- [测试验收](workflow/testing.md)
- [配置说明](configuration/settings.md)
- [日志与产物](logging/data-storage.md)
- [ROS 2 录包](guide/rosbag-recording.md)
- [Deb 打包与 Release](guide/package-install.md)
- [产品路线图](roadmap.md)
- [故障排查](faq/troubleshooting.md)

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

## 目录一览

| 路径 | 说明 |
| --- | --- |
| `src/core/robot_sim_bringup/` | 仿真入口、profile loader、schema 校验、lint、run_case、scaffold 和 launch |
| `src/core/robot_sim_description/` | Panda/Fanuc 模型、xacro、mesh、传感器挂载和 ros2_control 标签 |
| `src/core/robot_sim_control/` | controller 配置 |
| `src/core/robot_sim_moveit_config/` | MoveIt2 和 RViz2 配置 |
| `src/core/robot_sim_scenarios/` | scene library、world preset、assets、schema 和 SDF 生成 |
| `src/sensors/` | camera、depth、lidar、imu receiver |
| `src/interfaces/` | 通用仿真和任务接口 |
| `src/vendor/gz_ros2_control/` | Gazebo Harmonic 的 ros2_control overlay |
| `docs/` | 本文档站点 |
| `packaging/` | deb 打包脚本 |
