# robot_sim 开发者文档

`robot_sim` 是一个 ROS 2 Humble 机器人仿真与验收系统。根目录 README 面向使用者，说明如何安装、运行和接入机器人；本目录面向开发者，系统性说明仓库边界、模块职责、配置模型、运行链路和维护工作流。

当前仓库只维护 `schema: 3` robot domain。通用 `schema: 4` ROS 2 pipeline 验证已经迁移到同级项目 `robot_validation`，本仓库遇到 `schema: 4` case 会直接提示切换项目。

## 系统边界

`robot_sim` 负责把机器人仿真验收需要的资产和运行时串起来：

| 边界 | 本仓库负责 |
| --- | --- |
| 机器人资产 | URDF/xacro、mesh、传感器挂载、ros2_control 标签、MoveIt 配置 |
| 仿真资产 | Gazebo world、scene、object、workspace、world preset、SDF 生成 |
| 运行入口 | `sim.launch.py`、`run_case`、`profile_lint`、`scaffold_robot`、Debian wrapper |
| 验收执行 | profile lint、URDF 渲染校验、Gazebo/控制/MoveIt/传感器等待、任务 runner、报告输出 |
| 扩展接入 | 外部 package 发现、外部 profile/scene/case、legacy welding/FANUC module validation |

`robot_sim` 不负责生产控制器，不替代真实机器人安全链路，也不继续承载通用 pipeline schema v4 验证。

## 开发心智模型

一次验收运行可以按四层理解：

```text
配置层
  sim_profile + scene + world_preset + validation_case

资产层
  description + control + moveit_config + scenarios + sensors + integrations

运行层
  run_case / sim.launch.py -> sim_launch_builder -> Gazebo / ros2_control / MoveIt / bridge
                                                -> optional standalone receivers

验收层
  profile_lint -> smoke_helper waits -> task_runners -> module_runner/adapters -> metrics/report/artifacts
```

主要数据流：

```text
validation_case
  -> resolve profile and scene
  -> apply CLI overrides, scene variant, scene params, sensor overrides
  -> write effective_case.yaml and effective_profile.yaml
  -> render robot.urdf and build Gazebo world
  -> launch runtime according to mode
  -> run task-specific checks
  -> collect logs, metrics, rosbag, report.md, report.html
```

开发时以 schema、loader 和 CLI `--help` 为契约真相源：

| 契约 | 真相源 |
| --- | --- |
| profile/case 字段 | `src/core/robot_sim_bringup/schemas/` 与对应 loader |
| scene/world 字段 | `src/core/robot_sim_scenarios/schemas/` 与 `schema_validation.py` |
| 命令参数 | 各命令的 `--help` 与 `launch/` 中的 `DeclareLaunchArgument` |
| 内置资产 | `examples/robot_arm/robot_sim/`、`integrations/welding/robot_sim/` |
| CI 实际覆盖 | `.github/workflows/ci.yml` 与 `.github/workflows/simulation-smoke.yml` |

## 核心模块

| 路径 | 职责 |
| --- | --- |
| `src/core/robot_sim_bringup/` | CLI、launch、schema 校验、registry、运行编排、profile lint、smoke helper、任务 runner、scaffold |
| `src/core/robot_sim_description/` | Panda/Fanuc 机器人描述、mesh、xacro macro、传感器挂载、Gazebo 与 ros2_control 标签 |
| `src/core/robot_sim_control/` | ros2_control controller 配置 |
| `src/core/robot_sim_moveit_config/` | SRDF、kinematics、OMPL、MoveIt controller、RViz 和 MoveIt launch |
| `src/core/robot_sim_scenarios/` | scene/world preset schema、场景库、SDF/world 生成、场景参数化 |
| `src/sensors/` | 可独立启动的 camera、depth、lidar、IMU receiver 与健康诊断；主仿真不会自动启动 receiver |
| `src/interfaces/` | simulation 和 task 相关 ROS message/service 类型声明；当前主运行链不提供对应 server |
| `examples/robot_arm/` | 内置 Panda/Fanuc profile 和 validation case |
| `integrations/` | welding legacy 集成资产；`auto_cover` 当前只有预留目录，没有可运行 case |
| `src/vendor/gz_ros2_control/` | Gazebo Harmonic 使用的 ros2_control overlay |
| `packaging/` | Debian 打包脚本和安装后命令 wrapper |

更细的代码地图见 [architecture/maintainer-code-map.md](architecture/maintainer-code-map.md)。

## 配置模型

`schema: 3` 下有四类核心 YAML：

| 类型 | 来源 | 作用 |
| --- | --- | --- |
| `sim_profile` | `robot_sim/profiles/*.yaml` | 描述机器人、控制、MoveIt、传感器、bridge、world、layout 和 capability |
| `scene` | `robot_sim/scenes/*.yaml` 或 `robot_sim_scenarios/scenes/*.yaml` | 描述工位区域、对象、workspace、参数、variant 和 generator |
| `world_preset` | `robot_sim_scenarios/world_presets/*.yaml` | 组合基础 world 和 legacy/base world 资产 |
| `validation_case` | `robot_sim/validation_cases/*.yaml` | 描述启动参数、场景、任务族、planning scene、期望指标、adapter 和产物 |

解析和发现规则集中在 `robot_sim_bringup.common.registry`：

```text
share/<pkg>/robot_sim/profiles/*.yaml
share/<pkg>/robot_sim/validation_cases/*.yaml
share/<pkg>/robot_sim/scenes/*.yaml
```

内置 profile/case 安装在 `robot_sim_bringup` share 下的 `examples/` 与 `integrations/`；通用 scene 安装在 `robot_sim_scenarios` share 下。

配置细节见 [configuration/settings.md](configuration/settings.md)。

## 运行入口

| 入口 | 面向场景 |
| --- | --- |
| `ros2 launch robot_sim_bringup sim.launch.py` | 手动启动单个仿真 profile |
| `ros2 run robot_sim_bringup run_case` | 执行完整 validation case 并生成产物 |
| `ros2 run robot_sim_bringup profile_lint` | 单独校验 sim_profile |
| `ros2 run robot_sim_bringup scaffold_robot` | 生成外部机器人仿真包模板 |
| `ros2 run robot_sim_bringup migrate_config` | 将支持的旧配置迁移到 v3 |
| `robot-sim ...` | Debian 包安装后的用户命令 wrapper |

仿真模式由 `src/core/robot_sim_bringup/config/sim_modes.yaml` 定义：

| 模式 | 运行时组成 |
| --- | --- |
| `mock` | 不启动 Gazebo，关闭 sim time、MoveIt、RViz 和传感器，适合快速 CI |
| `light` | 启动 headless Gazebo 与 ros2_control，默认关闭 MoveIt/RViz/传感器 |
| `full` | 启动 Gazebo、MoveIt/RViz、bridge 和传感器，适合人工调试与完整验收；receiver 仍需单独启动 |

## `run_case` 执行链路

```text
run_case
  -> 定位并校验 validation_case
  -> 加载 profile、scene，并应用 CLI 覆盖
  -> 创建 robot_sim_runs/<timestamp>_<case>_<profile>/
  -> 写入 manifest、effective_case、effective_profile
  -> 运行 profile_lint、渲染 URDF、执行 check_urdf
  -> 按 mode 启动 mock/light/full 运行时
  -> 等待 Gazebo spawn、joint state、controller、trajectory action、sensor topic、TF
  -> 根据 task.type 调用 task_runners
  -> module_validation 任务改由 legacy module_runner 启动外部模块和 adapter
  -> 可选录制 rosbag
  -> 输出 metrics.json、validation_metrics.json、report.md、report.html
  -> 清理进程；失败时保留产物用于复盘
```

产物结构和排查方式见 [logging/data-storage.md](logging/data-storage.md)。

## 扩展点

开发新能力时优先沿用这些扩展点：

| 需求 | 推荐改动位置 |
| --- | --- |
| 新机器人 | 外部 package 的 `robot_sim/profiles/`、description、control、MoveIt 配置 |
| 新场景 | `robot_sim/scenes/` 或 `robot_sim_scenarios/scenes/`，必要时扩展 SDF builder |
| 新验收任务 | `validation_case.task.type` 加任务配置，并在 `robot_domain/task_runners.py` 接入 runner |
| 新运行检查 | `robot_domain/sim_smoke_helper.py` 增加等待或探测命令 |
| 新配置字段 | 同步更新 schema、loader、lint、文档和测试 |
| 新外部模块集成 | 使用 `legacy_integrations/module_runner.py` 与 `module_adapter.py` 的 adapter 模式 |
| 新发布形态 | 修改 `packaging/` 和 CI workflow，保持 CLI wrapper 与 ROS entrypoint 一致 |

新增或修改公开行为时，同一变更应同时包含实现、schema/loader、测试和本目录文档；不要只更新 YAML 示例。

## 测试与维护

常用本地验证：

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic

colcon build --symlink-install --allow-overriding gz_ros2_control --packages-select \
  gz_ros2_control \
  robot_sim_description robot_sim_control robot_sim_scenarios robot_sim_moveit_config \
  robot_sim_sensor_camera robot_sim_sensor_depth robot_sim_sensor_lidar robot_sim_sensor_imu \
  robot_sim_bringup robot_task_interfaces simulation_interfaces

source install/setup.bash

colcon test --packages-select robot_sim_bringup robot_sim_scenarios
ros2 run robot_sim_bringup run_case --case empty_motion --mode mock --no-rosbag --output-dir robot_sim_runs --timeout 120
```

测试策略、CI 和发布流程分别见：

- [workflow/testing.md](workflow/testing.md)
- [workflow/ci-cd.md](workflow/ci-cd.md)
- [guide/package-install.md](guide/package-install.md)

## 文档导航

- [快速上手](guide/quick-start.md)：构建、source、运行第一个 case。
- [环境依赖](guide/prerequisites.md)：ROS、Gazebo、MoveIt、系统依赖。
- [仿真方案](guide/simulation.md)：`mock`、`light`、`full` 和 profile 启动参数。
- [外部项目资产](guide/external-projects.md)：外部机器人包目录与发现规则。
- [外部模块接入](guide/external-modules.md)：legacy welding/FANUC module validation。
- [架构总览](architecture/README.md)：模块关系和设计重点。
- [模块全景](architecture/module-overview.md)：核心 ROS package 职责。
- [数据流](architecture/data-flow.md)：配置、launch、运行产物的流转。
- [仿真状态模型](architecture/state-model.md)：运行阶段与状态转换。
- [ROS API](interfaces/ros-api.md)：launch、topic、service、action 和命令入口。
- [故障排查](faq/troubleshooting.md)：构建、Gazebo、MoveIt、controller 和 sensor 常见问题。
