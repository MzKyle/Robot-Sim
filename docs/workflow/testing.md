# 测试验收

`robot_sim` 的测试分三层：单元测试保证 schema/loader/runner 逻辑可靠；profile lint 和 smoke test 保证机器人接入闭环；validation case 生成可交付的验收产物。

## 推荐顺序

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
source install/setup.bash

colcon test --packages-select robot_sim_bringup robot_sim_scenarios
colcon test-result --verbose

ros2 run robot_sim_bringup profile_lint --profile panda --mode full --require-moveit --require-receivers
ros2 run robot_sim_bringup profile_lint --profile fanuc_m20id12l_industrial_cell --mode full --require-moveit --require-receivers

ros2 run robot_sim_bringup run_case --case empty_motion --output-dir robot_sim_runs --timeout 120
```

## 单元测试

```bash
colcon test --packages-select robot_sim_bringup robot_sim_scenarios
colcon test-result --verbose
```

覆盖内容包括：

- 内置 `sim_profile`、`scene`、`world_preset`、`validation_case` 的 schema v3 校验。
- v1/v2 配置拒绝加载，并给出 `migrate_config` 迁移提示。
- scene 参数、variant、generator 在固定 seed 下可复现。
- 外部 package registry 能发现 profile/case/scene。
- scaffold 生成的 package 文件能通过 schema 和 profile lint。
- task runner registry 能按 `task.type` 分发六类任务族。
- runner 失败时仍生成 artifact、manifest、metrics 和报告。

## Profile Lint

Profile lint 用来验证机器人接入配置是否完整，不启动完整验收流程。

```bash
ros2 run robot_sim_bringup profile_lint --profile panda --mode full --require-moveit --require-receivers
ros2 run robot_sim_bringup profile_lint --profile fanuc_m20id12l --mode full --require-moveit --require-receivers
ros2 run robot_sim_bringup profile_lint --profile fanuc_m20id12l_industrial_cell --mode full --require-moveit --require-receivers
```

外部 package：

```bash
ros2 run robot_sim_bringup profile_lint \
  --profile-package my_robot_sim \
  --profile my_robot \
  --mode full \
  --require-moveit \
  --require-receivers
```

Lint 会检查 schema、ROS package、xacro、controller、MoveIt、bridge、sensor receiver、world source 和 smoke 规则。

## Smoke Test

Smoke test 是 shell 层的快速仿真检查，适合 CI 或调试 launch。

快速 mock：

```bash
scripts/sim_smoke_test.sh --profile panda --mode mock --timeout 60
```

完整 Gazebo：

```bash
scripts/sim_smoke_test.sh --profile panda --mode full --timeout 120
scripts/sim_smoke_test.sh --profile fanuc_m20id12l --mode full --with-moveit --timeout 120
```

保留失败日志：

```bash
scripts/sim_smoke_test.sh --profile fanuc_m20id12l --mode full --with-moveit --keep-logs
```

## Validation Case

`run_case` 是用户级验收入口。它读取 `validation_case`，启动仿真，执行标准任务族，录制 rosbag，并生成报告。

```bash
ros2 run robot_sim_bringup run_case \
  --case industrial_fixture_to_pallet \
  --output-dir robot_sim_runs \
  --timeout 120
```

常用参数：

| 参数 | 说明 |
| --- | --- |
| `--case <name|path>` | 内置 case 名或 YAML 路径，必填 |
| `--case-package <pkg>` | 从外部包 `share/<pkg>/robot_sim/validation_cases/` 查找 case |
| `--profile <name>` | 覆盖 case 中的 `launch.profile` |
| `--profile-package <pkg>` | 从外部包 `share/<pkg>/robot_sim/profiles/` 查找 profile |
| `--profile-file <path>` | 显式指定 profile YAML |
| `--scene <name|path>` | 覆盖 case 中的 scene |
| `--scene-package <pkg>` | 从外部包 `share/<pkg>/robot_sim/scenes/` 查找 scene |
| `--scene-variant <name>` | 应用 scene variant |
| `--scene-param name=value` | 覆盖 scene 参数，可重复 |
| `--mode mock|light|full` | 覆盖启动模式 |
| `--sensor-overrides ...` | 覆盖传感器开关 |
| `--timeout <sec>` | 覆盖 case timeout |
| `--rosbag-duration <sec>` | rosbag 录制时长 |
| `--no-rosbag` | 关闭本次 rosbag |
| `--keep-sim` | 验收结束后保留仿真进程 |

## Run Case 流程

一次 `run_case` 会按顺序执行：

1. 读取 case/profile/scene，并执行 JSON Schema 校验和语义校验。
2. 应用 CLI 覆盖、scene variant、scene 参数和传感器覆盖。
3. 创建 `robot_sim_runs/<UTC timestamp>_<case>_<profile>/`。
4. 写入 `manifest.json`、`effective_case.yaml`、`effective_profile.yaml`。
5. 执行 profile lint。
6. 渲染 URDF 到 `robot.urdf` 并运行 `check_urdf`。
7. 启动 `sim.launch.py`，日志写入 `logs/sim.launch.log`。
8. 等待 Gazebo spawn、joint state、controller active、trajectory action、sensor topic 和 TF。
9. 按 `task.type` 分发 task runner，并执行 MoveIt/planning scene/业务事件验收。
10. 按 case 配置录 rosbag。
11. 汇总 `metrics.json`、`validation_metrics.json`、`report.md`、`report.html`。
12. 清理仿真进程；如果传入 `--keep-sim`，则保留进程用于人工检查。

失败时流程会尽量继续写出 manifest、metrics、report 和日志路径，方便定位第一个失败步骤。

## 内置 Case

| Case | Profile | Scene | Task | 默认执行方式 |
| --- | --- | --- | --- | --- |
| `empty_motion` | `panda` | `debug_empty` | `empty_motion` | MoveIt plan + execute |
| `industrial_obstacle_clearance` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | `obstacle_clearance` | MoveIt plan + execute |
| `industrial_fixture_to_pallet` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | `fixture_to_pallet` | MoveIt plan + execute |
| `industrial_planning_goal` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | `obstacle_clearance` | MoveIt plan + execute |
| `panda_pick_place` | `panda` | `tabletop_pick_place` | `pick_place` | 规划验收，`execute: false` |
| `sensor_calibration` | `panda` | `tabletop_pick_place` | `sensor_calibration` | 规划/传感器验收，`execute: false` |
| `conveyor_sorting` | `panda` | `conveyor_sorting` | `conveyor_sorting` | 规划/业务事件验收，`execute: false` |

批量手动验证：

```bash
ros2 run robot_sim_bringup run_case --case empty_motion --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case industrial_obstacle_clearance --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case industrial_fixture_to_pallet --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case panda_pick_place --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case sensor_calibration --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case conveyor_sorting --output-dir robot_sim_runs --timeout 120
```

## 报告指标

`metrics.json` 和报告包含这些关键字段：

| 指标 | 含义 |
| --- | --- |
| `passed` | 本次最终通过/失败 |
| `steps[]` | 每个阶段的状态、耗时、日志路径和返回码 |
| `sensor_hz` | 每个期望 topic 的频率、样本数和是否达标 |
| `tf_ok` | TF 树完整性检查结果 |
| `plan_success_rate` | MoveIt 多阶段目标规划成功率 |
| `planning_time_sec` / `execution_time_sec` | 规划和执行耗时 |
| `goal_position_error_m` | 末端或目标点误差 |
| `max_controller_error_rad` / `peak_controller_error_rad` | controller 跟踪误差 |
| `min_tcp_clearance_m` | TCP 与障碍物的最小 clearance |
| `moveit_error_code` | MoveIt 返回码 |
| `business_actions` | task runner 声明的业务步骤 |

## 产物结构

```text
robot_sim_runs/<timestamp>_<case>_<profile>/
  manifest.json
  effective_case.yaml
  effective_profile.yaml
  robot.urdf
  metrics.json
  validation_metrics.json
  report.md
  report.html
  logs/
    sim.launch.log
    profile_lint.log
    check_urdf.log
    sensor_hz.log
    tf_tree.log
    moveit.log
    validation_case.log
  rosbag/
    metadata.yaml
```

`report.md` 和 `report.html` 来自同一个报告模型。CI 推荐上传整个 run directory，而不是只上传日志。

## Gazebo Plugin 检查

```bash
gz plugin -p "$(ros2 pkg prefix gz_ros2_control)/lib/libgz_ros2_control-system.so" --info
```

输出应包含：

```text
gz_ros2_control::GazeboSimROS2ControlPlugin
```
