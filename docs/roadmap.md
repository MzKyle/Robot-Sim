# 产品路线图

`robot_sim` 的产品定位是工业机器人仿真验收与回归测试平台，而不是单纯的仿真 demo。

## 第一阶段：可验收

目标是让一个机器人应用能够被配置化启动、执行、度量和归档。

- 以 `sim_profile`、`scene`、`world_preset`、`validation_case` 四类配置作为公开契约。
- 用 `validation_case` 描述启动参数、场景、MoveIt 目标、规划场景、期望指标和产物。
- 用 `run_case` 执行单个验收用例，并生成独立运行目录。
- 每次运行输出 `manifest.json`、`metrics.json`、`report.md`、`report.html`、日志、URDF 和 rosbag。
- 指标覆盖启动、controller active、TF、sensor Hz、MoveIt plan/execute、目标误差、控制误差和 TCP clearance。

推荐命令：

```bash
ros2 run robot_sim_bringup run_case \
  --case industrial_fixture_to_pallet \
  --output-dir robot_sim_runs \
  --timeout 120
```

安装 deb 后也可以使用：

```bash
robot-sim run-case --case industrial_obstacle_clearance
```

## 第二阶段：可复用

目标是让新机器人、新场景和新验收任务可以在外部 ROS package 中复用，而不是都塞进本仓库。

- 升级为 `schema: 3`；v1/v2 配置直接报错，并提示 `migrate_config`。
- `sim_profile` 显式声明 `metadata`、`capabilities`、`end_effector` 和 gripper。
- `scene` 支持 `parameters`、`variants` 和受控 `random_boxes` generator。
- `run_case` 支持 `--profile-package`、`--case-package`、`--scene-package`、`--scene-variant` 和 `--scene-param`。
- 标准任务族 runner registry 覆盖 `empty_motion`、`obstacle_clearance`、`fixture_to_pallet`、`pick_place`、`sensor_calibration`、`conveyor_sorting`。
- `scaffold_robot` 生成外部接入包骨架：description、control、MoveIt、sensor、smoke、profile、scene、validation case。

推荐命令：

```bash
ros2 run robot_sim_bringup scaffold_robot \
  --package my_robot_sim \
  --robot-name my_robot \
  --output /tmp \
  --planning-group manipulator \
  --tool-link tool0 \
  --joint-names joint_1 joint_2 joint_3 joint_4 joint_5 joint_6

ros2 run robot_sim_bringup run_case \
  --profile-package my_robot_sim \
  --profile my_robot \
  --case-package my_robot_sim \
  --case smoke_empty_motion
```

## 第三阶段：工程化

- 增加批量矩阵执行：profile × scene × validation case。
- 输出 JUnit 或等价 CI 报告，方便接入交付流水线。
- 增加 Humble/Jazzy 兼容矩阵。
- 增加 docker/devcontainer，降低环境搭建成本。
- 增强 rosbag、日志和指标的长期归档能力。
- 在不改变 CLI 契约的前提下补充 Web/HTML 汇总视图。
