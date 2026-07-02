# 产品路线图

`robot_sim` 的产品定位是工业机器人仿真验收与回归测试平台，而不是单纯的仿真 demo。

## 第一阶段：可验收

目标是让一个机器人应用能够被配置化启动、执行、度量和归档。

- 以 `sim_profile`、`scene`、`world_preset`、`validation_case` 四类 `schema: 2` 配置作为公开契约。
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

- 扩展标准任务族：空场运动、障碍避让、fixture-to-pallet、pick-place、sensor calibration、conveyor sorting。
- 支持外部 profile package 和外部 validation case package。
- 增加批量矩阵执行：profile × scene × validation case。
- 输出 JUnit 或等价 CI 报告，方便接入交付流水线。

## 第三阶段：工程化

- 增加 Humble/Jazzy 兼容矩阵。
- 增加 docker/devcontainer，降低环境搭建成本。
- 增强 rosbag、日志和指标的长期归档能力。
- 在不改变 CLI 契约的前提下补充 Web/HTML 汇总视图。
