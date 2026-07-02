# 日志与产物

## 本地目录

| 目录 | 说明 |
| --- | --- |
| `build/` | colcon build 产物 |
| `install/` | colcon install overlay |
| `log/` | colcon 日志 |
| `robot_sim_bags/` | 推荐 rosbag 输出目录 |
| `robot_sim_runs/` | `run_case` 验收运行产物 |
| `dist/` | deb 打包输出 |

这些目录默认不提交到 Git。

## Smoke 日志

`scripts/sim_smoke_test.sh --keep-logs` 会保留临时日志目录：

```text
/tmp/robot_sim_smoke.*
```

目录内通常包含：

- `sim.launch.log`
- `robot.urdf`
- rosbag 检查日志
- validation metrics

## Validation Case 产物

`ros2 run robot_sim_bringup run_case --case <name>` 会为每次运行创建独立目录：

- `manifest.json`：case、profile、scene、命令、时间和产物路径。
- `metrics.json`：步骤状态、controller、TF、sensor Hz、MoveIt、误差和 clearance 指标。
- `report.md` / `report.html`：同一报告模型生成的人工可读报告。
- `logs/`：各步骤日志，包含 `sim.launch.log`。
- `rosbag/`：按 case 配置录制的 rosbag。

## CI 产物

GitHub Actions 会在失败或完成后上传：

- colcon logs。
- package test results。
- full smoke logs。
- release deb。
