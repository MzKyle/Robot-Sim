# 日志与产物

`robot_sim` 的验收产物按“每次运行一个目录”保存，目标是让失败可定位、结果可归档、报告可交付。

## 本地目录

| 目录 | 说明 |
| --- | --- |
| `build/` | colcon build 产物 |
| `install/` | colcon install overlay |
| `log/` | colcon 日志 |
| `robot_sim_bags/` | 手动 rosbag 输出目录 |
| `robot_sim_runs/` | `run_case` 验收运行产物 |
| `dist/` | deb 打包输出 |

这些目录默认不提交到 Git。

## Run Case 产物

`ros2 run robot_sim_bringup run_case --case <name>` 会为每次运行创建独立目录：

```text
robot_sim_runs/<UTC timestamp>_<case>_<profile>/
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
    profile.json
    render_urdf.log
    check_urdf.log
    gazebo_spawn.log
    joint_states.log
    controllers_active.log
    trajectory_action.log
    sensor_hz.log
    tf_tree.log
    moveit.log
    validation_case.log
  rosbag/
    metadata.yaml
```

| 文件 | 说明 |
| --- | --- |
| `manifest.json` | case、profile、scene、命令参数、git commit、开始/结束时间、退出码和产物路径 |
| `effective_case.yaml` | 应用 CLI 覆盖、scene variant 和 scene 参数后的最终 case |
| `effective_profile.yaml` | 本次使用的最终 profile |
| `robot.urdf` | 渲染后的机器人模型，便于排查 xacro 和 link/frame |
| `metrics.json` | 步骤状态、controller、TF、sensor Hz、MoveIt、误差、clearance 和最终 pass/fail |
| `validation_metrics.json` | validation helper 输出的任务级指标 |
| `report.md` | Markdown 报告，适合在 CI artifact 或 PR 评论中查看 |
| `report.html` | HTML 报告，适合交付和人工浏览 |
| `logs/sim.launch.log` | 启动仿真的主日志 |
| `rosbag/metadata.yaml` | rosbag 元数据；传入 `--no-rosbag` 时不会生成 |

失败时优先打开 `report.html`，找到 Steps 表中第一个失败项，再打开对应 log。

## Metrics 字段

常用字段：

| 字段 | 含义 |
| --- | --- |
| `passed` | 本次验收是否通过 |
| `steps[]` | 每个阶段的状态、耗时、返回码和日志路径 |
| `business_actions[]` | task runner 生成的业务步骤 |
| `sensor_hz` | 每个期望 topic 的频率和样本数 |
| `tf_ok` | TF 检查结果 |
| `plan_success_rate` | MoveIt 目标规划成功率 |
| `planning_time_sec` / `execution_time_sec` | 规划和执行耗时 |
| `goal_position_error_m` | 目标误差 |
| `max_controller_error_rad` | controller 跟踪误差 |
| `min_tcp_clearance_m` | TCP clearance |
| `moveit_error_code` | MoveIt 返回码 |

CI 中推荐上传整个 `robot_sim_runs/` 子目录，而不是只保存 `metrics.json`。

## Rosbag

`run_case` 默认启用 rosbag，topic group 由 case `artifacts.rosbag.topic_group` 决定。临时关闭：

```bash
ros2 run robot_sim_bringup run_case --case empty_motion --no-rosbag
```

调整录制时长：

```bash
ros2 run robot_sim_bringup run_case --case empty_motion --rosbag-duration 15
```

回放：

```bash
ros2 bag info robot_sim_runs/<run>/rosbag/<bag_name>
ros2 bag play robot_sim_runs/<run>/rosbag/<bag_name>
```

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

## CI 产物

GitHub Actions 会在失败或完成后上传：

- colcon logs。
- package test results。
- full smoke logs。
- `robot_sim_runs/`。
- release deb。
