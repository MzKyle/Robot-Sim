# 日志与产物

## 本地目录

| 目录 | 说明 |
| --- | --- |
| `build/` | colcon build 产物 |
| `install/` | colcon install overlay |
| `log/` | colcon 日志 |
| `robot_sim_bags/` | 推荐 rosbag 输出目录 |
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

## CI 产物

GitHub Actions 会在失败或完成后上传：

- colcon logs。
- package test results。
- full smoke logs。
- release deb。
