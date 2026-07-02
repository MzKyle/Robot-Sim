# robot_sim

<p align="center">

  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-00A6A6" />
  <img alt="Gazebo Harmonic" src="https://img.shields.io/badge/Gazebo-Harmonic%20%7C%20gz%20sim%208-F57C00" />
  <img alt="MoveIt2" src="https://img.shields.io/badge/Planning-MoveIt2-2D6CDF" />
  <a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache--2.0-blue" /></a>
</p>

`robot_sim` 是一个面向 ROS 2 Humble、Gazebo Harmonic、ros2_control 和 MoveIt2 的工业机器人仿真验收与回归测试平台。它提供 Panda 与 Fanuc M-20iD/12L profile，可用于验证模型、场景、控制器、传感器桥接、MoveIt 规划执行、RViz 可视化、rosbag 录制和 validation case 验收链路。

## Features

- `mock`、`light`、`full` 三档仿真模式。
- Gazebo Harmonic (`gz sim 8`) + `gz_ros2_control` 源码 overlay。
- Panda 与 Fanuc M-20iD/12L 的 URDF/xacro、controller 和 MoveIt 配置。
- RGB、深度、点云、2D LaserScan、3D lidar 点云和 IMU bridge。
- 仿真传感器 receiver 包，发布 `/diagnostics` 健康信息。
- Profile lint、smoke test、GitHub Actions CI、GitHub Pages 文档和 tag deb release。
- `run_case` 验收入口，生成 `manifest.json`、`metrics.json`、`report.md/html`、日志和 rosbag。

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic / `gz sim 8`
- MoveIt2
- `colcon`、`python3-colcon-override-check`、`rosdep`、`git`

安装 Gazebo Harmonic 时需要 OSRF apt 源。完整依赖说明见 [docs/guide/prerequisites.md](docs/guide/prerequisites.md)。

## Quick Start

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
ros2 launch robot_sim_bringup sim.launch.py
```

默认 `sim_mode:=light` 会启动 Gazebo 和控制链，关闭传感器、MoveIt 和 RViz，适合日常控制链调试。

完整仿真：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=full
```

Fanuc + MoveIt + RViz：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l sim_mode:=full
```

纯 ROS mock 控制链：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=mock
```

## Simulation Profiles

内置 profile：

```text
src/core/robot_sim_bringup/config/sim_profiles/panda.yaml
src/core/robot_sim_bringup/config/sim_profiles/fanuc_m20id12l.yaml
src/core/robot_sim_bringup/config/sim_profiles/fanuc_m20id12l_industrial_cell.yaml
```

新机器人建议从模板开始：

```bash
cp src/core/robot_sim_bringup/config/templates/template_robot.yaml custom_robot.yaml
ros2 run robot_sim_bringup profile_lint --profile-file custom_robot.yaml --mode full --require-moveit
```

## Test

```bash
colcon test --packages-select robot_sim_bringup robot_sim_scenarios
colcon test-result --verbose

ros2 run robot_sim_bringup profile_lint --profile panda --mode full --require-moveit --require-receivers
ros2 run robot_sim_bringup profile_lint --profile fanuc_m20id12l --mode full --require-moveit --require-receivers

scripts/sim_smoke_test.sh --profile panda --mode mock --timeout 60
scripts/sim_smoke_test.sh --profile fanuc_m20id12l --mode full --with-moveit --timeout 120
ros2 run robot_sim_bringup run_case --case industrial_fixture_to_pallet --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case industrial_obstacle_clearance --output-dir robot_sim_runs --timeout 120
```

`mock` smoke 适合快速 CI；`full` smoke 会实际启动 Gazebo，耗时更长，适合手动或定时验证。

## CI/CD

- `ci.yml`：PR 和 `main` push 执行构建、测试、profile lint 和 mock smoke。
- `simulation-smoke.yml`：手动或每周定时执行 Gazebo full smoke 和工业 validation case。
- `docs.yml`：`main` 更新后部署 `docs/` 到 GitHub Pages。
- `release.yml`：推送 `vMAJOR.MINOR.PATCH` tag 后构建 deb 并上传 GitHub Release。

本地 deb 构建：

```bash
bash packaging/build_deb.sh
sudo apt install ./dist/robot-sim_0.1.0-1_amd64.deb
robot-sim-check
robot-sim run-case --case industrial_fixture_to_pallet
```

## Repository Layout

| 路径 | 说明 |
| --- | --- |
| `src/core/robot_sim_bringup/` | 仿真入口、profile loader、lint、smoke helper 和 launch |
| `src/core/robot_sim_description/` | Panda/Fanuc 模型、xacro、mesh、传感器挂载和 ros2_control 标签 |
| `src/core/robot_sim_control/` | 各机器人 controller 配置 |
| `src/core/robot_sim_moveit_config/` | 各机器人 MoveIt 和 RViz 配置 |
| `src/core/robot_sim_scenarios/` | scene library、base world、assets 和 world presets |
| `src/sensors/` | camera、depth、lidar、imu 仿真 receiver |
| `src/interfaces/` | 仿真 scenario 和任务上下文接口 |
| `src/vendor/gz_ros2_control/` | vendored `gz_ros2_control` source overlay |
| `docs/` | docsify 文档站点 |
| `packaging/` | deb 打包脚本 |

## Documentation

完整文档从 [docs/README.md](docs/README.md) 开始。常用入口：

- [环境依赖](docs/guide/prerequisites.md)
- [仿真运行](docs/guide/simulation.md)
- [测试验收](docs/workflow/testing.md)
- [产品路线图](docs/roadmap.md)
- [ROS API](docs/interfaces/ros-api.md)
- [故障排查](docs/faq/troubleshooting.md)

## License

本项目使用 [Apache License 2.0](LICENSE)。第三方资源说明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
