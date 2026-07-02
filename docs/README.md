# robot_sim 文档

> ROS 2 Humble + Gazebo Harmonic 的工业机器人仿真验收与回归测试平台。

`robot_sim` 将机器人描述、Gazebo 场景、ros2_control、MoveIt2、RViz2、ros_gz bridge、仿真传感器 receiver 和 validation case 组织成一条可复用的验收链路。当前内置 Panda 与 Fanuc M-20iD/12L profile。

## 快速入口

- [环境依赖](guide/prerequisites.md)
- [产品路线图](roadmap.md)
- [开发运行](guide/run-app.md)
- [仿真方案](guide/simulation.md)
- [测试验收](workflow/testing.md)
- [ROS 2 录包](guide/rosbag-recording.md)
- [Deb 打包与 Release](guide/package-install.md)
- [常见问题](faq/troubleshooting.md)

## 目录一览

| 路径 | 说明 |
| --- | --- |
| `src/core/robot_sim_bringup/` | 仿真总入口、profile、lint、smoke test 和 launch |
| `src/core/robot_sim_description/` | Panda/Fanuc 模型、xacro、mesh 和传感器挂载 |
| `src/core/robot_sim_control/` | ros2_control controller 配置 |
| `src/core/robot_sim_moveit_config/` | MoveIt2 和 RViz2 配置 |
| `src/core/robot_sim_scenarios/` | scene library、base world、assets 和 world presets |
| `src/sensors/` | camera、depth、lidar、imu receiver |
| `src/interfaces/` | 通用仿真和任务接口 |
| `src/vendor/gz_ros2_control/` | Gazebo Harmonic 的 ros2_control overlay |

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
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l sim_mode:=full
ros2 run robot_sim_bringup run_case --case industrial_fixture_to_pallet --output-dir robot_sim_runs
```

## CI/CD

GitHub Actions 覆盖 PR 构建测试、定时 full smoke、GitHub Pages 文档部署和 tag deb release。详见 [CI/CD](workflow/ci-cd.md)。
