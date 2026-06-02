# Contributing

感谢你愿意改进 `robot_sim`。这个仓库以 ROS 2 Humble、Gazebo Harmonic、MoveIt2 和 ros2_control 仿真链路为核心。

## 开发环境

```bash
git clone --recursive https://github.com/MzKyle/robot_sim.git robot_sim
cd robot_sim
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
```

安装依赖请参考 [环境依赖](docs/guide/prerequisites.md)。

## 提交前检查

```bash
colcon list --names-only
colcon build --symlink-install --allow-overriding gz_ros2_control --packages-select \
  gz_ros2_control \
  robot_sim_description robot_sim_control robot_sim_scenarios robot_sim_moveit_config \
  robot_sim_sensor_camera robot_sim_sensor_depth robot_sim_sensor_lidar robot_sim_sensor_imu \
  robot_sim_bringup robot_task_interfaces simulation_interfaces
colcon test --packages-select robot_sim_bringup robot_sim_scenarios
colcon test-result --verbose
ros2 run robot_sim_bringup profile_lint --profile panda --mode full --require-moveit --require-receivers
scripts/sim_smoke_test.sh --profile panda --mode mock --timeout 60
```

完整 Gazebo smoke test 运行时间更长，适合在发版前或手动 CI 中执行：

```bash
scripts/sim_smoke_test.sh --profile fanuc_m20id12l --mode full --with-moveit --timeout 120
```

## Pull Request 约定

- 保持 ROS 包名、launch 参数、profile 名称和消息/服务接口稳定，除非 PR 明确说明破坏性变更。
- 新增机器人优先通过 `sim_profile`、URDF/xacro、controller yaml 和 MoveIt 配置接入。
- 不提交 `build/`、`install/`、`log/`、rosbag 或本地 IDE 文件。
- `src/vendor/gz_ros2_control` 是 submodule；除非 PR 专门处理 vendor patch，不要混入 submodule 内部改动。
