# 常见问题

## 找不到 robot_sim_bringup

确认已经构建并 source overlay：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix robot_sim_bringup
```

## Gazebo 插件 ABI 不匹配

确认构建前设置了 `GZ_VERSION=harmonic`，并重新构建 `gz_ros2_control`：

```bash
export GZ_VERSION=harmonic
rm -rf build/gz_ros2_control install/gz_ros2_control
colcon build --symlink-install --allow-overriding gz_ros2_control --packages-select gz_ros2_control
source install/setup.bash
```

检查插件：

```bash
gz plugin -p "$(ros2 pkg prefix gz_ros2_control)/lib/libgz_ros2_control-system.so" --info
```

## MoveIt 规划后有半透明轨迹

RViz 的 `Planned Path` 会显示规划预览。默认配置关闭循环播放；如果仍持续播放，检查 RViz 中 MotionPlanning 面板的 `Loop Animation`。

## Octomap 3D sensor 提示

如果日志出现 `No 3D sensor plugin(s) defined for octomap updates`，表示当前没有启用 MoveIt octomap updater。它不影响本项目默认的轨迹规划和执行。

## Full smoke 在 CI 上失败

先查看上传的 smoke log artifact，重点检查：

- `sim.launch.log`
- Gazebo spawn 是否成功。
- controller 是否 active。
- `gz_ros2_control` 插件是否来自当前 workspace overlay。

## `arm_controller` 一直是 inactive

当前实现会用单个 spawner 成组激活同一 manager 下的 controller，并把 profile timeout
显式用于 switch。如果仍出现 `Switch controller timed out`：

```bash
ros2 control list_controllers -c /controller_manager
ros2 control list_hardware_interfaces -c /controller_manager
```

确认使用的是最新 workspace overlay，并检查 controller YAML 中每个 controller 的
`type`、joint 和 command/state interface。不要同时为同一 manager 启动另一组 spawner。

## MoveIt 返回 `INVALID_MOTION_PLAN (-2)`

先在 `logs/sim.launch.log` 搜索 `Computed path is not valid` 和具体碰撞对。如果规划器
找到路径、最终校验却报告窄障碍物碰撞，检查机器人 OMPL 配置中的
`longest_valid_segment_fraction`。Fanuc 当前使用 `0.01`；调大可能让插值跨过夹具或
立柱。目标区域本身碰撞时，应修正 scene/region，而不是放宽最终校验。

## CI 中点云 Hz 偶发低于阈值

smoke helper 对带 header 的传感器消息按仿真时间计算 Hz，只有无合法时间戳时才使用
墙钟。先确认 CI 已构建并 source 最新 `robot_sim_bringup`；再检查实际样本数、消息
header 是否递增以及 Gazebo bridge topic 是否正确。不要仅因 runner real-time factor
低于 1 就降低仿真频率阈值。
