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
