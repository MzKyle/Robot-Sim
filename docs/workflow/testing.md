# 测试验收

## 快速自检

```bash
ros2 node list
ros2 service list
ros2 control list_controllers
ros2 topic echo --once /joint_states
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/points
ros2 topic hz /scan
ros2 topic echo --once /diagnostics
```

## 推荐检查项

- `profile_lint --require-receivers` 是否通过。
- `light/full` 模式下 `profile_lint` 是否能确认 `gz_ros2_control::GazeboSimROS2ControlPlugin` 从当前 overlay 的 `libgz_ros2_control-system.so` 导出。
- 仿真节点、controller 和 bridge 是否正常启动。
- RGB、深度、点云、LaserScan 和 IMU 话题是否有数据。
- `robot_sim_sensors` 是否在 `/diagnostics` 输出 receiver 健康状态。
- 旧 `data_collect` 硬件启动链路本轮暂不维护，不作为仿真验收条件。

## Gazebo 插件预检

真实 Gazebo 链路必须使用本工作空间编译的 `gz_ros2_control` overlay。只跑 `mock` 不会加载 Gazebo system plugin，因此不能覆盖这类 ABI 问题。

```bash
ros2 run robot_sim_bringup profile_lint --profile panda --mode light --require-receivers
gz plugin -p "$(ros2 pkg prefix gz_ros2_control)/lib/libgz_ros2_control-system.so" --info
```

输出里应包含 `gz_ros2_control::GazeboSimROS2ControlPlugin`。如果路径落到 `/opt/ros/humble/lib/libgz_ros2_control-system.so` 且提示没有导出插件，重新执行带 `--allow-overriding gz_ros2_control` 的工作空间构建并 `source install/setup.bash`。

## 结束条件

满足以下条件时可以认为主流程通过：

1. `sim.launch.py` 可正常启动目标 profile。
2. controller 均处于预期状态。
3. 启用的传感器 bridge topic 有稳定数据。
4. `sensor_receivers.launch.py` 能根据 profile 自动启动 receiver。
5. `/diagnostics` 可看到对应 receiver 的消息计数和 Hz。
