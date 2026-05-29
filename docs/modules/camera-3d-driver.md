# 3D 相机节点

> 已废弃：`camera_3d_driver` 真实硬件驱动包已从工作空间移除。

当前仿真深度图和点云接收由 `robot_sim_sensor_depth` 完成，入口由 `robot_sim_bringup/launch/sensor_receivers.launch.py` 根据 `sim_profile` 自动生成。

```bash
ros2 launch robot_sim_bringup sensor_receivers.launch.py sim_profile:=panda sensor_overrides:=depth=true
```

新项目不再接入旧 `/tcp_cloud_raw`、`/fixed_scan` 或 `/scan_3d`，请直接使用 profile 中的深度 bridge topic，例如 `/camera/depth/image_raw`、`/camera/depth/camera_info` 和 `/camera/points`。
