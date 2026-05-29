# 2D 相机节点

> 已废弃：`camera_pool_driver` 真实硬件驱动包已从工作空间移除。

当前仿真 RGB 图像接收由 `robot_sim_sensor_camera` 完成，入口由 `robot_sim_bringup/launch/sensor_receivers.launch.py` 根据 `sim_profile` 自动生成。

```bash
ros2 launch robot_sim_bringup sensor_receivers.launch.py sim_profile:=panda sensor_overrides:=camera=true
```

新项目不再接入旧 `/image_topic`，请直接使用 profile 中的 RGB bridge topic，例如 `/camera/color/image_raw` 和 `/camera/color/camera_info`。
