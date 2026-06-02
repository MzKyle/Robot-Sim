# 传感器 Receiver

`src/sensors/` 中包含四个仿真 receiver 包：

| 包 | 订阅 |
| --- | --- |
| `robot_sim_sensor_camera` | RGB `Image` 和 `CameraInfo` |
| `robot_sim_sensor_depth` | 深度 `Image`、`CameraInfo` 和 `PointCloud2` |
| `robot_sim_sensor_lidar` | `LaserScan` 和 lidar `PointCloud2` |
| `robot_sim_sensor_imu` | `Imu` |

receiver 会统计消息数、Hz、最后时间戳和 frame，并发布 `/diagnostics`。

启动：

```bash
ros2 launch robot_sim_bringup sensor_receivers.launch.py sim_profile:=panda
```
