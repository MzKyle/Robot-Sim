# 模块全景

| 包名 | 职责 |
| --- | --- |
| `robot_sim_bringup` | gz sim 8 仿真入口、三档仿真模式和传感器桥接 |
| `robot_sim_description` | Panda/Fanuc M20iD 描述、模型资源、传感器挂载和 Gazebo 插件 |
| `robot_sim_control` | Panda/Fanuc M20iD 的 ros2_control 配置 |
| `robot_sim_moveit_config` | Panda/Fanuc M20iD 的 MoveIt2 配置 |
| `robot_sim_sensor_camera` | 仿真 RGB 相机话题接收与 diagnostics |
| `robot_sim_sensor_depth` | 仿真深度图、相机参数和点云接收与 diagnostics |
| `robot_sim_sensor_lidar` | 仿真 LaserScan 和 lidar 点云接收与 diagnostics |
| `robot_sim_sensor_imu` | 仿真 IMU 话题接收与 diagnostics |
| `robot_sim_scenarios` | base world、assets 和 scenario 组合 |
| `simulation_interfaces` | 通用仿真 scenario 接口 |
| `robot_task_interfaces` | 通用任务上下文接口 |

## 代码边界

- 机器人型号资源集中在 `robot_sim_description/models/robots/<model>/`。
- 控制器和 MoveIt 配置集中在通用 `robot_sim_control`、`robot_sim_moveit_config` 包中。
- `sim_profile` 是新增或切换机器人型号的入口。
- 通用接口保留在 `robot_task_interfaces` 和 `simulation_interfaces`。
