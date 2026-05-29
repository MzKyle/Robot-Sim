# 模块全景

| 包名 | 职责 |
| --- | --- |
| `data_collect_quality` | 采集质量评估与状态发布 |
| `data_collect` | 采集保存、状态发布和数据目录组织 |
| `robot_sim_bringup` | gz sim 8 仿真入口、三档仿真模式和传感器桥接 |
| `robot_sim_sensor_camera` | 仿真 RGB 相机话题接收与 diagnostics |
| `robot_sim_sensor_depth` | 仿真深度图、相机参数和点云接收与 diagnostics |
| `robot_sim_sensor_lidar` | 仿真 LaserScan 和 lidar 点云接收与 diagnostics |
| `robot_sim_sensor_imu` | 仿真 IMU 话题接收与 diagnostics |
| `robot_sim_scenarios` | base world、assets 和 scenario 组合 |
| `simulation_interfaces` | 通用仿真 scenario 接口 |
| `robot_task_interfaces` | 通用任务上下文接口 |
| `acquisition_interfaces` | 通用采集状态、质量和任务接口 |
| `data_collect_ui` | 桌面操作台、任务录入和历史检索 |
| `data_collect_bringup` | launch 入口和默认配置注入 |
| `weld_interface` | 焊接业务 adapter 和旧接口兼容层 |
| `file_reader` | 配置读取和辅助工具 |

## 代码边界

- 设备相关代码集中在相机和机器人包中。
- 采集逻辑集中在 `data_collect`。
- UI 只负责展示和交互，不直接操作硬件。
- 配置文件和协议定义尽量放在共享层，减少重复实现。
