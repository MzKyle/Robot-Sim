# 模块全景

| 包名 | 职责 |
| --- | --- |
| `camera_pool_driver` | 2D 相机图像采集与发布 |
| `camera_3d_driver` | 3D 相机固定扫描与点云发布 |
| `fanuc_robot` | Fanuc 机器人状态、目标寄存器和服务接口 |
| `data_collect_quality` | 采集质量评估与状态发布 |
| `data_collect` | 采集保存、状态发布和数据目录组织 |
| `data_collect_sim` | gz sim 8 仿真入口、Panda 机械臂和模拟链路 |
| `data_collect_ui` | 桌面操作台、任务录入和历史检索 |
| `data_collect_bringup` | launch 入口和默认配置注入 |
| `weld_interface` | ROS 消息和服务定义 |
| `file_reader` | 配置读取和辅助工具 |

## 代码边界

- 设备相关代码集中在相机和机器人包中。
- 采集逻辑集中在 `data_collect`。
- UI 只负责展示和交互，不直接操作硬件。
- 配置文件和协议定义尽量放在共享层，减少重复实现。
