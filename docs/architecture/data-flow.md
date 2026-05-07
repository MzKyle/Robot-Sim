# 数据流

## 主数据流

```mermaid
flowchart LR
    Camera2D[2D Camera] --> Node2D[camera_pool_driver]
    Camera3D[3D Camera] --> Node3D[camera_3d_driver]
    Robot[Fanuc Robot] --> RobotNode[fanuc_robot]

    Node2D --> Collect[data_collect]
    Node3D --> Collect
    RobotNode --> Collect

    Collect --> Status[/data_collect_status/]
    Collect --> Manifest[manifest.json]
    Collect --> Media[Camera / PointCloud / CSV]

    Status --> UI[data_collect_ui]
    UI -->|开始/停止/参数修改| Collect
```

## 关键流程

1. `data_collect_bringup` 读取 `nodemanage.yaml` 并启动各个节点。
2. 相机和机器人节点先完成硬件初始化，再开始对外发布数据。
3. `data_collect` 根据任务状态和采样间隔决定是否保存数据。
4. `data_collect_ui` 订阅状态话题，并通过服务完成采集控制和任务录入。
5. 每次采集结束后会生成标准元数据，供历史检索使用。
