# Fanuc 机器人节点

`fanuc_robot` 负责读取 Fanuc 机器人状态、目标寄存器值和相关焊接信息，并向外发布 ROS 数据。

## 主要职责

- 加载 Fanuc 共享库。
- 连接机器人控制器。
- 发布机器人状态、报警和生产相关数据。
- 提供与采集流程配套的服务和状态接口。

## 关键配置

```yaml
robot_driver_fanuc:
  ros__parameters:
    so_file_path: '/home/kyle/sany/weld_data_collect_ws/src/fanuc_robot/lib/libFanucRobot.so'
    robot_ip: '10.16.140.114'
    robot_port: 60008
    target_register_index: 100
```

## 运行要点

- `so_file_path` 必须指向真实存在的 `libFanucRobot.so`。
- `robot_ip` 和 `robot_port` 需要与控制器保持一致。
- `target_register_index` 用于区分采集类别或工件类别。
