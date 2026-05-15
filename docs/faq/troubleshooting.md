# 常见问题

## 找不到 ROS 包

确认已经加载环境：

```bash
source /opt/ros/humble/setup.bash
source /home/kyle/sany/weld_data_collect_ws/install/setup.bash
```

## UI 无法启动

如果提示缺少 PySide6：

```bash
python3 -m pip install --user PySide6
```

如果按钮是灰色，通常表示对应 ROS 服务还没有启动。请先确认后端节点是否在线。

## 采集目录进入 unknown

说明启动采集时还没有收到焊接寄存器信息。请检查 Fanuc 节点或仿真节点是否启动、机器人连接是否正常、`target_register_index` 是否正确。

## 没有点云或图像保存

请先确认采集状态为 `running`，再检查对应 topic 是否有数据。

如果你跑的是仿真链路，还要确认 `data_collect_sim.launch.py` 的 `use_gz_sensors` 和 `use_gazebo` 是否开启。

## Fanuc 节点启动失败

优先检查：

- `so_file_path` 是否指向真实存在的 `libFanucRobot.so`。
- `robot_ip` 和 `robot_port` 是否正确。
- 当前主机是否能访问机器人控制器。
- Fanuc 共享库依赖是否完整。
