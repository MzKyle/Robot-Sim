# 测试验收

## 快速自检

```bash
ros2 node list
ros2 service list
ros2 topic echo --once /acquisition/status
ros2 topic echo --once /acquisition/quality
ros2 topic echo --once /data_collect_status
ros2 topic echo --once /fanuc_robot_info
ros2 topic hz /image_topic
ros2 topic hz /tcp_cloud_raw
ros2 topic echo --once /tool_pos
ros2 topic echo --once /data_collect_quality
```

## 推荐检查项

- 节点是否正常启动。
- `/acquisition/status` 是否能持续更新；旧链路可同时检查 `/data_collect_status`。
- 2D 图像、3D 点云和 TCP 位姿是否有数据。
- Fanuc 节点或仿真节点是否正常发布状态。
- 采集目录是否按当前 adapter 的目录模板组织。
- `/acquisition/quality` 是否持续输出质量等级；旧链路可同时检查 `/data_collect_quality`。

## 结束条件

满足以下条件时可以认为主流程通过：

1. 后端可正常启动。
2. UI 可以打开并显示状态。
3. 图像、点云、TCP 位姿和机器人状态可保存。
4. 服务调用可以正常启停采集。
5. 历史目录和元数据可以正常生成。
6. 质量评估能够给出 PASS、WARN 或 FAIL 结果。
