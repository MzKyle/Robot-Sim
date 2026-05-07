# 测试验收

## 快速自检

```bash
ros2 node list
ros2 service list
ros2 topic echo --once /data_collect_status
ros2 topic echo --once /fanuc_robot_info
ros2 topic hz /image_topic
ros2 topic hz /tcp_cloud_raw
```

## 推荐检查项

- 节点是否正常启动。
- `/data_collect_status` 是否能持续更新。
- 2D 图像和 3D 点云是否有数据。
- Fanuc 节点是否能正确连接控制器。
- 采集目录是否按目标寄存器值组织。

## 结束条件

满足以下条件时可以认为主流程通过：

1. 后端可正常启动。
2. UI 可以打开并显示状态。
3. 图像、点云和机器人状态可保存。
4. 服务调用可以正常启停采集。
5. 历史目录和元数据可以正常生成。
