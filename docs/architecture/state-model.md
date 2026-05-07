# 状态模型

## 采集状态

采集状态以 `/data_collect_status` 话题发布，常见字段包括：

| 字段 | 说明 |
| --- | --- |
| `running` | 当前是否正在采集 |
| `auto_save` | 是否启用自动采集模式 |
| `current_save_dir` | 当前保存目录 |
| `target_register_index` | 目标寄存器编号 |
| `target_register_value` | 目标寄存器当前值 |
| `task_id` | 任务号 |
| `workpiece_id` | 工件号 |
| `weld_seam_id` | 焊道号 |
| `operator_name` | 操作员 |
| `shift` | 班次 |
| `notes` | 备注 |
| `image_count` | 已保存图像数量 |
| `point_cloud_count` | 已保存点云数量 |
| `tool_pose_count` | 已保存工具位姿数量 |
| `fanuc_info_count` | 已保存 Fanuc 状态数量 |
| `last_error` | 最近一次写入错误 |

## 数据目录

每次采集会生成一个独立目录：

```text
data/<target_register_value>/<YYYY-MM-DD>/<HH-MM-SS>/
```

如果启动采集时还没有收到目标寄存器值，会使用：

```text
data/unknown/<YYYY-MM-DD>/<HH-MM-SS>/
```

## 典型输出

- `camera/`：2D 图像。
- `camera_log/`：图像日志。
- `height_log/`：高度日志图像。
- `scan_point_cloud/`：3D 点云 PLY 文件。
- `robot_state/`：TCP 位姿 CSV。
- `fanuc_robot_info/`：Fanuc 状态 CSV。
- `manifest.json`：标准采集元数据。
