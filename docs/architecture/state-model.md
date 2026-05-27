# 状态模型

## 采集状态

通用采集状态以 `/acquisition/status` 话题发布，旧焊接链路仍同步发布 `/data_collect_status` 作为兼容接口。

`/acquisition/status` 常见字段包括：

| 字段 | 说明 |
| --- | --- |
| `running` | 当前是否正在采集 |
| `auto_save` | 是否启用自动采集模式 |
| `current_save_dir` | 当前保存目录 |
| `target_source` | 目标来源，例如 `fanuc_register` |
| `target_index` | 目标索引 |
| `target_value` | 目标值 |
| `task.task_id` | 任务号 |
| `task.workpiece_id` | 工件号 |
| `task.operator_name` | 操作员 |
| `task.shift` | 班次 |
| `task.notes` | 备注 |
| `task.extension_keys/values` | 业务扩展字段，焊接场景可放 `weld_seam_id` |
| `image_count` | 已保存图像数量 |
| `image_log_count` | 已保存图像日志数量 |
| `height_log_count` | 已保存高度日志数量 |
| `point_cloud_count` | 已保存点云数量 |
| `tool_pose_count` | 已保存工具位姿数量 |
| `estimated_line_count` | 已保存估计线数量 |
| `device_state_count` | 已保存设备状态数量 |
| `last_error` | 最近一次写入错误 |
| `quality_available` | 当前质量评估是否可用 |
| `quality_sync_error_ms` | 图像与点云同步误差 |
| `quality_frame_loss_rate` | 图像帧丢失率 |
| `quality_blur_score` | 图像清晰度评分 |
| `quality_point_cloud_completeness` | 点云完整度评分 |
| `quality_reason` | 质量评估原因或状态说明 |

## 数据目录

每次采集会生成一个独立目录：

```text
data/<YYYY-MM-DD>/<weld_id>/<weld_layer>/<HH-MM-SS>/
```

这是当前焊接 adapter 的默认目录约定。如果启动采集时还没有收到焊接寄存器信息，会使用：

```text
data/<YYYY-MM-DD>/unknown/unknown/<HH-MM-SS>/
```

## 典型输出

- `camera/`：2D 图像。
- `camera_log/`：图像日志。
- `height_log/`：高度日志图像。
- `camera_depth/`：深度图或深度相关保存目录。
- `camera_depth_log/`：深度图日志目录。
- `scan_point_cloud/`：3D 点云 PLY 文件。
- `robot_state/`：TCP 位姿 CSV。
- `welding_state/`：焊接 adapter 状态记录目录。
- `control_cmd/`：控制命令记录目录。
- `state_type/`：状态类型记录目录。
- `fanuc_robot_info/`：Fanuc adapter 状态 CSV。
- `manifest.json`：标准采集元数据。
- `meta.json`：兼容旧流程的元数据文件。
