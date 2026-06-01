# 采集质量节点

旧 `data_collect_quality` 采集质量节点已从当前仿真主线移除。当前项目只保留仿真传感器 receiver 的 diagnostics 输出。

## 主要职责

- 订阅 `/data_collect_status`、`/image_topic` 和 `/tcp_cloud_raw`。
- 计算图像同步误差、帧丢失率、清晰度和点云完整度。
- 以 PASS / WARN / FAIL 的形式给出质量等级。
- 旧质量消息接口已移除；新的质量评估应在独立数据检验项目中实现。

## 默认阈值

- `expected_image_fps`：默认 15.0。
- `min_blur_variance`：默认 120.0。
- `expected_points_per_cloud`：默认 40000.0。
- `warn_sync_ms` / `fail_sync_ms`：默认 50 / 100 毫秒。
- `warn_frame_loss_rate` / `fail_frame_loss_rate`：默认 0.05 / 0.10。
- `warn_blur_score` / `fail_blur_score`：默认 70 / 40。
- `warn_cloud_completeness` / `fail_cloud_completeness`：默认 70 / 40。

## 运行要点

- 只有在采集运行中且图像、点云都到位时，`available` 才会变为 `true`。
- UI 和 `data_collect` 可以直接订阅质量结果做状态展示或数据归档。
