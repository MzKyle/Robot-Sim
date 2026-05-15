# 2D 相机节点

`camera_pool_driver` 负责采集 2D 相机图像并发布到 ROS 主题，供 `data_collect` 保存、`data_collect_quality` 评估和 `data_collect_ui` 预览。

## 主要职责

- 初始化 2D 相机 SDK。
- 加载相机参数，如触发模式、饱和度、Gamma、曝光时间和增益。
- 将图像转换为 ROS 可用的消息格式并发布。
- 支持参数在运行时更新。
- 发布话题为 `/image_topic`，`frame_id` 固定为 `camera_frame`。

## 关键文件

- `src/camera_pool_driver/src/camera_pool_driver.cpp`
- `src/camera_pool_driver/package.xml`

## 运行要点

- 若相机是单色传感器，输出编码会切换为 `mono8`。
- 若相机为彩色传感器，输出编码会切换为 `bgr8`。
- 运行前要确保相机 SDK 和驱动已经正确安装。
- 默认参数与 `src/config/nodemanage.yaml` 中的 `camera_node` 段一致。
