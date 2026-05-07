# 3D 相机节点

`camera_3d_driver` 负责固定扫描点云采集，是 3D 数据保存链路的入口。

## 主要职责

- 读取 3D 相机配置文件。
- 初始化 3D 相机并开始采集。
- 对外发布点云或深度相关数据。
- 将设备参数和运行状态统一纳入 `nodemanage.yaml`。

## 关键文件

- `src/camera_3d_driver/src/camera_3d_driver.cpp`
- `src/camera_3d_driver/config/cameratcp.yaml`

## 运行要点

- `camera_driver_3d.cfg` 指向相机配置文件。
- `publish_tf` 决定是否额外发布坐标变换信息。
- 如果 3D SDK 或设备连接异常，节点会在启动阶段报错。
