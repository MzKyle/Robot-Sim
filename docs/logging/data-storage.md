# 数据存储结构

## 采集目录

每次采集会生成一个独立目录：

```text
data/<target_register_value>/<YYYY-MM-DD>/<HH-MM-SS>/
```

如果启动采集时还没有收到目标寄存器值，会使用：

```text
data/unknown/<YYYY-MM-DD>/<HH-MM-SS>/
```

## 典型内容

```text
camera/                 2D 图像
camera_log/             图像日志
height_log/             高度日志图像
scan_point_cloud/       3D 点云 PLY 文件
robot_state/            TCP 位姿 CSV
fanuc_robot_info/       Fanuc 状态 CSV
manifest.json           标准采集元数据
```

## 其他日志

- 终端输出是排查问题的第一手信息。
- 如果使用 colcon 构建，构建日志会在工作空间的 `log/` 目录中保留。
