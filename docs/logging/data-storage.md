# 数据存储结构

## 采集目录

每次采集会生成一个独立目录：

```text
data/<YYYY-MM-DD>/<weld_id>/<weld_layer>/<HH-MM-SS>/
```

这是当前焊接 adapter 的默认目录模板。如果启动采集时还没有收到焊接寄存器信息，会使用：

```text
data/<YYYY-MM-DD>/unknown/unknown/<HH-MM-SS>/
```

## 典型内容

```text
camera/                 2D 图像
camera_log/             图像日志
height_log/             高度日志图像
camera_depth/           深度图或深度相关保存目录
camera_depth_log/       深度图日志目录
scan_point_cloud/       3D 点云 PLY 文件
robot_state/            TCP 位姿 CSV
welding_state/          焊接 adapter 状态记录目录
control_cmd/            控制命令记录目录
state_type/             状态类型记录目录
fanuc_robot_info/       Fanuc adapter 状态 CSV
manifest.json           标准采集元数据
meta.json               兼容旧流程的元数据文件
```

## 其他日志

- 终端输出是排查问题的第一手信息。
- 如果使用 colcon 构建，构建日志会在工作空间的 `log/` 目录中保留。
