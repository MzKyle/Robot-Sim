# 生产部署建议

## 安装后的默认路径

- 配置文件：`/etc/weld_data_collect/nodemanage.yaml`
- 数据目录：`/var/lib/weld_data_collect/data`

## 推荐做法

- 在目标机上先确认 RVC SDK、MVSDK、Fanuc 共享库和数据目录权限都可用。
- 通过 `ros2 launch data_collect_bringup data_collect.launch.py` 启动真实采集栈。
- 没有真实设备时，使用 `ros2 launch data_collect_sim data_collect_sim.launch.py` 验证前后端联调。
- 仅调试部分功能时，可按需选择核心包重新编译。
- 如果主机使用自定义 OpenCV，建议在干净系统中重新构建打包产物。

## 部署检查

- 机器人 IP、端口和共享库路径是否正确。
- 配置文件内容是否与现场设备一致。
- 数据保存目录是否有写权限。
- UI 所需的 Qt Python 绑定是否已安装。
