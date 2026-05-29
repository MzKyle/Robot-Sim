# 生产部署建议

## 安装后的默认路径

- 配置文件：`/etc/weld_data_collect/nodemanage.yaml`
- 数据目录：`/var/lib/weld_data_collect/data`

## 推荐做法

- 先使用 `ros2 launch robot_sim_bringup sim.launch.py` 验证仿真控制链；需要完整感知数据时追加 `sim_mode:=full`。
- 使用 `ros2 launch robot_sim_bringup sensor_receivers.launch.py sim_profile:=panda` 验证仿真传感器接收和 `/diagnostics`。
- 旧真实相机/Fanuc 硬件驱动包已移除，`data_collect_bringup` 的旧硬件启动入口本轮暂不维护。
- 仅调试部分功能时，可按需选择核心包重新编译。
- 如果主机使用自定义 OpenCV，建议在干净系统中重新构建打包产物。

## 部署检查

- profile 中 robot、controller、bridge、receiver 和 scenario 路径是否通过 lint。
- 数据保存目录是否有写权限。
- UI 所需的 Qt Python 绑定是否已安装。
