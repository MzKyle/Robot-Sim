# 开发运行

## 编译工作空间

```bash
cd /home/kyle/sany/weld_data_collect_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

如果只想编译核心采集相关包，可以使用：

```bash
colcon build --symlink-install --packages-select weld_interface data_collect data_collect_ui
```

## 加载环境

```bash
cd /home/kyle/sany/weld_data_collect_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 启动采集后端

```bash
ros2 launch data_collect_bringup data_collect.launch.py
```

## 启动桌面操作界面

```bash
ros2 run data_collect_ui data_collect_ui
```

## 常用说明

- 后端启动后，UI 可以查看采集状态、Fanuc 状态和历史数据。
- 如果主机未安装 RVC SDK、MVSDK 或 Fanuc 共享库，相关驱动包可能在编译或运行时失败。
- launch 启动时会读取 `src/config/nodemanage.yaml`，确保路径和内容与实际硬件一致。
