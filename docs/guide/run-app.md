# 开发运行

## 编译工作空间

```bash
cd /home/kyle/sany/weld_data_collect_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

如果只想编译核心采集相关包，可以使用：

```bash
colcon build --symlink-install --packages-select weld_interface data_collect data_collect_quality data_collect_ui
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

如果要跑仿真联调：

```bash
ros2 launch robot_sim_bringup sim.launch.py
```

## 启动桌面操作界面

```bash
ros2 run data_collect_ui data_collect_ui
```

## 常用说明

- 真实后端启动后，UI 可以查看采集状态、Fanuc 状态、质量评估和历史数据。
- 仿真平台启动后，控制链来自 `robot_sim_bringup`；需要图像、点云、激光或 IMU 时使用 `sim_mode:=full` 或按组打开传感器。
- 如果主机未安装 RVC SDK、MVSDK 或 Fanuc 共享库，真实驱动包可能在编译或运行时失败。
- 真实链路启动时会读取 `nodemanage.yaml`；仿真链路使用 `robot_sim_bringup` 的 launch 参数和 `robot_sim_control/config/panda_controllers.yaml`。
