# 开发运行

## 编译仿真工作空间

```bash
cd /home/kyle/sany/robot_sim
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
colcon build --symlink-install --allow-overriding gz_ros2_control --packages-select \
  gz_ros2_control \
  robot_sim_description robot_sim_control robot_sim_scenarios \
  robot_sim_moveit_config \
  robot_sim_fanuc_description robot_sim_fanuc_control robot_sim_fanuc_moveit_config \
  robot_sim_sensor_camera robot_sim_sensor_depth robot_sim_sensor_lidar robot_sim_sensor_imu \
  robot_sim_bringup
```

如果只做采集测试，可以按需编译采集相关包：

```bash
colcon build --symlink-install --packages-select \
  robot_task_interfaces acquisition_interfaces simulation_interfaces \
  file_reader
```

## 加载环境

```bash
cd /home/kyle/sany/robot_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 启动仿真

```bash
ros2 launch robot_sim_bringup sim.launch.py
```

完整仿真：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=full
```

启动仿真传感器接收器：

```bash
ros2 launch robot_sim_bringup sensor_receivers.launch.py sim_profile:=panda
```

## 启动采集测试界面

```bash
ros2 run data_collect_ui data_collect_ui
```

## 常用说明

- 当前主入口是 `robot_sim_bringup`；需要图像、点云、激光或 IMU 时使用 `sim_mode:=full` 或按组打开传感器。
- 仿真传感器接收由 `robot_sim_sensors` 完成，并通过 `/diagnostics` 输出健康状态。
- 采集测试 UI 用于验证旧数据采集链路、质量评估和历史数据。
- 旧真实相机和 Fanuc 硬件驱动包已移除，旧硬件后端启动入口本轮暂不维护。
- 真实链路启动时会读取 `nodemanage.yaml`；仿真链路使用 `robot_sim_bringup` 的 launch 参数和 `robot_sim_control/config/panda_controllers.yaml`。
