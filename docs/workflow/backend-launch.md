# 后端启动

## 启动完整采集栈

```bash
cd /home/kyle/sany/weld_data_collect_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch data_collect_bringup data_collect.launch.py
```

这条命令会启动真实相机、真实 Fanuc 节点、采集核心和采集质量节点。

## 启动仿真采集栈

```bash
cd /home/kyle/sany/weld_data_collect_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_sim_bringup sim.launch.py
```

这条命令会启动轻量仿真：gz sim 8、Panda 机械臂和 Gazebo hardware plugin，默认关闭传感器以节省性能。

## 常用参数覆盖

```bash
ros2 launch data_collect_bringup data_collect.launch.py \
  nodemanage_yaml:=/path/to/nodemanage.yaml \
  robot_ip:=10.16.140.114 \
  fanuc_so_path:=/path/to/libFanucRobot.so
```

## 可选开关

```bash
ros2 launch data_collect_bringup data_collect.launch.py \
  enable_fanuc:=false \
  enable_camera_3d:=false \
  enable_camera_2d:=false
```

仿真启动时常用的开关是 `sim_mode`、`enable_camera`、`enable_depth`、`enable_lidar`、`enable_imu`、`use_moveit`、`rviz`、`headless` 和 `use_sim_time`。

## 脚本启动

```bash
bash src/data_collect/start_data_collect_stack.sh
```
