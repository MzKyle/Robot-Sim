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
ros2 launch data_collect_sim data_collect_sim.launch.py
```

这条命令会启动 gz sim 8、Panda 机械臂、模拟机器人链和默认的仿真桥接。

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

仿真启动时常用的开关是 `use_gazebo`、`use_gz_sensors`、`use_sim_camera_2d`、`use_sim_camera_3d`、`use_sim_fanuc`、`use_tf_to_tcp` 和 `use_gz_joint_control`。

## 脚本启动

```bash
bash src/data_collect/start_data_collect_stack.sh
```
