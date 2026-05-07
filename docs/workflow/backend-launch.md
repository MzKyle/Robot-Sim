# 后端启动

## 启动完整采集栈

```bash
cd /home/kyle/sany/weld_data_collect_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch data_collect_bringup data_collect.launch.py
```

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

## 脚本启动

```bash
bash src/data_collect/start_data_collect_stack.sh
```
