# 开发运行

## 拉取源码

```bash
git clone --recursive https://github.com/MzKyle/robot_sim.git robot_sim
cd robot_sim
```

如果已经 clone 但没有 submodule：

```bash
git submodule update --init --recursive
```

## 构建

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic

colcon build --symlink-install \
  --allow-overriding gz_ros2_control \
  --packages-select \
    gz_ros2_control \
    robot_sim_description robot_sim_control robot_sim_scenarios \
    robot_sim_moveit_config \
    robot_sim_sensor_camera robot_sim_sensor_depth robot_sim_sensor_lidar robot_sim_sensor_imu \
    robot_sim_bringup robot_task_interfaces simulation_interfaces

source install/setup.bash
```

## 启动

轻量模式：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=light
```

完整模式：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=full
```

Fanuc profile：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l sim_mode:=full
```

mock 控制链：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=mock
```

## 常用检查

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic hz /camera/color/image_raw
ros2 run robot_sim_bringup profile_lint --profile panda --mode full --require-moveit
```
