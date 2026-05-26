# 仿真方案

本文档对齐 `robot_sim_*` 包族。旧 `data_collect_sim` 链路已移除，仿真统一从 `robot_sim_bringup` 启动。

## 启动前准备

```bash
cd /home/kyle/sany/robot_sim
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
colcon build --symlink-install --allow-overriding gz_ros2_control --packages-select \
  gz_ros2_control \
  robot_sim_description robot_sim_control robot_sim_scenarios \
  robot_sim_moveit_config robot_sim_bringup
source install/setup.bash
```

`gz sim 8` 是 Harmonic。Humble 的 apt 版 `ros-humble-gz-ros2-control` 面向 Fortress，Humble + Harmonic 需要在本工作空间内源码构建 `gz_ros2_control`，否则 Gazebo 会加载到不匹配的旧插件 ABI。

## 三种模式

```bash
# 轻量模式，默认：Gazebo + gz_ros2_control，传感器关闭
ros2 launch robot_sim_bringup sim.launch.py

# mock 模式：不启动 Gazebo，只验证 ROS 2 控制链
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=mock

# 完整模式：Gazebo + 全部传感器 + MoveIt2/RViz2
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=full
```

## 传感器开关

传感器按组控制，所有开关都支持 `auto|true|false`：

- `enable_camera`：RGB 图像和 CameraInfo。
- `enable_depth`：深度图、深度 CameraInfo 和点云。
- `enable_lidar`：2D scan 和 3D lidar 点云。
- `enable_imu`：IMU。

示例：

```bash
ros2 launch robot_sim_bringup sim.launch.py \
  sim_mode:=light \
  enable_camera:=true \
  enable_depth:=false
```

## 控制链路

- `mock` 模式使用 `mock_components/GenericSystem` 和 ROS 侧 `ros2_control_node`。
- `light`、`full` 模式使用 `gz_ros2_control/GazeboSimSystem`，由 Gazebo 内的 `libgz_ros2_control-system.so` 创建 controller manager。
- Gazebo 模式不再使用 joint command bridge，也不再启动 `joint_state_to_gz_joint_cmd_node`。

## 常用校验

```bash
gz sdf -k src/robot_sim_scenarios/worlds/robot_lab.world.sdf
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], points: [{positions: [0.2, -0.6, 0.1, -2.2, 0.1, 1.4, 0.6], time_from_start: {sec: 2}}]}}"
```
