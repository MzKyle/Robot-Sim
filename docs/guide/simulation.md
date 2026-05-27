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

传感器按组控制，`full` 模式默认开启 profile 中声明的传感器；也可以用 `sensor_overrides` 覆盖单个 group：

- `camera`：RGB 图像和 CameraInfo。
- `depth`：深度图、深度 CameraInfo 和点云。
- `lidar`：2D scan 和 3D lidar 点云。
- `imu`：IMU。

示例：

```bash
ros2 launch robot_sim_bringup sim.launch.py \
  sim_mode:=light \
  sensor_overrides:=camera=true,depth=false
```

## World 组件化

`sim_profile` 中的 world 入口指向 `robot_sim_scenarios/scenarios/*.yaml`，launch 会把场景组合成临时 SDF world 后交给 Gazebo。目录约定：

- `worlds/base/`：基础 world，包含物理、光照、地面和 GUI。
- `assets/`：可复用 SDF 组件，例如桌子、工件、标定板、目标物和障碍物。
- `scenarios/`：业务或测试场景组合，例如 `lab_demo`、`welding_demo`、`calibration_demo`、`sensor_test`、`planning_obstacles`。

新增场景时优先复制一个 scenario YAML，只替换 asset 列表和位姿；新增实体时放到 `assets/<name>/model.sdf`。

## 控制链路

- `mock` 模式使用 `mock_components/GenericSystem` 和 ROS 侧 `ros2_control_node`。
- `light`、`full` 模式使用 `gz_ros2_control/GazeboSimSystem`，由 Gazebo 内的 `libgz_ros2_control-system.so` 创建 controller manager。
- Gazebo 模式不再使用 joint command bridge，也不再启动 `joint_state_to_gz_joint_cmd_node`。

## 固定验收

```bash
gz sdf -k src/robot_sim_scenarios/worlds/base/lab.world.sdf
python3 -c "from robot_sim_bringup.sim_config_loader import load_sim_profile; print(load_sim_profile('panda')['worlds']['single']['path'])"
scripts/sim_smoke_test.sh --profile panda --mode full --timeout 120
```

`sim_smoke_test.sh` 会自行启动 `robot_sim_bringup sim.launch.py`，默认强制 headless、关闭 RViz 和 MoveIt，只把 MoveIt 与 rosbag 作为显式可选项：

```bash
scripts/sim_smoke_test.sh --profile panda --mode full --with-moveit
scripts/sim_smoke_test.sh --profile panda --mode full --with-rosbag --keep-logs
scripts/sim_smoke_test.sh --profile-file /path/to/custom_robot.yaml --mode full
```

通过标准：

- xacro 可生成 URDF，且 `check_urdf` 通过。
- Gazebo 中能看到 profile 的 `spawn_name`。
- `/joint_states` 有数据，并包含主轨迹控制器的 joints。
- profile 中默认启用的 controller 均为 `active`。
- 主 `FollowJointTrajectory` action 可以成功执行一条保持当前位置的短轨迹。
- 当前启用 sensor group 对应的 ROS topic 有消息，平均频率大于 1 Hz。
- URDF links 与启用传感器声明的静态 TF frame 处于同一连通 TF tree。

新增机器人时，profile 至少需要正确声明 robot xacro、spawn 名称、controller spawner、controller joints、sensor bridge group 和静态 TF；否则 smoke test 会在对应步骤给出失败项和日志路径。
