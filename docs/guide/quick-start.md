# 快速上手

这一页按“第一次把验收跑起来”的路径写。目标不是打开一个演示窗口，而是生成一份可复查的 `report.html`、`metrics.json`、日志和 rosbag。

## 1. 准备环境

支持环境：

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic / `gz sim 8`
- MoveIt2
- `colcon`、`rosdep`、`git`

详细安装见 [环境依赖](prerequisites.md)。每个新终端先加载 ROS 和 Gazebo 版本：

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
```

## 2. 构建

```bash
git clone https://github.com/MzKyle/robot_sim.git robot_sim
cd robot_sim

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

如果构建失败，先确认 `gz sim --versions` 能看到 Harmonic，并确认 ROS 依赖已通过 `rosdep` 安装。

## 3. 跑第一个验收

先跑最小完整闭环：

```bash
ros2 run robot_sim_bringup run_case \
  --case empty_motion \
  --output-dir robot_sim_runs \
  --timeout 120
```

这个用例会启动 Panda full 仿真，检查 controller、joint state、传感器 topic、TF 和 MoveIt，并执行两个空场目标点。

打开报告：

```bash
latest_run="$(ls -td robot_sim_runs/*_empty_motion_panda | head -1)"
xdg-open "${latest_run}/report.html"
```

命令行退出码为 `0` 表示本次验收通过；退出码非 `0` 时也会保留同样的产物目录。

## 4. 看懂产物

每次运行都会创建独立目录：

```text
robot_sim_runs/<UTC timestamp>_<case>_<profile>/
  manifest.json
  effective_case.yaml
  effective_profile.yaml
  robot.urdf
  metrics.json
  validation_metrics.json
  report.md
  report.html
  logs/
    sim.launch.log
    profile_lint.log
    moveit.log
    validation_case.log
  rosbag/
```

常用入口：

| 文件 | 用途 |
| --- | --- |
| `report.html` | 给人看的验收报告，包含通过/失败摘要、步骤表、指标表和日志路径 |
| `metrics.json` | 给脚本和 CI 用的结构化指标 |
| `manifest.json` | 本次运行的 case、profile、scene、命令、git commit、开始/结束时间和产物路径 |
| `logs/sim.launch.log` | Gazebo、controller、MoveIt、bridge 等启动日志 |
| `effective_case.yaml` | 应用 CLI 覆盖和 scene 参数后的最终 case |
| `rosbag/metadata.yaml` | rosbag 元数据，默认开启录制 |

## 5. 跑工业用例

```bash
ros2 run robot_sim_bringup run_case \
  --case industrial_obstacle_clearance \
  --output-dir robot_sim_runs \
  --timeout 120

ros2 run robot_sim_bringup run_case \
  --case industrial_fixture_to_pallet \
  --output-dir robot_sim_runs \
  --timeout 120
```

这两个用例使用 `fanuc_m20id12l_industrial_cell` profile 和 `industrial_cell` scene，会应用 collision objects，并执行 MoveIt 规划/轨迹。

## 6. 使用 scene 参数

部分 scene 支持参数、variant 和确定性随机生成：

```bash
ros2 run robot_sim_bringup run_case \
  --case industrial_obstacle_clearance \
  --scene-variant dense_obstacles \
  --scene-param generated_obstacle_count=6 \
  --scene-param seed=41 \
  --output-dir robot_sim_runs
```

参数只允许引用 YAML 中声明过的字段；不执行任意 Python 代码。

## 7. 启动交互式仿真

只想打开仿真环境时使用 launch：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=panda sim_mode:=light
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=panda sim_mode:=full
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l_industrial_cell sim_mode:=full
```

| 模式 | 用途 |
| --- | --- |
| `mock` | 不启动 Gazebo，快速验证 launch、controller 和 action |
| `light` | 启动 Gazebo 和控制链，默认关闭传感器、MoveIt 和 RViz |
| `full` | 启动完整 Gazebo、传感器、MoveIt、RViz 链路 |

## 8. 接入自己的机器人

先生成模板：

```bash
ros2 run robot_sim_bringup scaffold_robot \
  --package my_robot_sim \
  --robot-name my_robot \
  --output /tmp \
  --planning-group manipulator \
  --tool-link tool0 \
  --joint-names joint_1 joint_2 joint_3 joint_4 joint_5 joint_6 \
  --sensor-set camera,depth,lidar,imu \
  --with-gripper true
```

构建外部 package 后，用 package 名发现配置：

```bash
ros2 run robot_sim_bringup profile_lint \
  --profile-package my_robot_sim \
  --profile my_robot \
  --mode full \
  --require-moveit

ros2 run robot_sim_bringup run_case \
  --profile-package my_robot_sim \
  --profile my_robot \
  --case-package my_robot_sim \
  --case smoke_empty_motion
```

标准发现路径是：

```text
share/<pkg>/robot_sim/profiles/*.yaml
share/<pkg>/robot_sim/validation_cases/*.yaml
share/<pkg>/robot_sim/scenes/*.yaml
```

## 9. Deb 安装后的入口

```bash
robot-sim-check
robot-sim run-case --case empty_motion --output-dir robot_sim_runs
robot-sim sim_profile:=panda sim_mode:=light
```

`robot-sim` 无子命令时等价于 `ros2 launch robot_sim_bringup sim.launch.py`；`run-case`、`migrate-config`、`scaffold-robot` 会转发到对应 Python CLI。

## 10. 第一次失败时看哪里

先看 `report.html` 的 Steps 表，找到第一个 `FAIL`。常见定位方式：

| 失败步骤 | 优先检查 |
| --- | --- |
| `profile_lint` | profile 路径、ROS package 名、schema 版本、MoveIt/controller 配置 |
| `render_urdf` / `validate_urdf` | xacro 参数、mesh 路径、ros2_control 标签 |
| `simulation_start` | `logs/sim.launch.log`、Gazebo resource path、插件是否存在 |
| `controllers_active` | controller yaml、controller manager namespace、joint 名称 |
| `sensor_hz` | bridge topic、sensor xacro 开关、receiver 是否启用 |
| `tf_tree` | robot_state_publisher、URDF link、fixed joint、frame 名 |
| `moveit_plan_execute` | SRDF、planning group、IK、场景碰撞对象、目标点可达性 |

更多排查见 [常见问题](../faq/troubleshooting.md) 和 [日志与产物](../logging/data-storage.md)。
