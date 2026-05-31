# robot_sim Gazebo 仿真工作空间
----
> 面向 ROS 2 Humble + gz sim 8/Harmonic 的机器人仿真、运控、传感器和采集测试文档

<p align="center">
  <a href="#主要特性"><img alt="ROS 2" src="https://img.shields.io/badge/ROS%202-Humble-0f766e?style=flat-square" /></a>
  <img alt="Gazebo" src="https://img.shields.io/badge/Gazebo-Harmonic%20%7C%20gz%20sim%208-E95420?style=flat-square" />
  <img alt="ros2_control" src="https://img.shields.io/badge/Control-ros2__control-2E7D32?style=flat-square" />
  <img alt="MoveIt2" src="https://img.shields.io/badge/Planning-MoveIt2-2D6CDF?style=flat-square" />
</p>

<p align="center">
  <a href="#主要特性">主要特性</a> ·
  <a href="#系统架构">系统架构</a> ·
  <a href="#快速导航">快速导航</a> ·
  <a href="guide/simulation.md">仿真方案</a> ·
  <a href="workflow/testing.md">测试验收</a>
</p>

## 项目简介

`robot_sim` 当前以 Gazebo 仿真和机器人运控验证为主。核心入口是 `robot_sim_bringup`，它把 Panda/Fanuc 机械臂模型、Gazebo world、`gz_ros2_control`、MoveIt2、RViz2、`ros_gz_bridge` 和仿真传感器接收包组织成统一仿真链路。

旧真实硬件相机和 Fanuc 驱动包已移除。离线数据检验能力已经迁出到 [/home/kyle/sany/Offline_data_tool](/home/kyle/sany/Offline_data_tool)，`robot_sim` 现在只保留仿真主链路和录包辅助入口。

## 主要特性

- 提供 `mock`、`light`、`full` 三档仿真模式。
- 使用 `gz_ros2_control/GazeboSimSystem` 在 Gazebo 中创建 controller manager。
- 提供 `joint_state_broadcaster`、`arm_controller` 和可选 `gripper_controller`。
- 支持 MoveIt2 规划执行和 RViz2 可视化。
- 支持 RGB、深度、点云、2D LaserScan、3D lidar 点云和 IMU 话题桥接。
- 提供 `robot_sim_sensors` C++ receiver，订阅仿真传感器话题并发布 `/diagnostics`。
- 提供 ROS 2 录包辅助入口，可按运控、传感器、全量和分布式话题组录制 rosbag2。
- 提供本机分布式 launch，用于模拟 robot、sensors、supervisor 进程拆分。
- 保留数据采集测试包，可用于验证仿真传感器话题和后续采集流程。

## 系统架构

```mermaid
flowchart LR
    DESC[robot_sim_description\nPanda xacro + meshes + sensors]
    WORLD[robot_sim_scenarios\nGazebo worlds]
    BRINGUP[robot_sim_bringup\nsim.launch.py]
    GZ[gz sim 8\nHarmonic]
    CTRL[robot_sim_control\nros2_control]
    MOVEIT[robot_sim_moveit_config\nMoveIt2/RViz2]
    BRIDGE[ros_gz_bridge\nsensor topics]
    RECV[robot_sim_sensors\nreceiver diagnostics]

    BRINGUP --> DESC
    BRINGUP --> WORLD
    BRINGUP --> GZ
    GZ --> CTRL
    CTRL --> MOVEIT
    GZ --> BRIDGE
    BRIDGE --> RECV
```

## 目录一览

| 路径 | 说明 |
| --- | --- |
| `src/robot_sim_bringup/` | 仿真总入口、三档模式、传感器桥接和本机分布式启动 |
| `src/robot_sim_description/` | Panda 模型、传感器挂载、Gazebo 插件和 mesh 资源 |
| `src/robot_sim_control/` | ros2_control 控制器配置 |
| `src/robot_sim_fanuc_description/` | Fanuc M-20iD/12L Gazebo 描述和官方 DAE/STL mesh 资源 |
| `src/robot_sim_fanuc_control/` | Fanuc M-20iD/12L controller 配置 |
| `src/robot_sim_fanuc_moveit_config/` | Fanuc M-20iD/12L MoveIt2 配置 |
| `src/robot_sim_sensors/` | camera、depth、lidar、imu 仿真传感器接收包 |
| `src/robot_sim_scenarios/` | base world、assets 和 scenario 场景 |
| `src/robot_sim_moveit_config/` | MoveIt2 规划和执行配置 |
| `src/gz_ros2_control/` | Humble + gz sim 8/Harmonic 的源码 overlay |
| `src/simulation_interfaces/` | 通用仿真 scenario 接口 |
| `src/robot_task_interfaces/` | 通用任务上下文接口 |
| `src/acquisition_interfaces/` | 通用采集状态、质量和任务接口 |
| `src/data_collect*` | 已移除：旧数据采集链路已迁出到离线数据检验项目 |

## 快速导航

- [仿真方案](guide/simulation.md) - 三档模式、传感器开关和控制链说明。
- [ROS 2 录包辅助](guide/rosbag-recording.md) - 按预设话题组录制仿真和运控数据。
- [开发运行](guide/run-app.md) - 编译工作空间并启动主要入口。
- [环境依赖与准备](guide/prerequisites.md) - ROS、Gazebo 和系统依赖。
- [模块总览](modules/README.md) - 快速找到每个包的职责。
- [接口参考](interfaces/ros-api.md) - ROS 主题、服务和 action 索引。
- [测试验收流程](workflow/testing.md) - 启动后检查控制器、话题和采集测试链路。
