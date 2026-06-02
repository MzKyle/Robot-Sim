# 模块总览

本项目按职责组织 ROS 包：

- `core`：仿真主链路、机器人描述、控制、MoveIt 和场景。
- `sensors`：仿真传感器 receiver。
- `interfaces`：通用消息与服务。
- `vendor`：外部源码 overlay。

## 包列表

| 包 | 路径 | 说明 |
| --- | --- | --- |
| `robot_sim_bringup` | `src/core/robot_sim_bringup` | 启动、profile、lint、smoke |
| `robot_sim_description` | `src/core/robot_sim_description` | 机器人描述和模型资源 |
| `robot_sim_control` | `src/core/robot_sim_control` | ros2_control 配置 |
| `robot_sim_moveit_config` | `src/core/robot_sim_moveit_config` | MoveIt 与 RViz 配置 |
| `robot_sim_scenarios` | `src/core/robot_sim_scenarios` | 场景库 |
| `robot_sim_sensor_*` | `src/sensors` | 传感器 receiver |
| `robot_task_interfaces` | `src/interfaces/robot_task_interfaces` | 任务上下文接口 |
| `simulation_interfaces` | `src/interfaces/simulation_interfaces` | 仿真 scenario 接口 |
