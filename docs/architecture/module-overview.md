# 模块全景

| 模块 | 职责 |
| --- | --- |
| `robot_sim_bringup` | 统一 launch、profile 解析、lint、smoke helper |
| `robot_sim_description` | 机器人 xacro、mesh、传感器挂载和 ros2_control 标签 |
| `robot_sim_control` | controller manager 与各机器人 controller 配置 |
| `robot_sim_moveit_config` | SRDF、kinematics、joint limits、OMPL、MoveIt controller 和 RViz |
| `robot_sim_scenarios` | scene library、base world、assets、world presets |
| `robot_sim_sensor_*` | 仿真传感器 receiver 与 diagnostics |
| `robot_task_interfaces` | 通用任务上下文接口 |
| `simulation_interfaces` | 通用仿真 scenario 接口 |
| `gz_ros2_control` | Gazebo Harmonic 与 ros2_control 的 system plugin |

顶层目录按职责分组：

```text
src/core/
src/sensors/
src/interfaces/
src/vendor/
```
