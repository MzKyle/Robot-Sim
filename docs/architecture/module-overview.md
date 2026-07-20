# 模块全景

| 模块 | 职责 |
| --- | --- |
| `robot_sim_bringup.common` | registry、schema 校验和配置迁移 |
| `robot_sim_bringup.robot_domain` | `schema: 3` 机器人仿真 launch、profile、case、lint 和 smoke helper |
| `robot_sim_bringup.legacy_integrations` | 焊接/FANUC 旧外部模块兼容 adapter 和 runner |
| `robot_sim_bringup.scaffold` | 外部机器人模板生成 |
| `robot_sim_description` | 机器人 xacro、mesh、传感器挂载和 ros2_control 标签 |
| `robot_sim_control` | controller manager 与各机器人 controller 配置 |
| `robot_sim_moveit_config` | SRDF、kinematics、joint limits、OMPL、MoveIt controller 和 RViz |
| `robot_sim_scenarios` | scene library、base world、assets、world presets |
| `robot_sim_sensor_*` | 可选的仿真传感器 receiver 与 diagnostics，不由主 launch 自动启动 |
| `robot_task_interfaces` | 通用任务上下文类型声明 |
| `simulation_interfaces` | 通用仿真 scenario 类型声明 |
| `gz_ros2_control` | Gazebo Harmonic 与 ros2_control 的 system plugin |

顶层目录按职责分组：

```text
examples/
integrations/
src/core/
src/sensors/
src/interfaces/
src/vendor/
```

`examples/robot_arm` 是内置机器人示例；`integrations/welding` 提供 legacy case，
`integrations/auto_cover` 当前仅保留目录骨架。通用 v4 验证模块已拆到同级项目
`robot_validation`。
