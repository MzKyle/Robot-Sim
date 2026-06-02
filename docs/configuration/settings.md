# Profile 配置

`sim_profile` 是接入机器人和场景的唯一入口。内置 profile 位于：

```text
src/core/robot_sim_bringup/config/sim_profiles/
```

模板：

```text
src/core/robot_sim_bringup/config/templates/template_robot.yaml
```

## 主要字段

| 字段 | 说明 |
| --- | --- |
| `robot` | xacro、spawn 名称、pose 和 xacro 参数 |
| `layouts` | 单机/分布式 namespace 与 world 选择 |
| `worlds` | scene 或 world preset 引用 |
| `gazebo` | Gazebo launch、resource path 和启动参数 |
| `control` | controller yaml、controller manager 和 spawner |
| `moveit` | MoveIt launch、SRDF、kinematics、OMPL、RViz |
| `bridges` / `bridge_groups` | ros_gz_bridge topic 声明 |
| `sensors` | 传感器能力、xacro 开关、receiver 配置 |
| `smoke` | controller、sensor Hz 和 TF 验收规则 |

## 校验

```bash
ros2 run robot_sim_bringup profile_lint --profile panda --mode full --require-moveit --require-receivers
ros2 run robot_sim_bringup profile_lint --profile fanuc_m20id12l --mode full --require-moveit --require-receivers
```

新增 profile 先通过 lint，再运行 smoke test。
