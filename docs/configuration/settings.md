# Profile 配置

`sim_profile` 是接入机器人和场景的唯一入口。第二阶段后内置 profile 使用 `schema: 3` 和 `kind: sim_profile`。

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
| `metadata` | profile 所属 package、robot name、vendor/model |
| `capabilities` | 支持的标准任务族和传感器集合 |
| `end_effector` | MoveIt planning group、tool link、可选 gripper controller |
| `robot` | xacro、spawn 名称、pose 和 xacro 参数 |
| `layouts` | 单机/分布式 namespace 与 world 选择 |
| `worlds` | 明确使用 `scene`、`world_preset` 或 `file` 三选一 |
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

外部 profile package 使用标准路径：

```text
share/<pkg>/robot_sim/profiles/*.yaml
share/<pkg>/robot_sim/validation_cases/*.yaml
share/<pkg>/robot_sim/scenes/*.yaml
```

示例：

```bash
ros2 run robot_sim_bringup run_case \
  --profile-package my_robot_sim \
  --profile my_robot \
  --case-package my_robot_sim \
  --case smoke_empty_motion
```

## Validation Case

`validation_case` 使用 `schema: 3` 和 `kind: validation_case`，用于描述单次验收运行。

| 字段 | 说明 |
| --- | --- |
| `launch` | profile、mode、layout、timeout 和 sensor overrides |
| `scene` | 验收使用的 scene YAML |
| `task` | 标准任务族、随机种子、任务区域和 MoveIt/gripper/conveyor 参数 |
| `planning_scene` | 是否将 scene collision objects 应用到 MoveIt |
| `expect` | topic、TF、误差、sensor Hz 和 clearance 阈值 |
| `artifacts` | rosbag 和 report 输出设置 |

运行：

```bash
ros2 run robot_sim_bringup run_case --case industrial_fixture_to_pallet --output-dir robot_sim_runs
```

标准任务族：

```text
empty_motion
obstacle_clearance
fixture_to_pallet
pick_place
sensor_calibration
conveyor_sorting
```

scene 可使用参数化 variant：

```bash
ros2 run robot_sim_bringup run_case \
  --case industrial_obstacle_clearance \
  --scene-variant dense_obstacles \
  --scene-param generated_obstacle_count=6 \
  --scene-param seed=41
```

v1/v2 配置不再静默兼容，先迁移：

```bash
ros2 run robot_sim_bringup migrate_config --input old.yaml --output new.yaml
```
