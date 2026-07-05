# 配置说明

`robot_sim` 当前维护 `schema: 3` 机器人仿真配置。通用 `schema: 4` ROS2 pipeline
验证配置已经移动到同级项目 `robot_validation`。

| kind | 作用 | 常见路径 |
| --- | --- | --- |
| `sim_profile` | 描述机器人如何启动、控制、规划、挂载传感器和验收 smoke 指标 | `robot_sim/profiles/*.yaml` |
| `scene` | 描述完整工况世界、区域、对象、workspace、参数和随机生成器 | `robot_sim/scenes/*.yaml` |
| `world_preset` | 描述 legacy/base-world 资产组合 | `robot_sim/world_presets/*.yaml` |
| `validation_case` | 描述一次 v3 机器人仿真验收 | `robot_sim/validation_cases/*.yaml` |

v1/v2 外部配置不会静默兼容。先迁移到 v3：

```bash
ros2 run robot_sim_bringup migrate_config --input old.yaml --output new.yaml
```

## sim_profile

`sim_profile` 是机器人接入的核心入口。

```yaml
schema: 3
kind: sim_profile
name: panda
metadata:
  package: robot_sim_bringup
  robot_name: panda
capabilities:
  task_families: [empty_motion, pick_place, sensor_calibration, conveyor_sorting]
  sensors: [camera, depth, lidar, imu]
end_effector:
  planning_group: panda_arm
  tool_link: panda_hand_tcp
```

主要字段：

| 字段 | 说明 |
| --- | --- |
| `metadata` | profile 所属 package、robot name、vendor/model 等信息 |
| `capabilities` | 支持的标准任务族和传感器集合 |
| `end_effector` | MoveIt planning group、tool link、可选 gripper 描述 |
| `robot` | xacro、spawn 名称、pose、xacro 参数 |
| `layouts` | 单机/分布式 namespace 与 world 选择 |
| `worlds` | 每个 world source 必须明确为 `scene`、`world_preset` 或 `file` |
| `gazebo` | Gazebo launch、resource path 和启动参数 |
| `control` | controller yaml、controller manager 和 spawner |
| `moveit` | MoveIt launch、SRDF、kinematics、OMPL、RViz |
| `bridges` / `bridge_groups` | ros_gz_bridge topic 声明 |
| `sensors` | 传感器能力、xacro 开关和 receiver 配置 |
| `smoke` | controller、sensor Hz、TF 和轨迹验收规则 |

校验：

```bash
ros2 run robot_sim_bringup profile_lint --profile panda --mode full --require-moveit --require-receivers
ros2 run robot_sim_bringup profile_lint --profile fanuc_m20id12l_industrial_cell --mode full --require-moveit --require-receivers
```

## scene

`scene` 表达可复用工况。它包含 world 物理设置、light、objects、regions、
workspace、可选参数、variant 和 generator。

```yaml
schema: 3
kind: scene
name: industrial_cell
parameters:
  generated_obstacle_count:
    type: integer
    default: 3
variants:
  dense_obstacles:
    parameters:
      generated_obstacle_count: 6
generators:
  - type: random_boxes
    count: ${generated_obstacle_count}
    seed: 11
    region: obstacle_zone
    geometry:
      type: box
      size: [0.12, 0.12, 0.20]
```

## validation_case

`validation_case` 描述一次完整 v3 验收。

```yaml
schema: 3
kind: validation_case
name: industrial_obstacle_clearance
launch:
  profile: fanuc_m20id12l_industrial_cell
  mode: full
  layout: single
  timeout_sec: 120
scene:
  package: robot_sim_scenarios
  path: scenes/industrial_cell.yaml
task:
  type: obstacle_clearance
  seed: 17
  start_region: planning_start
  goal_region: planning_goal
planning_scene:
  apply: true
  exclude_tags: [ground, robot_mount, pedestal, visual_marker, optional]
expect:
  max_goal_position_error_m: 0.30
  max_controller_error_rad: 0.50
  require_tf_ok: true
artifacts:
  rosbag:
    enabled: true
    topic_group: all
  reports: [md, html]
```

`task.type` 必须是标准任务族之一：

```text
empty_motion
obstacle_clearance
fixture_to_pallet
pick_place
sensor_calibration
conveyor_sorting
module_validation
```

`module_validation` 用于接入外部 ROS2 模块，可选字段包括 `module`、`adapters`
和 `expect.module`。详见 [外部模块接入指南](../guide/external-modules.md)。

## 外部 Package 发现

推荐把新机器人放在独立 ROS package 中，而不是改本仓库内置配置。标准路径：

```text
share/<pkg>/robot_sim/profiles/*.yaml
share/<pkg>/robot_sim/validation_cases/*.yaml
share/<pkg>/robot_sim/scenes/*.yaml
share/<pkg>/robot_sim/world_presets/*.yaml
```

运行：

```bash
ros2 run robot_sim_bringup run_case \
  --profile-package my_robot_sim \
  --profile my_robot \
  --case-package my_robot_sim \
  --case smoke_empty_motion \
  --scene-package my_robot_sim
```

如果需要通用 topic/service/TF/process 验证、data source replay、evaluator 或 dataset
工作流，请使用同级项目 `robot_validation`。
