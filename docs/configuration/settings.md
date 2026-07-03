# 配置说明

`robot_sim` 的配置不是隐式 YAML 协议，而是四类一等配置对象。第二阶段起，内置配置使用 `schema: 3` 和 `kind`，加载时先做 JSON Schema 校验，再做路径、ROS package、scene region、MoveIt group 等语义校验。

| kind | 作用 | 常见路径 |
| --- | --- | --- |
| `sim_profile` | 描述一个机器人如何启动、控制、规划、挂载传感器和验收 smoke 指标 | `robot_sim/profiles/*.yaml` |
| `scene` | 描述完整工况世界、区域、对象、workspace、参数和随机生成器 | `robot_sim/scenes/*.yaml` |
| `world_preset` | 描述 legacy/base-world 资产组合 | `robot_sim/world_presets/*.yaml` |
| `validation_case` | 描述一次验收：启动参数、场景、任务、期望指标和输出产物 | `robot_sim/validation_cases/*.yaml` |

v1/v2 外部配置不会静默兼容。先迁移：

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
| `worlds` | 每个 world source 必须明确为 `scene`、`world_preset` 或 `file` 三选一 |
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

外部 profile：

```bash
ros2 run robot_sim_bringup profile_lint \
  --profile-package my_robot_sim \
  --profile my_robot \
  --mode full \
  --require-moveit
```

## scene

`scene` 表达可复用工况。它包含 world 物理设置、light、objects、regions、workspace、可选参数、variant 和 generator。

```yaml
schema: 3
kind: scene
name: industrial_cell
parameters:
  generated_obstacle_count:
    type: integer
    default: 3
  seed:
    type: integer
    default: 11
variants:
  dense_obstacles:
    parameters:
      generated_obstacle_count: 6
generators:
  - type: random_boxes
    count: ${generated_obstacle_count}
    seed: ${seed}
    region: obstacle_zone
    geometry:
      type: box
      size: [0.12, 0.12, 0.20]
```

参数化规则：

- `${name}` 只能引用 `parameters` 中声明过的参数。
- CLI `--scene-param name=value` 只能覆盖已声明参数。
- variant 只能覆盖参数，不执行任意 Python。
- 当前内置 generator 支持 `random_boxes`，固定 seed 下可复现。
- generator 的 region 必须存在于 scene `regions` 中。

运行 variant：

```bash
ros2 run robot_sim_bringup run_case \
  --case industrial_obstacle_clearance \
  --scene-variant dense_obstacles \
  --scene-param generated_obstacle_count=6 \
  --scene-param seed=41
```

## world_preset

`world_preset` 用于组合 base world 和 assets，适合保留已有 world 资产：

```yaml
schema: 3
kind: world_preset
name: planning_obstacles
base_world: worlds/base/lab.world.sdf
includes:
  - uri: model://planning_column
    name: planning_column_1
    pose: [0.7, 0.0, 0.5, 0, 0, 0]
```

新工况优先使用 `scene`；只有需要复用 legacy/base-world 资产时再使用 `world_preset`。

## validation_case

`validation_case` 描述一次完整验收。

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
  moveit:
    group: manipulator
    target_link: tool0
    frame: world
    planning_time_sec: 10.0
    velocity_scaling: 0.1
    acceleration_scaling: 0.1
    execute: true
planning_scene:
  apply: true
  exclude_tags: [ground, robot_mount, pedestal, visual_marker, optional]
  include_tags: []
expect:
  position_tolerance_m: 0.15
  orientation_tolerance_rad: 3.14159
  max_goal_position_error_m: 0.30
  min_tcp_clearance_m: 0.05
  max_controller_error_rad: 0.50
  required_sensor_min_hz: 1.0
  require_tf_ok: true
  topics:
    - name: /camera/color/image_raw
      min_hz: 1.0
artifacts:
  rosbag:
    enabled: true
    topic_group: all
    compression: false
    extra_topics: []
  reports: [md, html]
```

主要字段：

| 字段 | 说明 |
| --- | --- |
| `launch` | profile、mode、layout、timeout、sensor overrides |
| `scene` | scene 名称或 `package` + `path`，可带 variant 和 parameters |
| `task` | 标准任务族、seed、region/waypoints、MoveIt、gripper、conveyor、object 等任务参数 |
| `planning_scene` | `apply` 控制是否向 MoveIt 应用 collision objects，`include_tags`/`exclude_tags` 控制对象过滤 |
| `expect` | 位姿容差、controller error、全局 sensor Hz、必需 topic、TF 和 TCP clearance 阈值 |
| `artifacts` | rosbag 开关、topic group、压缩开关、额外 topic 和 report 格式 |

`task.type` 必须是标准任务族之一：

```text
empty_motion
obstacle_clearance
fixture_to_pallet
pick_place
sensor_calibration
conveyor_sorting
```

`task.moveit.execute: false` 表示只做规划、场景、TF、传感器和业务事件级验收，不向控制器发送最终执行轨迹。内置 `panda_pick_place`、`sensor_calibration`、`conveyor_sorting` 默认使用这个模式。

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

低层 escape hatch：

```bash
ros2 run robot_sim_bringup run_case \
  --profile-file /path/to/profile.yaml \
  --case /path/to/case.yaml \
  --scene /path/to/scene.yaml
```

## 机器人接入模板

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

生成目录包含：

```text
my_robot_sim/
  robot_sim/
    profiles/
    validation_cases/
    scenes/
  description/
  control/
  moveit_config/
```

生成后建议顺序：

1. 替换 description 中的真实 xacro/mesh。
2. 对齐 controller yaml 和 joint 名称。
3. 补齐 MoveIt planning group、SRDF、kinematics 和 OMPL。
4. 根据硬件能力调整 sensors 和 bridge topics。
5. 先跑 `profile_lint`，再跑 smoke case。

## 内置配置位置

```text
src/core/robot_sim_bringup/config/sim_profiles/
src/core/robot_sim_bringup/config/validation_cases/
src/core/robot_sim_bringup/config/templates/
src/core/robot_sim_scenarios/scenes/
src/core/robot_sim_scenarios/world_presets/
src/core/robot_sim_scenarios/schemas/
```
