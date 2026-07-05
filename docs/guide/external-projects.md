# 外部机器人项目资产规范

外部 ROS package 可以提供 `robot_sim` v3 机器人仿真资产，不需要改核心 runner。

推荐目录：

```text
share/<pkg>/robot_sim/
  profiles/
  validation_cases/
  scenes/
  world_presets/
```

## 推荐模型

- `profiles/`：放 `schema: 3`、`kind: sim_profile`，描述机器人、控制、MoveIt、传感器和 bridge。
- `validation_cases/`：放 `schema: 3`、`kind: validation_case`，描述一次机器人仿真验收。
- `scenes/`：放工况 scene。
- `world_presets/`：仅在复用 legacy/base world 资产时使用。

通用 ROS2 pipeline 验证、topic/service replay、dataset manifest 和 evaluator 已迁移到同级项目
`robot_validation`。

## 脚手架

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

生成的 package 会包含 `package.xml`、`CMakeLists.txt` 和标准 `robot_sim/` 目录，可以直接由 colcon 安装。

## 运行

```bash
ros2 run robot_sim_bringup run_case \
  --case-package my_robot_sim \
  --case smoke_empty_motion \
  --profile-package my_robot_sim \
  --profile my_robot \
  --scene-package my_robot_sim \
  --output-dir robot_sim_runs
```

也可以传入直接 YAML 路径作为 escape hatch。维护新项目时优先只改外部 package 里的
YAML、URDF/xacro、controller 和 MoveIt 配置。
