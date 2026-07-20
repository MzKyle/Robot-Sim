# MoveIt 配置

`robot_sim_moveit_config` 按机器人组织 MoveIt 配置：

```text
config/robots/<robot>/
rviz/robots/<robot>.rviz
```

每个机器人通常包含：

- SRDF
- kinematics
- joint limits
- OMPL planning
- MoveIt controller
- RViz MotionPlanning 配置

Fanuc 使用 `manipulator` planning group；Panda 使用 `panda_arm` planning group。

工业场景包含窄夹具和立柱。Fanuc 的 OMPL
`longest_valid_segment_fraction` 当前为 `0.01`，用于避免规划器以过粗的关节空间
插值跨过障碍物、到最终路径校验时才返回 `INVALID_MOTION_PLAN (-2)`。新增窄障碍物时
应保留或进一步收紧碰撞离散分辨率，并用真实 `run_case` 回归。
