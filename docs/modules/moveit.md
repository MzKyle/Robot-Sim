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
