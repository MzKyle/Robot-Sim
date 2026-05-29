# Fanuc 机器人节点

> 已废弃：旧 `fanuc_robot` 真实硬件驱动包已从工作空间移除。

当前 Fanuc 仿真模板使用 `robot_sim_fanuc_description`、`robot_sim_fanuc_control` 和 `robot_sim_fanuc_moveit_config`，通过 `sim_profile:=fanuc_m20ia10l` 接入。

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20ia10l sim_mode:=full
```

真实 Fanuc 控制器适配后续应作为新的硬件 adapter 独立设计，不再复用本次移除的 SDK 耦合包。
