# Fanuc 机器人节点

> 已废弃：旧 `fanuc_robot` 真实硬件驱动包已从工作空间移除。

当前 Fanuc M20iD/12L 仿真资源已并入通用 `robot_sim_description`、`robot_sim_control` 和 `robot_sim_moveit_config` 包，通过 `sim_profile:=fanuc_m20id12l` 接入。

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l sim_mode:=full
```

真实 Fanuc 控制器适配后续应作为新的硬件 adapter 独立设计，不再复用本次移除的 SDK 耦合包。
