# 仿真验收状态模型

smoke test 和 validation case 将仿真验收拆成稳定的阶段：

1. profile lint：检查路径、controller、MoveIt、receiver 和 TF 声明。
2. URDF 渲染：运行 xacro 并用 `check_urdf` 校验。
3. 启动仿真：根据 profile 与 mode 运行 `sim.launch.py`，`mock` 使用 mock control，不启动 Gazebo。
4. Gazebo spawn：`light/full` 确认模型出现在 Gazebo，`mock` 跳过。
5. 控制链：检查 `/joint_states`、controller active 和 trajectory action。
6. 传感器：检查启用的 bridge topic Hz。
7. TF：确认 URDF link 和传感器 frame 在同一棵树中。
8. MoveIt：执行 plan/execute 或 validation case 的区域目标。
9. 指标：采集 goal error、controller error、sensor Hz、TCP clearance 和 MoveIt 结果。
10. 产物：录制 rosbag，生成 `manifest.json`、`metrics.json`、`report.md/html`。

CI 默认跑到 mock smoke；手动/定时 workflow 会覆盖 full Gazebo、MoveIt 和工业 validation case。
