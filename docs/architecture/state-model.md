# 仿真状态模型

smoke test 将仿真验收拆成稳定的阶段：

1. profile lint：检查路径、controller、MoveIt、receiver 和 TF 声明。
2. URDF 渲染：运行 xacro 并用 `check_urdf` 校验。
3. 启动仿真：根据 profile 与 mode 运行 `sim.launch.py`。
4. Gazebo spawn：确认模型出现在 Gazebo。
5. 控制链：检查 `/joint_states`、controller active 和 trajectory action。
6. 传感器：检查启用的 bridge topic Hz。
7. TF：确认 URDF link 和传感器 frame 在同一棵树中。
8. 可选 MoveIt：执行一次 plan/execute。
9. 可选 rosbag：录制短 bag 并检查 metadata。

CI 默认跑到 mock smoke；手动/定时 workflow 会覆盖 full Gazebo 与 MoveIt。
