# 扩展方向

后续扩展保持以下边界：

- 新机器人通过新增 profile、URDF/xacro、controller 和 MoveIt 配置接入。
- 新传感器通过 `robot_sim_sensor_*` receiver 包和 profile bridge 声明接入。
- 新场景通过 `robot_sim_scenarios` 的 assets、scenes 和 world presets 接入。
- 新非机器人项目优先通过 `schema: 4` 的 system/data_source/adapter/suite 接入。
- legacy welding adapter 只保留兼容，不继续扩展为核心平台能力。
- CI 保持 PR 快速，重仿真检查放到手动或定时 workflow。

不建议把机器人型号差异写进 launch 分支；profile 应该是主要扩展面。
