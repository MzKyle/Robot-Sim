# 模块总览

本章节按包划分说明每个模块的职责和入口。当前项目以 `robot_sim_*` 仿真运控链路为主，旧硬件相机和 Fanuc 驱动包已移除；采集和 UI 模块作为旧业务链路测试辅助保留。

## 阅读建议

先看以下内容：

1. [仿真入口](data-collect-sim.md)
2. [Bringup 入口](data-collect-bringup.md)
3. [采集核心](data-collect.md)
4. [采集质量节点](data-collect-quality.md)
5. [桌面操作台](data-collect-ui.md)
6. 仿真传感器 receiver：`src/robot_sim_sensors/robot_sim_sensor_*`
