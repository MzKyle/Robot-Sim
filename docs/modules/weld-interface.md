# ROS 接口包

`weld_interface` 现在定位为焊接业务 adapter 和旧接口兼容层。通用任务、采集和仿真接口分别放在 `robot_task_interfaces`、`acquisition_interfaces` 和 `simulation_interfaces`。

## 主要内容

- 消息定义：位姿、焊接寄存器、焊接扩展字段和旧状态结构体。
- 服务定义：旧任务设置、焊接扩展任务设置、3D 扫描和辅助调用。
- 共享常量：话题名、服务名和配置约定。

## 使用建议

- 新增通用节点时优先使用中性接口包。
- 只有焊接专有字段才进入 `weld_interface`，例如 `weld_seam_id`、`weld_id` 和 `weld_layer`。
- 旧 UI 或旧采集流程仍可继续使用 `/data_collect_set_task` 和 `/data_collect_status`。
