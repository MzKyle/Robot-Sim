# 接口包

这两个包当前只生成 ROS message/service 类型，没有安装 client/server 节点，也没有被
`run_case` 主链自动调用。外部项目可以依赖这些类型定义，但服务行为需要自行实现。

## robot_task_interfaces

用于描述通用任务上下文。

包含：

- `TaskContext.msg`
- `TaskStatus.msg`
- `SetTaskContext.srv`

## simulation_interfaces

用于描述仿真场景和资产。

包含：

- `ScenarioAsset.msg`
- `SimulationScenario.msg`
- `SetSimulationScenario.srv`
