# 场景库

`robot_sim_scenarios` 提供可组合的 Gazebo 场景。`scene` 和 `world_preset` 都是 `schema: 2` 配置。

目录：

```text
worlds/base/
assets/
world_presets/
scenes/
```

profile 可以引用 `scene`、`world_preset` 或 SDF `file`。启动时会生成临时 world 文件并交给 Gazebo。

测试：

```bash
colcon test --packages-select robot_sim_scenarios
```
