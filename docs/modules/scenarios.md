# 场景库

`robot_sim_scenarios` 提供可组合的 Gazebo 场景。`scene` 和 `world_preset` 都是 `schema: 3` 配置。

目录：

```text
worlds/base/
assets/
world_presets/
scenes/
```

profile 可以引用 `scene`、`world_preset` 或 SDF `file`。启动时会生成临时 world 文件并交给 Gazebo。

`scene` 支持安全参数化：

- `parameters` 声明可覆盖参数和默认值。
- `variants` 只覆盖已声明参数。
- `${param}` 只做值替换，不执行 Python 或 shell。
- `generators` 当前支持受控 `random_boxes`，固定 seed 下生成结果可复现。

示例：

```bash
ros2 run robot_sim_bringup run_case \
  --case industrial_obstacle_clearance \
  --scene-variant dense_obstacles \
  --scene-param seed=31
```

测试：

```bash
colcon test --packages-select robot_sim_scenarios
```
