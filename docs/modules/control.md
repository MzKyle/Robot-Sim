# 控制配置

`robot_sim_control` 保存 controller yaml。

目录约定：

```text
config/robots/panda/controllers.yaml
config/robots/fanuc_m20id12l/controllers.yaml
```

profile 中通过 `control.controllers_file` 引用 controller 文件。smoke test 会检查：

- controller spawner 是否声明。
- 必需 controller 是否 `active`。
- 主轨迹 controller 的 action 是否可执行。
- controller joints 是否与 MoveIt controller 配置一致。

启动时，所有通过 feature gate 启用的 controller 由一个 spawner 进程加载和配置，
最后通过一次 `--activate-as-group` 切换为 active。controller YAML 必须为每个 spawner
名称提供 `type`；profile 中的 `type` 用于 lint 和轨迹 controller 识别。profile 的
`timeout` 最大值会显式传给 manager、service 和 switch timeout。

不要为同一个 controller manager 再并行启动额外 spawner。CI 负载较高时，并发
`switch_controller` 容易让 controller 留在 `inactive`。
