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
