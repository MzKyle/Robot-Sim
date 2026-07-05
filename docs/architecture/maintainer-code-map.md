# 维护者代码地图

`robot_sim_bringup` 现在按 v3 机器人仿真职责分层。通用 v4 验证代码已移动到同级
`robot_validation` 项目。

## 分层

| 子包 | 职责 | 典型模块 |
| --- | --- | --- |
| `common` | Registry、schema 校验、v2->v3 配置迁移 | `registry`、`schema_validation`、`migrate_config` |
| `robot_domain` | `schema: 3` 机器人/Gazebo/MoveIt/ros2_control 验收链 | `run_case`、`sim_config_loader`、`sim_launch_builder`、`sim_smoke_helper`、`validation_cases` |
| `legacy_integrations` | 旧焊接/FANUC 外部模块兼容层 | `module_runner`、`module_adapter` |
| `scaffold` | 外部机器人模板生成 | `robot` |

## 依赖方向

```text
common
  <- robot_domain
  <- legacy_integrations
  <- scaffold
```

## 维护规则

- 新机器人通过 profile、URDF/xacro、controller、MoveIt 配置和 v3 validation case 接入。
- 复杂通用 ROS2 topic/service/TF/process 验证不要加回本仓库，放到 `robot_validation`。
- 顶层 `robot_sim_bringup.*` 文件只做兼容 re-export，不新增业务逻辑。

## 常用入口

| 场景 | 入口 |
| --- | --- |
| 单 case 运行 | `robot_domain.run_case` |
| v3 profile 加载 | `robot_domain.sim_config_loader.load_sim_profile` |
| v3 case 加载 | `robot_domain.validation_cases.load_validation_case` |
| 旧焊接模块验证 | `legacy_integrations.module_runner` |
