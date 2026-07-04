# 维护者代码地图

`robot_sim_bringup` 现在按职责分为五个内部子包。顶层同名模块仍然保留为兼容 wrapper，外部脚本和旧 import 不需要改；维护新代码时优先导入内部子包。

## 分层

| 子包 | 职责 | 典型模块 |
| --- | --- | --- |
| `common` | Registry、schema 校验、配置迁移等跨 domain 基础能力 | `registry`、`schema_validation`、`migrate_config` |
| `platform` | `schema: 4` 通用 ROS2 pipeline 验证 | `config`、`adapter`、`assertions`、`runner`、`run_suite` |
| `robot_domain` | `schema: 3` 机器人/Gazebo/MoveIt/ros2_control 验收链 | `run_case`、`sim_config_loader`、`sim_launch_builder`、`sim_smoke_helper`、`validation_cases` |
| `legacy_integrations` | 旧焊接/FANUC 外部模块兼容层 | `module_runner`、`module_adapter` |
| `scaffold` | 外部 package 和机器人模板生成 | `assets`、`robot` |

## 依赖方向

推荐依赖方向是：

```text
common
  <- platform
  <- robot_domain
  <- legacy_integrations
  <- scaffold
```

允许 `robot_domain.run_case` 调用 `platform.runner`，因为顶层 `run_case` 需要按 case schema 分发 v3/v4。除此之外，新代码不应从通用 v4 平台反向依赖机器人、焊接或 auto_cover 语义。

## 维护规则

- 新项目接入优先走 `schema: 4` 的 `system_profile`、`data_source`、`adapter_ref`、`validation_case` 和 `suite`。
- 机器人仿真能力留在 `robot_domain`；Panda、Fanuc、industrial cell 只是 `examples/robot_arm` 示例。
- 焊接/FANUC 旧能力留在 `legacy_integrations` 和 `integrations/welding`；不要把新的焊接判断写进 `platform`。
- 顶层 `robot_sim_bringup.platform_config` 等文件只做兼容 re-export，不在 wrapper 中新增业务逻辑。
- 子进程内部调用优先使用新模块路径，例如 `robot_sim_bringup.platform.adapter`。

## 常用入口

| 场景 | 入口 |
| --- | --- |
| 单 case 运行 | `robot_domain.run_case`，自动分发 v3/v4 |
| suite 运行 | `platform.run_suite` |
| v4 case 加载 | `platform.config.load_platform_validation_case` |
| v4 adapter 运行 | `platform.adapter` |
| v3 profile 加载 | `robot_domain.sim_config_loader.load_sim_profile` |
| v3 case 加载 | `robot_domain.validation_cases.load_validation_case` |
| 旧焊接模块验证 | `legacy_integrations.module_runner` |
