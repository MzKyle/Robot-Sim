# Bringup 入口

`data_collect_bringup` 负责统一启动真实采集相关节点，并把默认配置文件传递给各个模块。

## 主要职责

- 组织 launch 流程。
- 注入 `nodemanage.yaml`。
- 作为真实采集栈的统一入口，默认启动相机、Fanuc、采集核心和质量节点。

## 常见用途

```bash
ros2 launch data_collect_bringup data_collect.launch.py
```

## 参数覆盖

可以按需覆盖配置文件、机器人地址和共享库路径。

## 与仿真入口的区别

- `data_collect_bringup` 面向真实硬件。
- `data_collect_sim` 面向 gz sim 8、Panda 机械臂和 mock 链路。
