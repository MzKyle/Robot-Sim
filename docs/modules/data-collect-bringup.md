# Bringup 入口

`data_collect_bringup` 负责统一启动采集相关节点，并把默认配置文件传递给各个模块。

## 主要职责

- 组织 launch 流程。
- 注入 `nodemanage.yaml`。
- 作为完整采集栈的统一入口。

## 常见用途

```bash
ros2 launch data_collect_bringup data_collect.launch.py
```

## 参数覆盖

可以按需覆盖配置文件、机器人地址和共享库路径。
