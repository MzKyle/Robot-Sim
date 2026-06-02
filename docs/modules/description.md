# 机器人描述

`robot_sim_description` 保存机器人 xacro、mesh、传感器挂载和 ros2_control 标签。

当前机器人：

- Panda
- Fanuc M-20iD/12L

目录约定：

```text
robots/<robot>/<robot>.urdf.xacro
models/robots/<robot>/
models/sensors/
macros/
```

新增机器人时应支持通用 xacro 参数：

```text
hardware_plugin
controllers_file
controller_manager_name
use_gz_ros2_control
ros_namespace
```
