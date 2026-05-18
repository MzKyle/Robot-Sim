# 配置总览

默认配置文件位于：

```text
src/config/nodemanage.yaml
```

真实设备链路启动时，`data_collect.launch.py` 会把这个 YAML 传给 2D 相机、3D 相机、Fanuc 节点和采集质量节点；`data_collect_node` 也会通过 `AUTOCOVER_NODEMANAGE_YAML` 读取同一个文件。界面中的 `参数设置` 页会直接修改这个 YAML。

仿真链路使用 `robot_sim_bringup` 的 launch 参数和 `robot_sim_control/config/panda_controllers.yaml`。旧 `data_collect_sim/config/nodemanage_sim.yaml` 已移除。

## 常用配置段

```yaml
robot_driver_fanuc:
  ros__parameters:
    so_file_path: lib/libFanucRobot.so
    robot_ip: 10.16.140.114
    robot_port: 60008
    target_register_index: 100

camera_driver_3d:
  ros__parameters:
    cfg: config/cameratcp.yaml
    publish_tf: true

data_collect_node:
  ros__parameters:
    save_dir_root: data
    image_save_interval: 5
    image_log_save_interval: 3
    height_log_save_interval: 4
    fix_scan_interval: 6
    auto_save_flag: 0
    target_register_index: 100

camera_node:
  ros__parameters:
    trigger_mode: 0
    strobe_polarity: 1
    saturation: 57
    gamma: 106
    exposure_time: 4.3
    analog_gain: 63
    frame_rate: 59.9
```

## 参数含义

- `save_dir_root`：采集数据保存根目录。
- `image_save_interval`：2D 图像保存间隔。
- `image_log_save_interval`：图像日志保存间隔。
- `height_log_save_interval`：高度日志图像保存间隔。
- `fix_scan_interval`：固定扫描点云保存间隔。
- `auto_save_flag`：是否根据 Fanuc 焊接检测信号自动启停采集，`0` 为手动，非 `0` 为自动。
- `target_register_index`：用于区分数据类别或工件类别的 Fanuc 目标寄存器编号。
- `camera_driver_3d.cfg`：3D 相机参数文件，相对路径会按包 share 目录解析。
- `camera_driver_3d.publish_tf`：是否把相机位姿发布到 TF。
- `camera_node.trigger_mode`：2D 相机触发模式。
- `camera_node.strobe_polarity`：2D 相机频闪极性。
- `camera_node.saturation`：2D 相机饱和度。
- `camera_node.gamma`：2D 相机 Gamma。
- `camera_node.exposure_time`：2D 相机曝光时间。
- `camera_node.analog_gain`：2D 相机模拟增益。
- `camera_node.frame_rate`：2D 图像发布频率。

## 仿真配置

`robot_sim_bringup` 的关键参数位于 launch 命令中：

- `sim_mode`：`mock`、`light` 或 `full`，默认 `light`。
- `enable_camera`、`enable_depth`、`enable_lidar`、`enable_imu`：按组启停传感器。
- `use_moveit`、`rviz`、`headless`、`use_sim_time`：支持 `auto|true|false`。
- `robot_sim_control/config/panda_controllers.yaml`：Panda 控制器配置。
