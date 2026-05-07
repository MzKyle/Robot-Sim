# 配置总览

默认配置文件位于：

```text
src/config/nodemanage.yaml
```

启动时 `data_collect.launch.py` 会把这个 YAML 传给 2D 相机、3D 相机和 Fanuc 节点；`data_collect_node` 也会通过 `AUTOCOVER_NODEMANAGE_YAML` 读取同一个文件。界面中的 `参数设置` 页会直接修改这个 YAML。

## 常用配置段

```yaml
robot_driver_fanuc:
  ros__parameters:
    so_file_path: /home/kyle/sany/weld_data_collect_ws/src/fanuc_robot/lib/libFanucRobot.so
    robot_ip: 10.16.140.114
    robot_port: 60008
    target_register_index: 100

data_collect_node:
  ros__parameters:
    save_dir_root: /home/kyle/sany/weld_data_collect_ws/data
    image_save_interval: 12
    image_log_save_interval: 3
    height_log_save_interval: 4
    fix_scan_interval: 6
    auto_save_flag: 0
    target_register_index: 100

camera_node:
  ros__parameters:
    trigger_mode: 2
    strobe_polarity: 0
    saturation: 64
    gamma: 106
    exposure_time: 4.3
    analog_gain: 64
    frame_rate: 60.0
```

## 参数含义

- `save_dir_root`：采集数据保存根目录。
- `image_save_interval`：2D 图像保存间隔。
- `image_log_save_interval`：图像日志保存间隔。
- `height_log_save_interval`：高度日志图像保存间隔。
- `fix_scan_interval`：固定扫描点云保存间隔。
- `auto_save_flag`：是否根据 Fanuc 焊接检测信号自动启停采集，`0` 为手动，非 `0` 为自动。
- `target_register_index`：用于区分数据类别或工件类别的 Fanuc 目标寄存器编号。
- `camera_node.trigger_mode`：2D 相机触发模式。
- `camera_node.strobe_polarity`：2D 相机频闪极性。
- `camera_node.saturation`：2D 相机饱和度。
- `camera_node.gamma`：2D 相机 Gamma。
- `camera_node.exposure_time`：2D 相机曝光时间。
- `camera_node.analog_gain`：2D 相机模拟增益。
- `camera_node.frame_rate`：2D 图像发布频率。
