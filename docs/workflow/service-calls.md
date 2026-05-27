# 服务调用

## 采集控制

手动开始采集：

```bash
ros2 service call /data_collect_activate std_srvs/srv/Empty "{}"
```

手动停止采集：

```bash
ros2 service call /data_collect_deactivate std_srvs/srv/Empty "{}"
```

## 固定扫描

开始 3D 固定扫描：

```bash
ros2 service call /start_fix_scan std_srvs/srv/Empty "{}"
```

停止 3D 固定扫描：

```bash
ros2 service call /stop_fix_scan std_srvs/srv/Empty "{}"
```

单次扫描：

```bash
ros2 service call /scan_3d weld_interface/srv/Scan3d "{}"
```

重新加载 3D 相机配置：

```bash
ros2 service call /reload_camera_3d_config std_srvs/srv/Trigger "{}"
```

## 设置任务信息

通用任务接口：

```bash
ros2 service call /task/set_context robot_task_interfaces/srv/SetTaskContext \
  "{context: {task_id: T-001, workpiece_id: WP-01, operator_name: zhang, shift: day, notes: test}}"
```

通用采集任务接口：

```bash
ros2 service call /acquisition/set_task acquisition_interfaces/srv/SetAcquisitionTask \
  "{context: {task_id: T-001, workpiece_id: WP-01, operator_name: zhang, shift: day, notes: test}}"
```

旧焊接兼容接口：

```bash
ros2 service call /data_collect_set_task weld_interface/srv/SetCollectionTask \
  "{task_id: T-001, workpiece_id: WP-01, weld_seam_id: S-01, operator_name: zhang, shift: day, notes: test}"
```
