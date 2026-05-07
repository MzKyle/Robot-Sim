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

## 设置任务信息

```bash
ros2 service call /data_collect_set_task weld_interface/srv/SetCollectionTask \
  "{task_id: T-001, workpiece_id: WP-01, weld_seam_id: S-01, operator_name: zhang, shift: day, notes: test}"
```
