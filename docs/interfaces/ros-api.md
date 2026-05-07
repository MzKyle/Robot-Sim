# ROS 主题与服务

## 主题

| 名称 | 说明 |
| --- | --- |
| `/image_topic` | 2D 图像预览和采集的数据源 |
| `/tcp_cloud_raw` | 3D 点云数据源 |
| `/fanuc_robot_info` | Fanuc 状态信息 |
| `/data_collect_status` | 采集状态广播 |

## 服务

| 名称 | 说明 |
| --- | --- |
| `/data_collect_activate` | 开始采集 |
| `/data_collect_deactivate` | 停止采集 |
| `/start_fix_scan` | 开始 3D 固定扫描 |
| `/stop_fix_scan` | 停止 3D 固定扫描 |
| `/data_collect_set_task` | 写入任务信息 |

## 调用示例

```bash
ros2 service call /data_collect_activate std_srvs/srv/Empty "{}"
```
