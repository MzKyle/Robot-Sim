# ROS 主题与服务

## 主题

| 名称 | 说明 |
| --- | --- |
| `/acquisition/status` | 通用采集状态广播 |
| `/acquisition/quality` | 通用采集质量评估结果 |
| `/image_topic` | 2D 图像预览和采集的数据源 |
| `/tcp_cloud_raw` | 3D 点云数据源 |
| `/tool_pos` | 机器人或仿真 TCP 位姿 |
| `/fanuc_robot_info` | Fanuc 状态信息 |
| `/fanuc_target_register_value` | 目标寄存器当前值 |
| `/fanuc_weld_register_info` | 焊接寄存器信息 |
| `/data_collect_status` | 旧焊接采集状态广播 |
| `/data_collect_quality` | 旧焊接采集质量评估结果 |
| `/fixed_scan` | 固定扫描点云 |
| `/fixed_scan_all` | 完整固定扫描点云 |
| `/scan_pose` | 扫描时的 TCP 位姿 |
| `/debug_height_img` | 3D 调试高度图像 |

## 服务

| 名称 | 说明 |
| --- | --- |
| `/data_collect_activate` | 开始采集 |
| `/data_collect_deactivate` | 停止采集 |
| `/task/set_context` | 写入通用任务上下文 |
| `/acquisition/set_task` | 写入通用采集任务 |
| `/start_fix_scan` | 开始 3D 固定扫描 |
| `/stop_fix_scan` | 停止 3D 固定扫描 |
| `/scan_3d` | 3D 单次扫描 |
| `/reload_camera_3d_config` | 重新加载 3D 配置 |
| `/data_collect_set_task` | 写入旧焊接任务信息 |
| `/fanuc_register_read` | 读取仿真或真实寄存器 |

## 调用示例

```bash
ros2 service call /data_collect_activate std_srvs/srv/Empty "{}"
```
