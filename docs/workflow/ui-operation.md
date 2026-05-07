# UI 操作

## 启动界面

```bash
cd /home/kyle/sany/weld_data_collect_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run data_collect_ui data_collect_ui
```

## 页面说明

- `采集操作`：查看状态、填写任务信息并控制采集。
- `实时预览`：查看 2D 图像和当前保存信息。
- `历史数据`：扫描 `manifest.json`，浏览历史记录。
- `参数设置`：修改 `nodemanage.yaml` 并自动保存。

## 常用按钮

- `保存任务信息`
- `启动采集`
- `停止采集`
- `开始3D扫描`
- `停止3D扫描`
- `打开保存目录`
