# ROS 2 录包

`robot_sim_bringup` 提供 `record_bag.launch.py`，用于按预设话题组录制仿真数据。

## 启动仿真

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=panda sim_mode:=full
```

## 录制

```bash
ros2 launch robot_sim_bringup record_bag.launch.py \
  topic_group:=all \
  output_dir:=robot_sim_bags \
  bag_name:=panda_full
```

常用参数：

| 参数 | 说明 |
| --- | --- |
| `topic_group` | 话题组，常用 `all` |
| `output_dir` | rosbag 输出目录 |
| `bag_name` | bag 名称 |
| `compression` | 是否启用压缩 |

smoke test 可自动做短录包检查：

```bash
scripts/sim_smoke_test.sh --profile panda --mode full --with-rosbag --keep-logs
```
