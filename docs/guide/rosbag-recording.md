# ROS 2 录包辅助

`robot_sim_bringup` 提供 `record_bag.launch.py`，用于快速录制 Gazebo 仿真、运控和传感器相关 topic。它只是辅助入口，不会自动绑定到 `sim.launch.py`，因此需要在另一个终端单独启动。

## 基本用法

先启动仿真：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_mode:=full
```

另开终端启动录包：

```bash
cd /home/kyle/sany/robot_sim
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch robot_sim_bringup record_bag.launch.py topic_group:=all
```

默认输出目录：

```text
~/robot_sim_bags/robot_sim_<topic_group>_<timestamp>
```

## 话题组

| `topic_group` | 说明 |
| --- | --- |
| `control` | `/clock`、TF、`/joint_states`、arm/gripper controller 状态和轨迹 topic |
| `sensors` | `/clock`、TF、RGB、深度、点云、LaserScan、lidar 点云和 IMU |
| `all` | 单机仿真的控制和传感器 topic |
| `distributed` | `distributed_local.launch.py` 下的 `/robot`、`/sensors` 命名空间 topic |
| `custom` | 不使用预设，只录制 `extra_topics` 指定的 topic |

## 常用示例

只录制运控链路：

```bash
ros2 launch robot_sim_bringup record_bag.launch.py topic_group:=control
```

录制传感器并启用 zstd 压缩：

```bash
ros2 launch robot_sim_bringup record_bag.launch.py \
  topic_group:=sensors \
  compression:=true \
  bag_name:=sensor_debug
```

录制本机分布式仿真：

```bash
ros2 launch robot_sim_bringup distributed_local.launch.py sim_mode:=full
ros2 launch robot_sim_bringup record_bag.launch.py topic_group:=distributed
```

自定义 topic：

```bash
ros2 launch robot_sim_bringup record_bag.launch.py \
  topic_group:=custom \
  extra_topics:="/joint_states /camera/points /tf /tf_static"
```

按大小切分：

```bash
ros2 launch robot_sim_bringup record_bag.launch.py \
  topic_group:=all \
  max_bag_size:=2147483648
```

## Launch 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `topic_group` | `all` | 预设话题组：`control`、`sensors`、`all`、`distributed`、`custom` |
| `extra_topics` | 空 | 追加 topic，支持空格或逗号分隔 |
| `output_dir` | `~/robot_sim_bags` | 相对 bag 名称的输出目录 |
| `bag_name` | `auto` | bag 目录名，`auto` 时自动加时间戳 |
| `storage_id` | `sqlite3` | rosbag2 storage 插件 |
| `include_action_topics` | `true` | 录制 FollowJointTrajectory feedback/status topic |
| `include_hidden_topics` | `false` | 手动打开 rosbag2 隐藏 topic 开关；录制 action topic 时会自动打开 |
| `compression` | `false` | 是否启用压缩 |
| `compression_mode` | `file` | rosbag2 压缩模式 |
| `compression_format` | `zstd` | rosbag2 压缩格式 |
| `max_bag_size` | `0` | 单包最大字节数，`0` 表示不按大小切分 |
| `max_bag_duration` | `0` | 单包最大秒数，`0` 表示不按时间切分 |

## 查看与回放

```bash
ros2 bag info ~/robot_sim_bags/<bag_name>
ros2 bag play ~/robot_sim_bags/<bag_name> --clock
```

回放给 RViz2 或下游算法使用时，记得让消费端启用 `use_sim_time:=true`。
