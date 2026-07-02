# Bringup

`robot_sim_bringup` 是仿真入口包。

主要能力：

- `sim.launch.py`：按 profile 与 mode 启动仿真。
- `sensor_receivers.launch.py`：按 profile 启动 receiver。
- `record_bag.launch.py`：录制 rosbag。
- `distributed_local.launch.py`：本机分布式模拟。
- `profile_lint`：检查 profile 一致性。
- `sim_smoke_helper`：为 smoke test 提供等待、控制、MoveIt 和 rosbag 检查。
- `run_case`：执行 validation case 并生成 manifest、metrics、report、日志和 rosbag。

常用命令：

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=panda sim_mode:=light
ros2 run robot_sim_bringup profile_lint --profile panda --mode full --require-moveit
ros2 run robot_sim_bringup run_case --case industrial_fixture_to_pallet
```
