# 测试验收

## 单元测试

```bash
colcon test --packages-select robot_sim_bringup robot_sim_scenarios
colcon test-result --verbose
```

## Profile lint

```bash
ros2 run robot_sim_bringup profile_lint --profile panda --mode full --require-moveit --require-receivers
ros2 run robot_sim_bringup profile_lint --profile fanuc_m20id12l --mode full --require-moveit --require-receivers
ros2 run robot_sim_bringup profile_lint --profile fanuc_m20id12l_industrial_cell --mode full --require-moveit --require-receivers
```

## Smoke test

快速 mock：

```bash
scripts/sim_smoke_test.sh --profile panda --mode mock --timeout 60
```

完整 Gazebo：

```bash
scripts/sim_smoke_test.sh --profile panda --mode full --timeout 120
scripts/sim_smoke_test.sh --profile fanuc_m20id12l --mode full --with-moveit --timeout 120
```

## Validation Case

验收用例会启动仿真、执行 MoveIt 任务、检查控制链/TF/传感器/碰撞指标，并生成结构化产物：

```bash
ros2 run robot_sim_bringup run_case \
  --case industrial_fixture_to_pallet \
  --output-dir robot_sim_runs \
  --timeout 120

ros2 run robot_sim_bringup run_case \
  --case industrial_obstacle_clearance \
  --output-dir robot_sim_runs \
  --timeout 120
```

每次运行会创建：

```text
robot_sim_runs/<timestamp>_<case>_<profile>/
  manifest.json
  metrics.json
  report.md
  report.html
  robot.urdf
  logs/
  rosbag/
```

失败排查时保留日志：

```bash
scripts/sim_smoke_test.sh --profile fanuc_m20id12l --mode full --with-moveit --keep-logs
```

## Gazebo plugin 检查

```bash
gz plugin -p "$(ros2 pkg prefix gz_ros2_control)/lib/libgz_ros2_control-system.so" --info
```

输出应包含：

```text
gz_ros2_control::GazeboSimROS2ControlPlugin
```
