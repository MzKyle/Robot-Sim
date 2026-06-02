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
