# CI/CD

本项目使用 GitHub Actions 做持续集成、仿真 smoke、文档发布和 deb release。

## CI

`ci.yml` 在 pull request 和 `main` push 时运行：

- checkout submodule。
- 安装 ROS 2 Humble、Gazebo Harmonic、MoveIt2 和 ros_gzharmonic。
- 构建 12 个包。
- 运行 `robot_sim_bringup` 与 `robot_sim_scenarios` 测试。
- 执行 Panda/Fanuc profile lint。
- 执行 `panda mock` smoke test。

## Full Smoke

`simulation-smoke.yml` 可手动运行，也会每周定时运行：

```bash
scripts/sim_smoke_test.sh --profile panda --mode full --timeout 120
scripts/sim_smoke_test.sh --profile fanuc_m20id12l --mode full --with-moveit --timeout 120
ros2 run robot_sim_bringup run_case --case industrial_fixture_to_pallet --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case industrial_obstacle_clearance --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case empty_motion --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case panda_pick_place --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case sensor_calibration --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case conveyor_sorting --output-dir robot_sim_runs --timeout 120
```

失败时会上传 colcon、smoke 日志和 `robot_sim_runs/` 验收产物。

## Docs

`docs.yml` 在 `main` 更新 `docs/` 后部署 GitHub Pages。站点内容直接来自 `docs/`，由 docsify 在浏览器端渲染。

## Release

`release.yml` 在 `vMAJOR.MINOR.PATCH` tag 上运行：

- 调用 `packaging/build_deb.sh`。
- 创建或更新 GitHub Release。
- 上传 `dist/*.deb`。
