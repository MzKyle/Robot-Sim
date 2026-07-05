# robot_sim

[English](README.md) | [简体中文](README.zh-CN.md)

<p align="center">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-00A6A6" />
  <img alt="Gazebo Harmonic" src="https://img.shields.io/badge/Gazebo-Harmonic%20%7C%20gz%20sim%208-F57C00" />
  <img alt="MoveIt2" src="https://img.shields.io/badge/Planning-MoveIt2-2D6CDF" />
  <a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache--2.0-blue" /></a>
</p>

![robot_sim](docs/assets/cover.svg)

`robot_sim` is a ROS 2 robot simulation and acceptance toolkit for checking whether a robot model, controller, MoveIt setup, sensors, scenes, and task workflow can run together reliably.

It is built for users who need a repeatable simulation environment, not just a one-off Gazebo demo. You can run an interactive simulation, execute a validation case, collect run artifacts, or scaffold a new robot simulation package.

Generic `schema: 4` ROS 2 pipeline validation has moved to the sibling project `robot_validation`. This repository now focuses on the `schema: 3` robot simulation domain.

## What You Can Do

| Goal | Use |
| --- | --- |
| Start a robot simulation | Launch Panda or Fanuc profiles in Gazebo with optional MoveIt and sensors |
| Run acceptance checks | Execute validation cases and get logs, metrics, reports, and optional rosbag output |
| Validate an industrial cell | Check obstacle clearance, fixture-to-pallet motion, planning goals, and welding integration dry runs |
| Test sensor workflows | Run camera, depth, lidar, IMU, calibration, and conveyor sorting scenarios |
| Bring your own robot | Generate an external robot package scaffold and add profiles, scenes, and validation cases |

## Requirements

- Ubuntu with ROS 2 Humble sourced from `/opt/ros/humble`
- Gazebo Harmonic / `gz sim 8`
- MoveIt 2 and ros2_control packages for Humble
- `colcon`, `rosdep`, and standard ROS 2 build tooling

See [docs/guide/prerequisites.md](docs/guide/prerequisites.md) for the complete setup checklist.

## Quick Start

```bash
git clone https://github.com/MzKyle/robot_sim.git robot_sim
cd robot_sim

source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic

colcon build --symlink-install \
  --allow-overriding gz_ros2_control \
  --packages-select \
    gz_ros2_control \
    robot_sim_description robot_sim_control robot_sim_scenarios \
    robot_sim_moveit_config \
    robot_sim_sensor_camera robot_sim_sensor_depth robot_sim_sensor_lidar robot_sim_sensor_imu \
    robot_sim_bringup robot_task_interfaces simulation_interfaces

source install/setup.bash
```

Run a fast validation case without Gazebo:

```bash
ros2 run robot_sim_bringup run_case \
  --case empty_motion \
  --mode mock \
  --no-rosbag \
  --output-dir robot_sim_runs \
  --timeout 120
```

The run writes a timestamped directory under `robot_sim_runs/` with the effective config, logs, metrics, and reports.

Start an interactive simulation when you need Gazebo:

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=panda sim_mode:=light
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l_industrial_cell sim_mode:=full
```

Simulation modes:

| Mode | Purpose |
| --- | --- |
| `mock` | Fast runtime and artifact checks without Gazebo |
| `light` | Headless Gazebo and ros2_control, sensors off by default |
| `full` | Gazebo, MoveIt/RViz, bridges, and sensors enabled by default |

## Built-In Examples

Robot profiles:

- `panda`
- `fanuc_m20id12l`
- `fanuc_m20id12l_industrial_cell`

Validation cases:

| Case | What it checks |
| --- | --- |
| `empty_motion` | Minimal MoveIt plan and execute flow |
| `industrial_obstacle_clearance` | Fanuc industrial obstacle avoidance |
| `industrial_fixture_to_pallet` | Fixture-to-pallet industrial motion |
| `industrial_planning_goal` | Industrial planning target smoke check |
| `panda_pick_place` | Panda tabletop pick-place planning |
| `sensor_calibration` | Multi-view sensor calibration workflow |
| `conveyor_sorting` | Conveyor sorting workflow |
| `weld_pre_positioning_scan_and_move` | Pre-weld 3D localization plus MoveIt jog |
| `weld_2d_lateral_correction_dry_run` | 2D weld correction dry run |

## Bring Your Own Robot

Use the scaffold command to create an external ROS package with the expected `robot_sim/` layout:

```bash
ros2 run robot_sim_bringup scaffold_robot \
  --package my_robot_sim \
  --robot-name my_robot \
  --output /tmp \
  --planning-group manipulator \
  --tool-link tool0 \
  --joint-names joint_1 joint_2 joint_3 joint_4 joint_5 joint_6 \
  --sensor-set camera,depth,lidar,imu \
  --with-gripper true
```

External packages are discovered from:

```text
share/<pkg>/robot_sim/profiles/*.yaml
share/<pkg>/robot_sim/validation_cases/*.yaml
share/<pkg>/robot_sim/scenes/*.yaml
```

## Documentation

- User setup and run guides: [docs/guide/quick-start.md](docs/guide/quick-start.md)
- Developer system overview: [docs/README.md](docs/README.md)
- Configuration reference: [docs/configuration/settings.md](docs/configuration/settings.md)
- Architecture notes: [docs/architecture/README.md](docs/architecture/README.md)
- Troubleshooting: [docs/faq/troubleshooting.md](docs/faq/troubleshooting.md)

## Debian Package

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
bash packaging/build_deb.sh
sudo apt install ./dist/robot-sim_0.1.0-1_amd64.deb
```

Installed command examples:

```bash
robot-sim-check
robot-sim run-case --case industrial_fixture_to_pallet
robot-sim migrate-config --input old.yaml --output new.yaml
robot-sim scaffold-robot --package my_robot_sim --robot-name my_robot --output /tmp --joint-names joint_1 joint_2 joint_3 joint_4 joint_5 joint_6
robot-sim sim_profile:=panda sim_mode:=light
```

## License

`robot_sim` is licensed under the [Apache License 2.0](LICENSE).
