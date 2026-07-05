# robot_sim

[English](README.md) | [简体中文](README.zh-CN.md)

<p align="center">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-00A6A6" />
  <img alt="Gazebo Harmonic" src="https://img.shields.io/badge/Gazebo-Harmonic%20%7C%20gz%20sim%208-F57C00" />
  <img alt="MoveIt2" src="https://img.shields.io/badge/Planning-MoveIt2-2D6CDF" />
  <a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache--2.0-blue" /></a>
</p>

![robot_sim](docs/assets/cover.svg)

`robot_sim` is now focused on the `schema: 3` robot simulation and acceptance chain:
Gazebo, MoveIt, ros2_control, robot profiles, scenes, validation cases, sensors, and
legacy welding/FANUC integrations.

Generic `schema: 4` ROS2 pipeline validation has moved to the sibling project
`../robot_validation`.

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

Run a v3 validation case:

```bash
ros2 run robot_sim_bringup run_case \
  --case empty_motion \
  --output-dir robot_sim_runs \
  --timeout 120
```

Run an interactive simulation:

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=panda sim_mode:=light
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l_industrial_cell sim_mode:=full
```

## Built-in Validation Cases

| Case | Profile | Scene | Purpose |
| --- | --- | --- | --- |
| `empty_motion` | `panda` | `debug_empty` | Minimal MoveIt plan and execute validation |
| `industrial_obstacle_clearance` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | Fanuc obstacle avoidance with planning-scene objects |
| `industrial_fixture_to_pallet` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | Fixture-to-pallet industrial motion validation |
| `industrial_planning_goal` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | Industrial planning target smoke validation |
| `panda_pick_place` | `panda` | `tabletop_pick_place` | Pick-place planning validation, execution disabled by default |
| `sensor_calibration` | `panda` | `tabletop_pick_place` | Multi-view sensor calibration workflow validation |
| `conveyor_sorting` | `panda` | `conveyor_sorting` | Conveyor sorting workflow validation |
| `weld_pre_positioning_scan_and_move` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | External pre-weld 3D localization with MoveIt jog |
| `weld_2d_lateral_correction_dry_run` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | External 2D weld correction dry run |

## Configuration Model

`robot_sim` validates these v3 YAML contracts:

| Config kind | What it describes |
| --- | --- |
| `sim_profile` | Robot description, control, MoveIt, sensors, bridges, worlds, layouts, capabilities |
| `scene` | A workcell with regions, objects, workspaces, parameters, variants, and generators |
| `world_preset` | Legacy/base world asset composition |
| `validation_case` | Launch settings, scene, task family, planning scene, expectations, adapters, artifacts |

External robot packages are discovered from:

```text
share/<pkg>/robot_sim/profiles/*.yaml
share/<pkg>/robot_sim/validation_cases/*.yaml
share/<pkg>/robot_sim/scenes/*.yaml
```

If a `schema: 4` validation case is passed to `robot_sim_bringup run_case`, the command
fails with a migration hint to run it with `robot_validation`.

## Robot Scaffold

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

## Debian Package

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
bash packaging/build_deb.sh
sudo apt install ./dist/robot-sim_0.1.0-1_amd64.deb
```

Installed commands:

```bash
robot-sim-check
robot-sim run-case --case industrial_fixture_to_pallet
robot-sim migrate-config --input old.yaml --output new.yaml
robot-sim scaffold-robot --package my_robot_sim --robot-name my_robot --output /tmp --joint-names joint_1 joint_2 joint_3 joint_4 joint_5 joint_6
robot-sim sim_profile:=panda sim_mode:=light
robot-sim sim_profile:=fanuc_m20id12l sim_mode:=full
```

## License

`robot_sim` is licensed under the [Apache License 2.0](LICENSE).
