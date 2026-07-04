# robot_sim

[English](README.md) | [简体中文](README.zh-CN.md)

<p align="center">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-00A6A6" />
  <img alt="Gazebo Harmonic" src="https://img.shields.io/badge/Gazebo-Harmonic%20%7C%20gz%20sim%208-F57C00" />
  <img alt="MoveIt2" src="https://img.shields.io/badge/Planning-MoveIt2-2D6CDF" />
  <a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache--2.0-blue" /></a>
</p>

![robot_sim](docs/assets/cover.svg)

`robot_sim` is a ROS 2 simulation and interface validation platform. It keeps the
industrial robot/Gazebo/MoveIt/ros2_control acceptance chain as the `robot` domain, and
adds a schema v4 platform runner for generic ROS2 topic/service/TF/process contract
checks with system profiles, data sources, adapters, assertions, and suites.

[Read the documentation](docs/README.md)

## Quick Start

1. Install Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic, MoveIt2, `colcon`, and `rosdep`.
   See [Prerequisites](docs/guide/prerequisites.md) for the full dependency list.
2. Clone and build the workspace:

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

3. Run the first validation case:

```bash
ros2 run robot_sim_bringup run_case \
  --case empty_motion \
  --output-dir robot_sim_runs \
  --timeout 120
```

4. Open the generated report:

```bash
latest_run="$(ls -td robot_sim_runs/*_empty_motion_panda | head -1)"
xdg-open "${latest_run}/report.html"
```

Each run creates a standalone artifact directory with `manifest.json`, effective YAML
configs, `robot.urdf`, logs, rosbag, `metrics.json`, `report.md`, and `report.html`.

## What You Can Do

- Start Panda or Fanuc M-20iD/12L simulations in `mock`, `light`, or `full` mode.
- Validate controller state, joint states, TF completeness, sensor topic frequency,
  MoveIt planning/execution, goal error, controller error, and TCP clearance.
- Run reusable validation cases for empty motion, obstacle clearance, fixture-to-pallet,
  pick-place, sensor calibration, conveyor sorting, and external module validation.
- Use scene variants and parameters to create deterministic industrial test conditions.
- Replay or stub generic ROS2 topics/services with schema v4 data sources and adapters.
- Keep legacy welding/FANUC integrations available without making them the core platform model.
- Generate run artifacts that can be reviewed manually, archived for delivery, or checked
  by CI.
- Scaffold an external robot simulation package instead of putting every robot in this
  repository.

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
| `weld_pre_positioning_scan_and_move` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | External pre-weld 3D localization with dataset `/scan_3d` and MoveIt jog |
| `weld_2d_lateral_correction_dry_run` | `fanuc_m20id12l_industrial_cell` | `industrial_cell` | External 2D weld correction dry run with synthetic vision |

## Simulation and Validation Workflow

- `sim.launch.py` starts an interactive simulation:

```bash
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=panda sim_mode:=light
ros2 launch robot_sim_bringup sim.launch.py sim_profile:=fanuc_m20id12l_industrial_cell sim_mode:=full
```

- `run_case` starts the full acceptance workflow:

```bash
ros2 run robot_sim_bringup run_case --case industrial_fixture_to_pallet --output-dir robot_sim_runs --timeout 120
ros2 run robot_sim_bringup run_case --case industrial_obstacle_clearance --output-dir robot_sim_runs --timeout 120
```

- External module validation requires sourcing the external workspace first:

```bash
source /home/kyle/sany/ROS2_Motion_Planner/install/setup.bash
ros2 run robot_sim_bringup run_case --case weld_pre_positioning_scan_and_move --output-dir robot_sim_runs --timeout 180
```

The pre-weld case uses `/home/kyle/sany/data/3dcamera_2d_img` when available. If matching
`.npz` point clouds exist, they are replayed with the real images. If only images exist,
`robot_sim` generates a deterministic synthetic point cloud. If no dataset frame is
available, it falls back to the packaged replay capture.

## Configuration Model

`robot_sim` supports two YAML contract families validated by JSON Schema:

| Config kind | What it describes |
| --- | --- |
| `schema: 3 sim_profile` | Robot description, control, MoveIt, sensors, bridges, worlds, layouts, capabilities |
| `scene` | A full workcell with regions, objects, workspaces, parameters, variants, and generators |
| `world_preset` | Legacy/base world asset composition |
| `schema: 3 validation_case` | Launch settings, scene, task family, planning scene, expectations, adapters, artifacts |
| `schema: 4 system/data_source/adapter/suite` | Generic ROS2 pipeline validation assets |

External packages are discovered from:

```text
share/<pkg>/robot_sim/profiles/*.yaml
share/<pkg>/robot_sim/validation_cases/*.yaml
share/<pkg>/robot_sim/suites/*.yaml
share/<pkg>/robot_sim/data_sources/*.yaml
share/<pkg>/robot_sim/adapters/*.yaml
```

The legacy `robot_sim/validation_suites` path is still accepted. Built-in
robot examples live under `examples/robot_arm`; welding assets live under
`integrations/welding`; RM vision interface examples live under `examples/rm_vision`.

## Robot Scaffold

Generate a reusable external robot package skeleton:

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

Build and install the local Debian package:

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
robot-sim scaffold-robot --package my_robot_sim --robot-name my_robot --output /tmp --joint-names joint_1 joint_2 joint_3 joint_4 joint_5 joint_6
robot-sim scaffold-system --package my_robot_sim --name minimal_system --output /tmp
robot-sim scaffold-case --package my_robot_sim --name smoke_case --system minimal_system --output /tmp
robot-sim scaffold-suite --package my_robot_sim --name smoke_suite --case smoke_case --output /tmp
robot-sim scaffold-adapter --package my_robot_sim --name smoke_adapter --output /tmp
robot-sim sim_profile:=panda sim_mode:=light
```

## Documentation

- [Documentation home](docs/README.md)
- [Quick start](docs/guide/quick-start.md)
- [Simulation guide](docs/guide/simulation.md)
- [External module integration](docs/guide/external-modules.md)
- [External project assets](docs/guide/external-projects.md)
- [Maintainer code map](docs/architecture/maintainer-code-map.md)
- [Testing and validation](docs/workflow/testing.md)
- [Configuration reference](docs/configuration/settings.md)
- [Run artifacts and logs](docs/logging/data-storage.md)
- [Packaging](docs/guide/package-install.md)
- [Roadmap](docs/roadmap.md)
- [Troubleshooting](docs/faq/troubleshooting.md)

## License

`robot_sim` is licensed under the [Apache License 2.0](LICENSE). See
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for third-party notices.
