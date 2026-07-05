from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

import yaml


def build_parser():
    parser = argparse.ArgumentParser(description="Scaffold an external robot_sim robot package.")
    parser.add_argument("--package", required=True, help="ROS package name to generate.")
    parser.add_argument("--robot-name", required=True, help="Robot/profile name.")
    parser.add_argument("--output", required=True, help="Directory where the package directory will be created.")
    parser.add_argument("--planning-group", default="manipulator")
    parser.add_argument("--tool-link", default="tool0")
    parser.add_argument("--joint-names", nargs="+", required=True)
    parser.add_argument("--sensor-set", default="camera,depth,lidar,imu")
    parser.add_argument("--with-gripper", default="true", choices=("true", "false"))
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        package_dir = scaffold_robot(args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(f"Generated robot_sim robot package: {package_dir}")
    return 0


def scaffold_robot(args) -> Path:
    package_name = _safe_identifier(args.package, "package")
    robot_name = _safe_identifier(args.robot_name, "robot-name")
    output_dir = Path(args.output).expanduser().resolve()
    package_dir = output_dir / package_name
    if package_dir.exists() and any(package_dir.iterdir()):
        raise RuntimeError(f"output package directory already exists and is not empty: {package_dir}")

    sensor_names = _split_csv(args.sensor_set)
    with_gripper = args.with_gripper == "true"
    joint_names = [str(joint) for joint in args.joint_names]

    files = {
        "package.xml": _package_xml(package_name),
        "CMakeLists.txt": _cmake(package_name),
        f"description/robots/{robot_name}.urdf.xacro": _urdf_xacro(robot_name, joint_names, args.tool_link, with_gripper),
        "control/controllers.yaml": _controllers_yaml(joint_names, with_gripper),
        f"moveit_config/config/{robot_name}.srdf": _srdf(robot_name, args.planning_group, args.tool_link, joint_names, with_gripper),
        "moveit_config/config/kinematics.yaml": _kinematics_yaml(args.planning_group),
        "moveit_config/config/joint_limits.yaml": _joint_limits_yaml(joint_names),
        "moveit_config/config/moveit_controllers.yaml": _moveit_controllers_yaml(joint_names, with_gripper),
        "moveit_config/config/ompl_planning.yaml": _ompl_yaml(args.planning_group),
        "moveit_config/rviz/robot.rviz": _rviz(args.planning_group),
        f"robot_sim/profiles/{robot_name}.yaml": yaml.safe_dump(
            _profile(package_name, robot_name, args.planning_group, args.tool_link, sensor_names, with_gripper),
            sort_keys=False,
        ),
        "robot_sim/scenes/smoke_scene.yaml": yaml.safe_dump(_scene(), sort_keys=False),
        "robot_sim/validation_cases/smoke_empty_motion.yaml": yaml.safe_dump(
            _validation_case(package_name, robot_name, args.planning_group, args.tool_link),
            sort_keys=False,
        ),
    }

    for relative_path, content in files.items():
        path = package_dir / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    return package_dir


def _profile(package_name, robot_name, planning_group, tool_link, sensor_names, with_gripper):
    bridges = ["clock"]
    bridge_groups = {
        "clock": {
            "node_name": "clock_bridge",
            "namespace": "sensors",
            "topics": [{
                "ros_topic_name": "/clock",
                "gz_topic_name": "/clock",
                "ros_type_name": "rosgraph_msgs/msg/Clock",
                "gz_type_name": "gz.msgs.Clock",
                "direction": "GZ_TO_ROS",
            }],
        }
    }
    sensors = {}
    for sensor_name in sensor_names:
        sensors[sensor_name] = {
            "xacro_arg": f"enable_{sensor_name}",
            "default_enabled": True,
            "bridge_group": sensor_name if sensor_name != "imu" else "imu",
            "static_tfs": [],
        }
        bridge_groups.setdefault(sensor_name, {
            "node_name": f"{sensor_name}_bridge",
            "namespace": "sensors",
            "topics": _bridge_topics(sensor_name),
        })
    spawners = [
        {"name": "joint_state_broadcaster", "type": "joint_state_broadcaster/JointStateBroadcaster", "timeout": 90},
        {"name": "arm_controller", "type": "joint_trajectory_controller/JointTrajectoryController", "timeout": 90},
    ]
    if with_gripper:
        spawners.append({
            "name": "gripper_controller",
            "type": "joint_trajectory_controller/JointTrajectoryController",
            "timeout": 90,
            "enabled_by": "use_gripper",
        })

    end_effector = {
        "planning_group": planning_group,
        "tool_link": tool_link,
        "base_frame": "world",
    }
    if with_gripper:
        end_effector["gripper"] = {
            "controller": "gripper_controller",
            "open_positions": [0.04, 0.04],
            "closed_positions": [0.0, 0.0],
        }

    return {
        "schema": 3,
        "kind": "sim_profile",
        "name": robot_name,
        "metadata": {
            "package": package_name,
            "robot_name": robot_name,
            "vendor": "custom",
            "model": robot_name,
        },
        "capabilities": {
            "task_families": ["empty_motion", "obstacle_clearance", "pick_place"],
            "sensors": sensor_names,
        },
        "end_effector": end_effector,
        "layouts": {
            "single": {
                "world": "single",
                "namespaces": {"robot": "", "sensors": "", "supervisor": ""},
                "moveit": {"namespace": "", "monitored_planning_scene_topic": "/monitored_planning_scene"},
            }
        },
        "robot": {
            "xacro": {"package": package_name, "path": f"description/robots/{robot_name}.urdf.xacro"},
            "spawn_name": robot_name,
            "spawn_node_name": f"spawn_{robot_name}",
            "allow_renaming": False,
            "xacro_args": {},
        },
        "worlds": {
            "single": {
                "scene": {"package": package_name, "path": "robot_sim/scenes/smoke_scene.yaml"}
            }
        },
        "gazebo": {
            "launch": {"package": "ros_gz_sim", "path": "launch/gz_sim.launch.py"},
            "gz_version": "8",
            "on_exit_shutdown": "true",
            "args": {"gui": "-r ", "headless": "-r -s "},
            "resource_env_vars": ["GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH"],
            "resource_paths": [{"package": package_name}],
        },
        "control": {
            "controllers_file": {"package": package_name, "path": "control/controllers.yaml"},
            "controller_manager_name": "controller_manager",
            "hardware_plugins": {
                "gazebo": "gz_ros2_control/GazeboSimSystem",
                "mock": "mock_components/GenericSystem",
            },
            "spawners": spawners,
        },
        "moveit": {
            "launch": {"package": "robot_sim_moveit_config", "path": "launch/moveit.launch.py"},
            "robot_xacro": {"package": package_name, "path": f"description/robots/{robot_name}.urdf.xacro"},
            "srdf_file": {"package": package_name, "path": f"moveit_config/config/{robot_name}.srdf"},
            "kinematics_yaml": {"package": package_name, "path": "moveit_config/config/kinematics.yaml"},
            "joint_limits_yaml": {"package": package_name, "path": "moveit_config/config/joint_limits.yaml"},
            "moveit_controllers_yaml": {"package": package_name, "path": "moveit_config/config/moveit_controllers.yaml"},
            "ompl_planning_yaml": {"package": package_name, "path": "moveit_config/config/ompl_planning.yaml"},
            "rviz_config": {"package": package_name, "path": "moveit_config/rviz/robot.rviz"},
        },
        "bridges": bridges,
        "bridge_groups": bridge_groups,
        "sensors": sensors,
        "smoke": {
            "controllers": {
                "required": ["joint_state_broadcaster", "arm_controller"],
                "primary_trajectory": "arm_controller",
            },
            "sensors": {"min_hz": 1.0, "min_samples": 3, "topic_timeout": 6.0},
            "tf": {"required_frames": []},
        },
    }


def _scene():
    return {
        "schema": 3,
        "kind": "scene",
        "name": "smoke_scene",
        "description": "Minimal scaffold smoke scene.",
        "parameters": {},
        "variants": {},
        "generators": [],
        "world": {
            "name": "smoke_scene",
            "sdf_version": "1.7",
            "gravity": [0.0, 0.0, -9.8],
            "magnetic_field": [0.0, 0.0, 0.0],
            "atmosphere": {"type": "adiabatic"},
            "physics": {"name": "physics", "type": "dart", "max_step_size": 0.001, "real_time_factor": 1.0},
            "plugins": [
                {"filename": "gz-sim-physics-system", "name": "gz::sim::systems::Physics"},
                {"filename": "gz-sim-user-commands-system", "name": "gz::sim::systems::UserCommands"},
                {"filename": "gz-sim-scene-broadcaster-system", "name": "gz::sim::systems::SceneBroadcaster"},
            ],
            "scene": {"ambient": [0.72, 0.72, 0.72], "background": [0.78, 0.80, 0.82], "grid": True},
            "gui": {
                "fullscreen": 0,
                "camera": {
                    "plugin_filename": "MinimalScene",
                    "plugin_name": "3D View",
                    "gz-gui": {"properties": [{"type": "string", "key": "state", "value": "docked"}]},
                    "engine": "ogre2",
                    "scene": "scene",
                    "ambient_light": [0.55, 0.55, 0.55],
                    "background_color": [0.78, 0.80, 0.82],
                    "pose": [-3.0, -2.2, 2.0, 0.0, 0.42, 0.62],
                    "clip": {"near": 0.05, "far": 25000.0},
                },
                "plugins": [],
            },
        },
        "ground": {
            "name": "ground_plane",
            "type": "model",
            "static": True,
            "collision": True,
            "visual_only": False,
            "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "geometry": {"type": "plane", "normal": [0.0, 0.0, 1.0], "size": [24.0, 24.0]},
            "material": {"ambient": [0.62, 0.64, 0.65, 1.0], "diffuse": [0.62, 0.64, 0.65, 1.0]},
            "tags": ["ground"],
        },
        "lights": [{
            "name": "sun",
            "type": "directional",
            "cast_shadows": True,
            "pose": [0.0, 0.0, 8.0, 0.0, 0.0, 0.0],
            "diffuse": [0.85, 0.85, 0.82, 1.0],
            "specular": [0.20, 0.20, 0.20, 1.0],
            "attenuation": {"range": 1000.0, "constant": 0.9, "linear": 0.01, "quadratic": 0.001},
            "direction": [-0.45, 0.20, -0.86],
        }],
        "robot_mount_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "workspace": {"frame": "world", "bounds": {"min": [-1.5, -1.5, 0.0], "max": [1.5, 1.5, 1.8]}},
        "objects": [],
        "regions": {
            "workspace_sample": {
                "frame": "world",
                "bounds": {"min": [-1.0, -1.0, 0.2], "max": [1.0, 1.0, 1.2]},
                "orientation_rpy": [0.0, 0.0, 0.0],
            }
        },
    }


def _bridge_topics(sensor_name):
    return {
        "camera": [
            _bridge_topic("camera/color/image_raw", "/camera/color/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image"),
            _bridge_topic("camera/color/camera_info", "/camera/color/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
        ],
        "depth": [
            _bridge_topic("camera/depth/image_raw", "/camera/depth/depth_image", "sensor_msgs/msg/Image", "gz.msgs.Image"),
            _bridge_topic("camera/depth/camera_info", "/camera/depth/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
            _bridge_topic("camera/points", "/camera/depth/points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked"),
        ],
        "lidar": [
            _bridge_topic("scan", "/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
            _bridge_topic("lidar/points", "/lidar/points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked"),
        ],
        "imu": [
            _bridge_topic("imu/data", "/imu/data", "sensor_msgs/msg/Imu", "gz.msgs.IMU"),
        ],
    }.get(sensor_name, [])


def _bridge_topic(ros_topic, gz_topic, ros_type, gz_type):
    return {
        "ros_topic_name": ros_topic,
        "gz_topic_name": gz_topic,
        "ros_type_name": ros_type,
        "gz_type_name": gz_type,
        "direction": "GZ_TO_ROS",
    }


def _validation_case(package_name, robot_name, planning_group, tool_link):
    return {
        "schema": 3,
        "kind": "validation_case",
        "name": "smoke_empty_motion",
        "description": "Scaffolded empty-motion validation case.",
        "launch": {
            "profile": robot_name,
            "profile_package": package_name,
            "mode": "mock",
            "layout": "single",
            "timeout_sec": 60.0,
            "sensor_overrides": "",
        },
        "scene": {"package": package_name, "path": "robot_sim/scenes/smoke_scene.yaml"},
        "task": {
            "type": "empty_motion",
            "seed": 1,
            "waypoints": ["workspace_sample", "workspace_sample"],
            "moveit": {
                "group": planning_group,
                "target_link": tool_link,
                "frame": "world",
                "planning_time_sec": 5.0,
                "velocity_scaling": 0.2,
                "acceleration_scaling": 0.2,
            },
        },
        "planning_scene": {"apply": False, "exclude_tags": ["ground"], "include_tags": []},
        "expect": {
            "position_tolerance_m": 0.15,
            "orientation_tolerance_rad": 3.14159,
            "max_goal_position_error_m": 0.30,
            "min_tcp_clearance_m": 0.0,
            "max_controller_error_rad": 0.60,
            "required_sensor_min_hz": 1.0,
            "require_tf_ok": True,
            "topics": [],
        },
        "artifacts": {
            "rosbag": {"enabled": True, "topic_group": "all", "compression": False, "extra_topics": []},
            "reports": ["md", "html"],
        },
    }


def _package_xml(package_name):
    return f"""<?xml version="1.0"?>
<package format="3">
  <name>{package_name}</name>
  <version>0.1.0</version>
  <description>External robot_sim scaffold package.</description>
  <maintainer email="todo@example.com">TODO</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_sim_bringup</exec_depend>
  <exec_depend>robot_sim_description</exec_depend>
  <exec_depend>robot_sim_moveit_config</exec_depend>
</package>
"""


def _cmake(package_name):
    return f"""cmake_minimum_required(VERSION 3.8)
project({package_name})

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  control
  description
  moveit_config
  robot_sim
  DESTINATION share/${{PROJECT_NAME}}
)

ament_package()
"""


def _urdf_xacro(robot_name, joint_names, tool_link, with_gripper):
    links = ["  <link name=\"base_link\"/>"]
    joints = []
    parent = "base_link"
    for index, joint_name in enumerate(joint_names, start=1):
        child = f"link_{index}"
        links.append(f"  <link name=\"{child}\"/>")
        joints.append(f"""  <joint name=\"{joint_name}\" type=\"revolute\">
    <parent link=\"{parent}\"/>
    <child link=\"{child}\"/>
    <origin xyz=\"0.0 0.0 0.12\" rpy=\"0 0 0\"/>
    <axis xyz=\"0 0 1\"/>
    <limit lower=\"-3.14\" upper=\"3.14\" effort=\"50\" velocity=\"1.0\"/>
  </joint>""")
        parent = child
    links.append(f"  <link name=\"{tool_link}\"/>")
    joints.append(f"""  <joint name=\"tool_fixed_joint\" type=\"fixed\">
    <parent link=\"{parent}\"/>
    <child link=\"{tool_link}\"/>
    <origin xyz=\"0.0 0.0 0.12\" rpy=\"0 0 0\"/>
  </joint>""")
    if with_gripper:
        links.extend(["  <link name=\"left_finger\"/>", "  <link name=\"right_finger\"/>"])
        joints.extend([
            f"""  <joint name=\"left_finger_joint\" type=\"prismatic\">
    <parent link=\"{tool_link}\"/>
    <child link=\"left_finger\"/>
    <origin xyz=\"0.0 0.03 0.0\" rpy=\"0 0 0\"/>
    <axis xyz=\"0 1 0\"/>
    <limit lower=\"0.0\" upper=\"0.04\" effort=\"20\" velocity=\"0.2\"/>
  </joint>""",
            f"""  <joint name=\"right_finger_joint\" type=\"prismatic\">
    <parent link=\"{tool_link}\"/>
    <child link=\"right_finger\"/>
    <origin xyz=\"0.0 -0.03 0.0\" rpy=\"0 0 0\"/>
    <axis xyz=\"0 -1 0\"/>
    <limit lower=\"0.0\" upper=\"0.04\" effort=\"20\" velocity=\"0.2\"/>
  </joint>""",
        ])
    return f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{robot_name}">
  <xacro:arg name="hardware_plugin" default="mock_components/GenericSystem"/>
  <xacro:arg name="controllers_file" default=""/>
  <xacro:arg name="controller_manager_name" default="controller_manager"/>
  <xacro:arg name="use_gz_ros2_control" default="false"/>
  <xacro:arg name="ros_namespace" default=""/>
  <xacro:arg name="enable_camera" default="true"/>
  <xacro:arg name="enable_depth" default="true"/>
  <xacro:arg name="enable_lidar" default="true"/>
  <xacro:arg name="enable_imu" default="true"/>
{chr(10).join(links)}
{chr(10).join(joints)}
</robot>
"""


def _controllers_yaml(joint_names, with_gripper):
    manager = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": 100,
                "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
                "arm_controller": {"type": "joint_trajectory_controller/JointTrajectoryController"},
            }
        },
        "arm_controller": {
            "ros__parameters": {
                "joints": joint_names,
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            }
        },
    }
    if with_gripper:
        manager["controller_manager"]["ros__parameters"]["gripper_controller"] = {
            "type": "joint_trajectory_controller/JointTrajectoryController"
        }
        manager["gripper_controller"] = {
            "ros__parameters": {
                "joints": ["left_finger_joint", "right_finger_joint"],
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            }
        }
    return yaml.safe_dump(manager, sort_keys=False)


def _srdf(robot_name, planning_group, tool_link, joint_names, with_gripper):
    joints = "\n".join(f"    <joint name=\"{joint}\"/>" for joint in joint_names)
    gripper = ""
    if with_gripper:
        gripper = """
  <group name="gripper">
    <joint name="left_finger_joint"/>
    <joint name="right_finger_joint"/>
  </group>"""
    return f"""<?xml version="1.0"?>
<robot name="{robot_name}">
  <group name="{planning_group}">
{joints}
    <link name="{tool_link}"/>
  </group>{gripper}
</robot>
"""


def _kinematics_yaml(planning_group):
    return yaml.safe_dump({planning_group: {"kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin"}}, sort_keys=False)


def _joint_limits_yaml(joint_names):
    return yaml.safe_dump({
        "robot_description_planning": {
            "joint_limits": {
                joint: {"has_velocity_limits": True, "max_velocity": 1.0, "has_acceleration_limits": True, "max_acceleration": 1.0}
                for joint in joint_names
            }
        }
    }, sort_keys=False)


def _moveit_controllers_yaml(joint_names, with_gripper):
    manager = {
        "moveit_simple_controller_manager": {
            "controller_names": ["arm_controller"],
            "arm_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "joints": joint_names,
            },
        }
    }
    if with_gripper:
        manager["moveit_simple_controller_manager"]["controller_names"].append("gripper_controller")
        manager["moveit_simple_controller_manager"]["gripper_controller"] = {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "joints": ["left_finger_joint", "right_finger_joint"],
        }
    return yaml.safe_dump(manager, sort_keys=False)


def _ompl_yaml(planning_group):
    return yaml.safe_dump({planning_group: {"planner_configs": ["RRTConnectkConfigDefault"]}}, sort_keys=False)


def _rviz(planning_group):
    return f"""Panels:
  - Class: rviz_common/Displays
Visualization Manager:
  Displays:
    - Class: moveit_rviz_plugin/MotionPlanning
      Name: MotionPlanning
      Planning Group: {planning_group}
"""


def _split_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def _safe_identifier(value: str, label: str) -> str:
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", value):
        raise RuntimeError(f"{label} must be a valid ROS-style identifier: {value}")
    return value


if __name__ == "__main__":
    sys.exit(main())
