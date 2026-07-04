import os
from pathlib import Path
import re
import tempfile
from copy import deepcopy
from hashlib import sha256
from xml.etree import ElementTree as ET

import yaml
from robot_sim_scenarios import build_world, load_scene
from robot_sim_scenarios.schema_validation import validate_config_schema as validate_scenario_schema

from robot_sim_bringup.common.registry import resolve_profile_path
from robot_sim_bringup.common.schema_validation import SCHEMA_VERSION, validate_config_schema


SIM_MODES_SCHEMA_VERSION = 1
BRIDGE_GROUP_SCHEMA_VERSION = 1


def load_sim_mode(mode_name):
    modes_path = os.path.join(
        _package_share_directory("robot_sim_bringup"),
        "config",
        "sim_modes.yaml",
    )
    raw = _load_yaml(modes_path, "sim_modes")
    if raw.get("schema") != SIM_MODES_SCHEMA_VERSION:
        raise RuntimeError(f"sim_modes schema must be {SIM_MODES_SCHEMA_VERSION}: {modes_path}")

    modes = raw.get("modes")
    if not isinstance(modes, dict):
        raise RuntimeError(f"sim_modes.yaml must define a modes mapping: {modes_path}")

    if not mode_name:
        mode_name = raw.get("default", "light")
    if mode_name not in modes:
        valid = ", ".join(sorted(modes))
        raise RuntimeError(f"sim_mode must be one of {valid}; got '{mode_name}'")

    mode = dict(modes[mode_name])
    mode["name"] = mode_name
    mode["delays"] = _resolve_delays(mode.get("delays", {}), mode_name)
    return mode


def load_sim_profile(
    profile_name="panda",
    profile_file="",
    profile_package="",
    require_moveit=False,
    include_optional_moveit=False,
):
    profile_path = str(_profile_path(profile_name, profile_file, profile_package))
    raw = _load_yaml(profile_path, "sim_profile")

    validate_config_schema(raw, "sim_profile.schema.json", "sim_profile", profile_path)

    name = raw.get("name") or os.path.splitext(os.path.basename(profile_path))[0]
    startup_bridges = _resolve_startup_bridges(raw, profile_path)
    profile = {
        "name": name,
        "path": profile_path,
        "metadata": _resolve_metadata(raw, name),
        "capabilities": _resolve_capabilities(raw),
        "end_effector": _resolve_end_effector(raw, profile_path),
        "robot": _resolve_robot(raw, profile_path),
        "layouts": _resolve_layouts(raw, profile_path),
        "worlds": _resolve_worlds(raw, profile_path),
        "gazebo": _resolve_gazebo(raw, profile_path),
        "control": _resolve_control(raw, profile_path),
        "startup_bridges": startup_bridges,
        "bridges": _resolve_bridge_groups(raw, startup_bridges, profile_path),
        "sensors": _resolve_sensors(raw, profile_path),
        "smoke": _resolve_smoke(raw, profile_path),
    }
    if require_moveit or (include_optional_moveit and "moveit" in raw):
        profile["moveit"] = _resolve_moveit(raw, profile_path)
    return profile


def _profile_path(profile_name, profile_file, profile_package=""):
    path = resolve_profile_path(profile_name or "panda", profile_file, profile_package)
    _require_path(path, "sim_profile")
    return path


def _load_yaml(path, field_name):
    _require_path(path, field_name)
    with open(path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, dict):
        raise RuntimeError(f"{field_name} must be a YAML mapping: {path}")
    return raw


def _resolve_metadata(raw, profile_name):
    metadata = dict(raw.get("metadata", {}))
    metadata.setdefault("package", "robot_sim_bringup")
    metadata.setdefault("robot_name", profile_name)
    return {str(key): value for key, value in metadata.items()}


def _resolve_capabilities(raw):
    capabilities = raw.get("capabilities", {})
    if not isinstance(capabilities, dict):
        raise RuntimeError("capabilities must be a mapping")
    return {
        "task_families": [
            str(item)
            for item in capabilities.get("task_families", [])
        ],
        "sensors": [
            str(item)
            for item in capabilities.get("sensors", [])
        ],
        **{
            str(key): value
            for key, value in capabilities.items()
            if key not in ("task_families", "sensors")
        },
    }


def _resolve_end_effector(raw, profile_path):
    end_effector = _section(raw, "end_effector", profile_path)
    result = {
        "planning_group": str(end_effector.get("planning_group", "manipulator")),
        "tool_link": str(end_effector.get("tool_link", "tool0")),
        "base_frame": str(end_effector.get("base_frame", "world")),
    }
    if "gripper" in end_effector:
        gripper = end_effector["gripper"]
        if not isinstance(gripper, dict):
            raise RuntimeError(f"end_effector.gripper must be a mapping: {profile_path}")
        result["gripper"] = {
            "controller": str(gripper.get("controller", "")),
            "open_positions": [
                float(value)
                for value in gripper.get("open_positions", [])
            ],
            "closed_positions": [
                float(value)
                for value in gripper.get("closed_positions", [])
            ],
        }
    return result


def _resolve_delays(raw, mode_name):
    delays = {}
    for name in ("gazebo", "spawn", "bridge", "controller", "moveit"):
        if name not in raw:
            raise RuntimeError(f"sim_modes.modes.{mode_name}.delays.{name} is required")
        delays[name] = float(raw[name])
    return delays


def _resolve_robot(raw, profile_path):
    robot = _section(raw, "robot", profile_path)
    xacro_args = {}
    for name, value in robot.get("xacro_args", {}).items():
        xacro_args[name] = _resolve_value(value, f"robot.xacro_args.{name}")

    return {
        "xacro": _resolve_path(robot.get("xacro"), "robot.xacro"),
        "spawn_name": robot.get("spawn_name") or robot.get("name") or "robot",
        "spawn_node_name": robot.get("spawn_node_name") or "spawn_robot",
        "allow_renaming": bool(robot.get("allow_renaming", False)),
        "spawn_pose": robot.get("spawn_pose", {}),
        "xacro_args": xacro_args,
    }


def _resolve_layouts(raw, profile_path):
    layouts = _section(raw, "layouts", profile_path)
    resolved = {}
    for name, layout in layouts.items():
        if not isinstance(layout, dict):
            raise RuntimeError(f"layouts.{name} must be a mapping: {profile_path}")
        namespaces = layout.get("namespaces", {})
        if not isinstance(namespaces, dict):
            raise RuntimeError(f"layouts.{name}.namespaces must be a mapping: {profile_path}")
        resolved[name] = {
            "world": layout.get("world", name),
            "namespaces": {key: str(value) for key, value in namespaces.items()},
            "robot_description_topic": layout.get("robot_description_topic"),
            "moveit": dict(layout.get("moveit", {})),
            "rqt_graph": dict(layout.get("rqt_graph", {})),
        }
    return resolved


def _resolve_worlds(raw, profile_path):
    worlds = _section(raw, "worlds", profile_path)
    resolved = {}
    for name, spec in worlds.items():
        field_name = f"worlds.{name}"
        if isinstance(spec, dict) and "scene" in spec:
            resolved[name] = _resolve_world_scene(spec["scene"], field_name)
        elif isinstance(spec, dict) and "world_preset" in spec:
            resolved[name] = _resolve_world_preset(spec["world_preset"], field_name)
        elif isinstance(spec, dict) and "file" in spec:
            resolved[name] = {
                "name": name,
                "source": "file",
                "path": _resolve_path(spec["file"], f"{field_name}.file"),
            }
        else:
            raise RuntimeError(
                f"{field_name} must define exactly one of scene, world_preset, or file: {profile_path}"
            )
    return resolved


def _resolve_world_scene(spec, field_name):
    scene_ref = _resolve_scene_ref(spec, f"{field_name}.scene")
    variant = spec.get("variant", "") if isinstance(spec, dict) else ""
    parameters = spec.get("parameters", {}) if isinstance(spec, dict) else {}
    scene = load_scene(scene_ref, variant=variant, parameters=parameters)
    world_path = build_world(scene)
    return {
        "name": scene.name,
        "source": "scene",
        "path": str(world_path),
        "scene": {
            "name": scene.name,
            "path": str(scene.path),
            "description": scene.description,
            "workspace": {
                "frame": scene.workspace.frame,
                "bounds": {
                    "min": list(scene.workspace.min_bounds),
                    "max": list(scene.workspace.max_bounds),
                },
            },
            "regions": {
                name: {
                    "frame": region.frame,
                    "bounds": {
                        "min": list(region.min_bounds),
                        "max": list(region.max_bounds),
                    },
                    "orientation_rpy": list(region.orientation_rpy),
                }
                for name, region in scene.regions.items()
            },
        },
        "variant": str(variant or ""),
        "parameters": dict(parameters or {}),
    }


def _resolve_scene_ref(spec, field_name):
    if isinstance(spec, dict):
        if spec.get("path"):
            return _resolve_path(spec, field_name)
        if spec.get("name"):
            from robot_sim_bringup.common.registry import resolve_scene_path

            return str(resolve_scene_path(str(spec["name"]), str(spec.get("package", ""))))
        raise RuntimeError(f"{field_name} must define path or name")
    if isinstance(spec, str):
        candidate = os.path.expanduser(os.path.expandvars(spec))
        if os.path.isabs(candidate) or candidate.startswith("package://"):
            return _resolve_path(spec, field_name)
        return spec
    raise RuntimeError(f"{field_name} must be a string or mapping")


def _resolve_world_preset(spec, field_name):
    preset_path = _resolve_path(spec, f"{field_name}.world_preset")
    scenario = _load_yaml(preset_path, f"{field_name}.world_preset")
    validate_scenario_schema(scenario, "world_preset.schema.json", "world_preset", preset_path)

    base = scenario.get("base")
    if not isinstance(base, dict):
        raise RuntimeError(f"{field_name}.world_preset.base must be a mapping: {preset_path}")
    base_path = _resolve_path(base, f"{field_name}.world_preset.base")

    assets = scenario.get("assets", [])
    if not isinstance(assets, list):
        raise RuntimeError(f"{field_name}.world_preset.assets must be a list: {preset_path}")

    resolved_assets = []
    for index, asset in enumerate(assets):
        asset_field = f"{field_name}.world_preset.assets[{index}]"
        if not isinstance(asset, dict):
            raise RuntimeError(f"{asset_field} must be a mapping: {preset_path}")
        resolved_assets.append({
            "name": str(asset.get("name") or f"asset_{index}"),
            "type": str(asset.get("type", "model")),
            "path": _resolve_path(asset, asset_field),
            "pose": _pose_text(asset.get("pose"), asset_field),
            "tags": [str(tag) for tag in asset.get("tags", [])],
        })

    world_path = _compose_world_scenario(
        preset_path,
        scenario,
        base_path,
        resolved_assets,
        field_name,
    )
    scenario_meta = scenario.get("scenario", {})
    if not isinstance(scenario_meta, dict):
        scenario_meta = {}

    return {
        "name": str(scenario.get("name") or os.path.splitext(os.path.basename(preset_path))[0]),
        "source": "world_preset",
        "path": world_path,
        "world_preset": {
            "path": preset_path,
            "base": base_path,
            "world_name": str(scenario.get("world_name", "default")),
            "type": str(scenario_meta.get("type", "")),
            "task_family": str(scenario_meta.get("task_family", "")),
            "assets": resolved_assets,
        },
    }


def _pose_text(value, field_name):
    if value is None:
        return None
    if isinstance(value, str):
        parts = value.split()
    else:
        parts = list(value)
    if len(parts) != 6:
        raise RuntimeError(f"{field_name}.pose must contain 6 values")
    return " ".join(str(part) for part in parts)


def _compose_world_scenario(scenario_path, scenario, base_path, assets, field_name):
    try:
        base_tree = ET.parse(base_path)
    except ET.ParseError as exc:
        raise RuntimeError(f"{field_name}.scenario.base is not valid XML: {base_path}: {exc}") from exc

    base_root = base_tree.getroot()
    world = _single_child(base_root, "world", f"{field_name}.scenario.base")
    world_name = scenario.get("world_name")
    if world_name:
        world.set("name", str(world_name))

    for asset in assets:
        asset_element = _load_asset_element(asset["path"], field_name)
        if asset["name"]:
            asset_element.set("name", asset["name"])
        if asset["pose"]:
            _set_pose(asset_element, asset["pose"])
        world.append(asset_element)

    if hasattr(ET, "indent"):
        ET.indent(base_tree, space="  ")

    output_path = _scenario_output_path(scenario_path, base_path, assets)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    base_tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path


def _load_asset_element(path, field_name):
    try:
        tree = ET.parse(path)
    except ET.ParseError as exc:
        raise RuntimeError(f"{field_name}.scenario asset is not valid XML: {path}: {exc}") from exc

    root = tree.getroot()
    if root.tag == "sdf":
        children = [
            child
            for child in list(root)
            if _strip_xml_namespace(child.tag) in ("model", "light", "include")
        ]
        if len(children) != 1:
            raise RuntimeError(
                f"{field_name}.scenario asset must contain exactly one model/light/include: {path}"
            )
        return deepcopy(children[0])

    if _strip_xml_namespace(root.tag) in ("model", "light", "include"):
        return deepcopy(root)

    raise RuntimeError(f"{field_name}.scenario asset must be an SDF model/light/include: {path}")


def _single_child(root, child_name, field_name):
    matches = [
        child for child in list(root)
        if _strip_xml_namespace(child.tag) == child_name
    ]
    if len(matches) != 1:
        raise RuntimeError(f"{field_name} must contain exactly one <{child_name}> element")
    return matches[0]


def _strip_xml_namespace(tag):
    return tag.rsplit("}", 1)[-1]


def _set_pose(element, pose_text):
    pose = None
    for child in list(element):
        if _strip_xml_namespace(child.tag) == "pose":
            pose = child
            break
    if pose is None:
        pose = ET.Element("pose")
        element.insert(0, pose)
    pose.text = pose_text


def _scenario_output_path(scenario_path, base_path, assets):
    digest = sha256()
    for path in [scenario_path, base_path, *[asset["path"] for asset in assets]]:
        with open(path, "rb") as handle:
            digest.update(handle.read())
    for asset in assets:
        digest.update(asset["name"].encode("utf-8"))
        digest.update((asset["pose"] or "").encode("utf-8"))
    name = os.path.splitext(os.path.basename(scenario_path))[0]
    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", name)
    return os.path.join(
        tempfile.gettempdir(),
        "robot_sim_worlds",
        f"{safe_name}-{digest.hexdigest()[:12]}.world.sdf",
    )


def _resolve_gazebo(raw, profile_path):
    gazebo = _section(raw, "gazebo", profile_path)
    launch = _section(gazebo, "launch", profile_path)
    args = gazebo.get("args", {})
    resource_env_vars = gazebo.get("resource_env_vars", [])
    resource_paths = []
    for index, spec in enumerate(gazebo.get("resource_paths", [])):
        resource_paths.append(_resolve_path(spec, f"gazebo.resource_paths[{index}]"))

    return {
        "launch": _resolve_path(launch, "gazebo.launch"),
        "gz_version": str(gazebo.get("gz_version", "8")),
        "on_exit_shutdown": str(gazebo.get("on_exit_shutdown", "true")),
        "args": {
            "gui": str(args.get("gui", "-r ")),
            "headless": str(args.get("headless", "-r -s ")),
        },
        "resource_env_vars": [str(name) for name in resource_env_vars],
        "resource_paths": resource_paths,
    }


def _resolve_control(raw, profile_path):
    control = _section(raw, "control", profile_path)
    hardware_plugins = control.get("hardware_plugins", {})
    spawners = control.get("spawners", [])
    if not isinstance(spawners, list) or not spawners:
        raise RuntimeError(f"control.spawners must be a non-empty list: {profile_path}")

    resolved_spawners = []
    for index, spawner in enumerate(spawners):
        if not isinstance(spawner, dict) or not spawner.get("name"):
            raise RuntimeError(
                f"control.spawners[{index}] must define a controller name: {profile_path}"
            )
        resolved_spawners.append(dict(spawner))

    return {
        "controllers_file": _resolve_path(
            control.get("controllers_file"),
            "control.controllers_file",
        ),
        "controller_manager_name": (
            control.get("controller_manager_name") or "controller_manager"
        ),
        "hardware_plugins": {
            "gazebo": hardware_plugins.get(
                "gazebo",
                "gz_ros2_control/GazeboSimSystem",
            ),
            "mock": hardware_plugins.get("mock", "mock_components/GenericSystem"),
        },
        "spawners": resolved_spawners,
    }


def _resolve_startup_bridges(raw, profile_path):
    bridges = raw.get("bridges", [])
    if not isinstance(bridges, list):
        raise RuntimeError(f"bridges must be a list: {profile_path}")
    return [str(name) for name in bridges]


def _resolve_bridge_groups(raw, startup_bridges, profile_path):
    bridge_names = set(startup_bridges)
    sensors = raw.get("sensors", {})
    if isinstance(sensors, dict):
        for sensor in sensors.values():
            if isinstance(sensor, dict) and sensor.get("bridge_group"):
                bridge_names.add(str(sensor["bridge_group"]))

    profile_bridge_groups = raw.get("bridge_groups", {})
    if profile_bridge_groups is None:
        profile_bridge_groups = {}
    if not isinstance(profile_bridge_groups, dict):
        raise RuntimeError(f"bridge_groups must be a mapping: {profile_path}")

    resolved = {}
    for name in sorted(bridge_names):
        if name in profile_bridge_groups:
            resolved[name] = _resolve_bridge_group_spec(
                name,
                profile_bridge_groups[name],
                profile_path,
            )
        else:
            resolved[name] = _load_bridge_group(name)
    return resolved


def _load_bridge_group(name):
    path = os.path.join(
        _package_share_directory("robot_sim_bringup"),
        "config",
        "bridge_groups",
        f"{name}.yaml",
    )
    raw = _load_yaml(path, f"bridge_group.{name}")
    if raw.get("schema") != BRIDGE_GROUP_SCHEMA_VERSION:
        raise RuntimeError(f"bridge_group schema must be {BRIDGE_GROUP_SCHEMA_VERSION}: {path}")

    config = _resolve_path(raw.get("config"), f"bridge_group.{name}.config")
    return {
        "name": raw.get("name", name),
        "package": raw.get("package", "ros_gz_bridge"),
        "executable": raw.get("executable", "parameter_bridge"),
        "node_name": raw.get("node_name") or f"{name}_bridge",
        "namespace": raw.get("namespace", "sensors"),
        "output": raw.get("output", "screen"),
        "config": config,
        "topics": _load_bridge_topics(config, f"bridge_group.{name}.config"),
    }


def _resolve_bridge_group_spec(name, spec, profile_path):
    field_name = f"bridge_groups.{name}"
    if not isinstance(spec, dict):
        raise RuntimeError(f"{field_name} must be a mapping: {profile_path}")

    has_config = spec.get("config") is not None
    has_topics = spec.get("topics") is not None
    if has_config and has_topics:
        raise RuntimeError(f"{field_name} must use either config or topics, not both")
    if not has_config and not has_topics:
        raise RuntimeError(f"{field_name} must define config or topics")

    if has_config:
        config = _resolve_path(spec.get("config"), f"{field_name}.config")
        topics = _load_bridge_topics(config, f"{field_name}.config")
    else:
        topics = _normalize_bridge_topics(spec.get("topics"), f"{field_name}.topics")
        config = _write_inline_bridge_config(name, topics)

    return {
        "name": spec.get("name", name),
        "package": spec.get("package", "ros_gz_bridge"),
        "executable": spec.get("executable", "parameter_bridge"),
        "node_name": spec.get("node_name") or f"{name}_bridge",
        "namespace": spec.get("namespace", "sensors"),
        "output": spec.get("output", "screen"),
        "config": config,
        "topics": topics,
    }


def _load_bridge_topics(path, field_name):
    _require_path(path, field_name)
    with open(path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    return _normalize_bridge_topics(raw, field_name)


def _normalize_bridge_topics(raw, field_name):
    if not isinstance(raw, list):
        raise RuntimeError(f"{field_name} must be a list of bridge topic mappings")

    required = (
        "ros_topic_name",
        "gz_topic_name",
        "ros_type_name",
        "gz_type_name",
        "direction",
    )
    topics = []
    for index, item in enumerate(raw):
        item_field = f"{field_name}[{index}]"
        if not isinstance(item, dict):
            raise RuntimeError(f"{item_field} must be a mapping")
        missing = [name for name in required if not item.get(name)]
        if missing:
            raise RuntimeError(f"{item_field} missing required fields: {', '.join(missing)}")
        topics.append({name: str(item[name]) for name in required})
    return topics


def _write_inline_bridge_config(name, topics):
    digest = sha256()
    digest.update(name.encode("utf-8"))
    digest.update(yaml.safe_dump(topics, sort_keys=True).encode("utf-8"))
    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", name)
    path = os.path.join(
        tempfile.gettempdir(),
        "robot_sim_bridge_configs",
        f"{safe_name}-{digest.hexdigest()[:12]}.yaml",
    )
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(topics, handle, sort_keys=False)
    return path


def _resolve_sensors(raw, profile_path):
    sensors = raw.get("sensors", {})
    if not isinstance(sensors, dict):
        raise RuntimeError(f"sensors must be a mapping: {profile_path}")

    resolved = {}
    for name, sensor in sensors.items():
        if not isinstance(sensor, dict):
            raise RuntimeError(f"sensors.{name} must be a mapping: {profile_path}")
        bridge_group = sensor.get("bridge_group")
        if bridge_group:
            bridge_group = str(bridge_group)
        resolved[name] = {
            "xacro_arg": sensor.get("xacro_arg"),
            "default_enabled": bool(sensor.get("default_enabled", True)),
            "bridge_group": bridge_group,
            "static_tfs": sensor.get("static_tfs", []),
            "receiver": _resolve_sensor_receiver(name, sensor, profile_path),
        }
    return resolved


def _resolve_sensor_receiver(sensor_name, sensor, profile_path):
    receiver = sensor.get("receiver")
    if receiver is None:
        return None
    if not isinstance(receiver, dict):
        raise RuntimeError(
            f"sensors.{sensor_name}.receiver must be a mapping: {profile_path}"
        )

    missing = [
        field
        for field in ("package", "executable", "type")
        if not receiver.get(field)
    ]
    if missing:
        raise RuntimeError(
            f"sensors.{sensor_name}.receiver missing fields: "
            + ", ".join(missing)
            + f": {profile_path}"
        )

    parameters = receiver.get("parameters", {})
    if parameters is None:
        parameters = {}
    if not isinstance(parameters, dict):
        raise RuntimeError(
            f"sensors.{sensor_name}.receiver.parameters must be a mapping: {profile_path}"
        )

    return {
        "package": str(receiver["package"]),
        "executable": str(receiver["executable"]),
        "type": str(receiver["type"]),
        "node_name": str(receiver.get("node_name") or f"{sensor_name}_receiver"),
        "namespace": str(receiver.get("namespace", "sensors")),
        "output": str(receiver.get("output", "screen")),
        "expected_min_hz": float(receiver.get("expected_min_hz", 1.0)),
        "log_period_sec": max(float(receiver.get("log_period_sec", 5.0)), 0.1),
        "parameters": {str(key): value for key, value in parameters.items()},
    }


def _resolve_smoke(raw, profile_path):
    smoke = raw.get("smoke", {})
    if smoke is None:
        smoke = {}
    if not isinstance(smoke, dict):
        raise RuntimeError(f"smoke must be a mapping: {profile_path}")

    controllers = smoke.get("controllers", {})
    sensors = smoke.get("sensors", {})
    tf = smoke.get("tf", {})
    if controllers is None:
        controllers = {}
    if sensors is None:
        sensors = {}
    if tf is None:
        tf = {}
    if not isinstance(controllers, dict):
        raise RuntimeError(f"smoke.controllers must be a mapping: {profile_path}")
    if not isinstance(sensors, dict):
        raise RuntimeError(f"smoke.sensors must be a mapping: {profile_path}")
    if not isinstance(tf, dict):
        raise RuntimeError(f"smoke.tf must be a mapping: {profile_path}")

    required = controllers.get("required")
    if required is not None:
        if not isinstance(required, list):
            raise RuntimeError(f"smoke.controllers.required must be a list: {profile_path}")
        required = [str(name) for name in required]

    required_frames = tf.get("required_frames", [])
    if not isinstance(required_frames, list):
        raise RuntimeError(f"smoke.tf.required_frames must be a list: {profile_path}")

    return {
        "controllers": {
            "required": required,
            "primary_trajectory": (
                str(controllers["primary_trajectory"])
                if controllers.get("primary_trajectory")
                else None
            ),
        },
        "sensors": {
            "min_hz": float(sensors.get("min_hz", 1.0)),
            "min_samples": int(sensors.get("min_samples", 3)),
            "topic_timeout": float(sensors.get("topic_timeout", 6.0)),
        },
        "tf": {
            "required_frames": [str(frame) for frame in required_frames],
        },
    }


def _resolve_moveit(raw, profile_path):
    moveit = _section(raw, "moveit", profile_path)
    return {
        "launch": _resolve_path(moveit.get("launch"), "moveit.launch"),
        "arguments": {
            "robot_xacro": _resolve_path(
                moveit.get("robot_xacro"),
                "moveit.robot_xacro",
            ),
            "srdf_file": _resolve_path(moveit.get("srdf_file"), "moveit.srdf_file"),
            "kinematics_yaml": _resolve_path(
                moveit.get("kinematics_yaml"),
                "moveit.kinematics_yaml",
            ),
            "joint_limits_yaml": _resolve_path(
                moveit.get("joint_limits_yaml"),
                "moveit.joint_limits_yaml",
            ),
            "moveit_controllers_yaml": _resolve_path(
                moveit.get("moveit_controllers_yaml"),
                "moveit.moveit_controllers_yaml",
            ),
            "ompl_planning_yaml": _resolve_path(
                moveit.get("ompl_planning_yaml"),
                "moveit.ompl_planning_yaml",
            ),
            "rviz_config": _resolve_path(moveit.get("rviz_config"), "moveit.rviz_config"),
        },
    }


def _section(raw, name, profile_path):
    value = raw.get(name)
    if not isinstance(value, dict):
        raise RuntimeError(f"sim_profile missing required section '{name}': {profile_path}")
    return value


def _resolve_value(spec, field_name):
    if isinstance(spec, dict) and "value" in spec:
        return str(spec["value"])
    if isinstance(spec, dict) and "package" in spec:
        path = _resolve_path(spec, field_name, must_exist=spec.get("must_exist", True))
        uri = spec.get("uri", "path")
        if uri == "file":
            return f"file://{path}"
        if uri == "package":
            rel_path = spec.get("path", "").lstrip("/")
            return f"package://{spec['package']}/{rel_path}"
        if uri == "path":
            return path
        raise RuntimeError(f"{field_name}.uri must be file, package, or path")
    return str(spec)


def _resolve_path(spec, field_name, must_exist=True):
    if spec is None:
        raise RuntimeError(f"sim_profile missing required field '{field_name}'")

    if isinstance(spec, dict):
        package_name = spec.get("package")
        if not package_name:
            raise RuntimeError(f"{field_name}.package is required")
        base = _package_share_directory(package_name)
        rel_path = spec.get("path", "")
        path = os.path.join(base, rel_path)
    elif isinstance(spec, str):
        if spec.startswith("package://"):
            package_name, rel_path = _split_package_uri(spec, field_name)
            path = os.path.join(_package_share_directory(package_name), rel_path)
        else:
            path = os.path.expanduser(os.path.expandvars(spec))
            if not os.path.isabs(path):
                raise RuntimeError(f"{field_name} must be absolute or package-relative")
    else:
        raise RuntimeError(f"{field_name} must be a string or mapping")

    path = os.path.normpath(path)
    if must_exist:
        _require_path(path, field_name)
    return path


def _split_package_uri(uri, field_name):
    remainder = uri[len("package://") :]
    if "/" not in remainder:
        raise RuntimeError(f"{field_name} package URI must include a relative path")
    package_name, rel_path = remainder.split("/", 1)
    return package_name, rel_path


def _package_share_directory(package_name):
    source_path = _source_package_directory(package_name)
    if source_path is not None:
        return str(source_path)

    from ament_index_python.packages import get_package_share_directory

    return get_package_share_directory(package_name)


def _source_package_directory(package_name):
    current = Path(__file__).resolve()
    for ancestor in current.parents:
        src_dir = ancestor / "src"
        if not src_dir.exists():
            continue
        matches = sorted(src_dir.glob(f"**/{package_name}/package.xml"))
        if matches:
            return matches[0].parent
    package_root = current.parents[1]
    if package_root.name == package_name and (package_root / "package.xml").exists():
        return package_root
    return None


def _require_path(path, field_name):
    if not os.path.exists(path):
        raise RuntimeError(f"{field_name} does not exist: {path}")
