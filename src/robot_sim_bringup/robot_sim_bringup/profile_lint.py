import argparse
import json
import os
import re
import subprocess
import sys
from xml.etree import ElementTree as ET

import yaml
from ament_index_python.packages import get_package_prefix

from robot_sim_bringup.sim_config_loader import load_sim_mode, load_sim_profile


SUCCESS = 0
FAILURE = 1


def _bool_text(value):
    return "true" if value else "false"


def _bool_value(value, name):
    normalized = str(value).strip().lower()
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off", ""):
        return False
    raise RuntimeError(f"{name} must be true or false; got '{value}'")


def _parse_sensor_overrides(text):
    overrides = {}
    if not text:
        return overrides
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            raise RuntimeError("sensor_overrides entries must use name=true or name=false")
        name, value = item.split("=", 1)
        name = name.strip()
        if not name:
            raise RuntimeError("sensor_overrides contains an empty sensor name")
        overrides[name] = _bool_value(value, f"sensor_overrides.{name}")
    return overrides


def _sensor_states(profile, mode_default, overrides_text):
    sensors = {
        name: bool(mode_default and sensor.get("default_enabled", True))
        for name, sensor in profile["sensors"].items()
    }
    for name, enabled in _parse_sensor_overrides(overrides_text).items():
        if name not in profile["sensors"]:
            raise RuntimeError(
                f"sensor_overrides references unknown sensor group '{name}' "
                f"for sim_profile '{profile['name']}'"
            )
        sensors[name] = enabled
    return sensors


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _xacro_declared_args(path):
    with open(path, "r", encoding="utf-8") as handle:
        text = handle.read()
    return set(re.findall(r"<xacro:arg\s+name=['\"]([^'\"]+)['\"]", text))


def _render_urdf(profile, mode, sensors):
    use_gazebo = bool(mode["use_gazebo"])
    hardware_plugin = (
        profile["control"]["hardware_plugins"]["gazebo"]
        if use_gazebo
        else profile["control"]["hardware_plugins"]["mock"]
    )
    xacro_args = dict(profile["robot"].get("xacro_args", {}))
    xacro_args.update({
        "hardware_plugin": hardware_plugin,
        "controllers_file": profile["control"]["controllers_file"],
        "controller_manager_name": profile["control"]["controller_manager_name"],
        "use_gz_ros2_control": _bool_text(use_gazebo),
        "ros_namespace": "",
    })
    for group, enabled in sensors.items():
        sensor = profile["sensors"][group]
        if sensor.get("xacro_arg"):
            xacro_args[sensor["xacro_arg"]] = _bool_text(enabled)

    cmd = ["xacro", profile["robot"]["xacro"]]
    cmd.extend(f"{name}:={value}" for name, value in xacro_args.items())
    result = subprocess.run(
        cmd,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return result.stdout, set(xacro_args)


def _urdf_links(urdf_text):
    root = ET.fromstring(urdf_text)
    return {
        element.attrib["name"].lstrip("/")
        for element in root.findall("link")
        if element.attrib.get("name")
    }


def _controller_types(raw):
    result = {}
    for key, value in raw.items():
        if key.strip("/").split("/")[-1] != "controller_manager":
            continue
        params = value.get("ros__parameters", {}) if isinstance(value, dict) else {}
        if not isinstance(params, dict):
            continue
        for name, spec in params.items():
            if isinstance(spec, dict) and spec.get("type"):
                result[str(name)] = str(spec["type"])
    return result


def _controller_parameters(raw, controller_name):
    for key, value in raw.items():
        if key.strip("/").split("/")[-1] != controller_name:
            continue
        if isinstance(value, dict):
            params = value.get("ros__parameters", {})
            return params if isinstance(params, dict) else {}
    return {}


def _first_trajectory_spawner(profile):
    for spawner in profile["control"]["spawners"]:
        if spawner.get("type") == "joint_trajectory_controller/JointTrajectoryController":
            return spawner["name"]
    return None


def _required_controllers(profile):
    configured = profile["smoke"]["controllers"].get("required")
    if configured is not None:
        return configured
    return [
        spawner["name"]
        for spawner in profile["control"]["spawners"]
        if not spawner.get("enabled_by")
    ]


def _check_xacro(profile, mode, sensors, errors, warnings):
    declared = _xacro_declared_args(profile["robot"]["xacro"])
    required = {
        "hardware_plugin",
        "controllers_file",
        "controller_manager_name",
        "use_gz_ros2_control",
        "ros_namespace",
    }
    for sensor in profile["sensors"].values():
        if sensor.get("xacro_arg"):
            required.add(str(sensor["xacro_arg"]))

    missing = sorted(required - declared)
    if missing:
        errors.append(
            "robot.xacro missing required xacro args: " + ", ".join(missing)
        )

    try:
        urdf_text, passed_args = _render_urdf(profile, mode, sensors)
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        stderr = getattr(exc, "stderr", "") or str(exc)
        errors.append("xacro render failed: " + stderr.strip())
        return set()

    extra_args = sorted(passed_args - declared)
    if extra_args:
        warnings.append(
            "xacro command passes args not declared in robot.xacro: "
            + ", ".join(extra_args)
        )

    try:
        return _urdf_links(urdf_text)
    except ET.ParseError as exc:
        errors.append(f"rendered URDF is not valid XML: {exc}")
        return set()


def _check_controllers(profile, errors):
    raw = _load_yaml(profile["control"]["controllers_file"])
    controller_types = _controller_types(raw)
    spawners = profile["control"]["spawners"]
    spawner_names = {spawner["name"] for spawner in spawners}

    for spawner in spawners:
        name = spawner["name"]
        configured_type = controller_types.get(name)
        if configured_type is None:
            errors.append(f"controller spawner '{name}' missing from controllers yaml")
            continue
        if spawner.get("type") and spawner["type"] != configured_type:
            errors.append(
                f"controller '{name}' type mismatch: "
                f"profile={spawner['type']} yaml={configured_type}"
            )

    for name in _required_controllers(profile):
        if name not in spawner_names:
            errors.append(f"smoke required controller '{name}' is not a spawner")

    primary = (
        profile["smoke"]["controllers"].get("primary_trajectory")
        or _first_trajectory_spawner(profile)
    )
    if not primary:
        errors.append("no JointTrajectoryController spawner found")
        return
    if primary not in spawner_names:
        errors.append(f"smoke primary trajectory controller '{primary}' is not a spawner")
        return

    params = _controller_parameters(raw, primary)
    joints = params.get("joints", [])
    if not isinstance(joints, list) or not joints:
        errors.append(
            f"primary trajectory controller '{primary}' must define "
            "ros__parameters.joints"
        )

    for spawner in spawners:
        if spawner.get("type") != "joint_trajectory_controller/JointTrajectoryController":
            continue
        params = _controller_parameters(raw, spawner["name"])
        joints = params.get("joints", [])
        if not isinstance(joints, list) or not joints:
            errors.append(
                f"trajectory controller '{spawner['name']}' must define "
                "ros__parameters.joints"
            )


def _check_bridges(profile, errors):
    try:
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        errors.append(f"cannot import rosidl_runtime_py for bridge type checks: {exc}")
        return

    for sensor_name, sensor in profile["sensors"].items():
        bridge_group = sensor.get("bridge_group")
        if bridge_group and bridge_group not in profile["bridges"]:
            errors.append(
                f"sensor '{sensor_name}' references missing bridge group '{bridge_group}'"
            )

    for bridge_name, bridge in profile["bridges"].items():
        topics = bridge.get("topics", [])
        if not topics:
            errors.append(f"bridge group '{bridge_name}' has no topics")
            continue
        for index, topic in enumerate(topics):
            prefix = f"bridge group '{bridge_name}' topic[{index}]"
            for field in (
                "ros_topic_name",
                "gz_topic_name",
                "ros_type_name",
                "gz_type_name",
                "direction",
            ):
                if not topic.get(field):
                    errors.append(f"{prefix} missing {field}")
            try:
                get_message(topic["ros_type_name"])
            except Exception as exc:
                errors.append(
                    f"{prefix} has invalid ros_type_name "
                    f"'{topic.get('ros_type_name')}': {exc}"
                )


def _receiver_topic_requirements(receiver_type):
    return {
        "camera": {
            "image_topic": "sensor_msgs/msg/Image",
            "camera_info_topic": "sensor_msgs/msg/CameraInfo",
        },
        "depth": {
            "depth_image_topic": "sensor_msgs/msg/Image",
            "camera_info_topic": "sensor_msgs/msg/CameraInfo",
            "pointcloud_topic": "sensor_msgs/msg/PointCloud2",
        },
        "lidar": {
            "scan_topic": "sensor_msgs/msg/LaserScan",
            "pointcloud_topic": "sensor_msgs/msg/PointCloud2",
        },
        "imu": {
            "imu_topic": "sensor_msgs/msg/Imu",
        },
    }.get(receiver_type, {})


def _check_receivers(profile, sensors, require_receivers, errors, warnings):
    for sensor_name, enabled in sensors.items():
        if not enabled and not require_receivers:
            continue
        sensor = profile["sensors"][sensor_name]
        receiver = sensor.get("receiver")
        if receiver is None:
            if require_receivers:
                errors.append(f"sensor '{sensor_name}' is missing receiver config")
            continue

        package = receiver["package"]
        executable = receiver["executable"]
        try:
            prefix = get_package_prefix(package)
        except Exception as exc:
            errors.append(
                f"sensor '{sensor_name}' receiver package '{package}' is not available: {exc}"
            )
            continue

        executable_path = os.path.join(prefix, "lib", package, executable)
        if not os.path.exists(executable_path):
            errors.append(
                f"sensor '{sensor_name}' receiver executable '{executable}' "
                f"not found at {executable_path}"
            )

        bridge_group = sensor.get("bridge_group")
        topics = []
        if bridge_group:
            bridge = profile["bridges"].get(bridge_group)
            topics = bridge.get("topics", []) if bridge else []

        requirements = _receiver_topic_requirements(receiver["type"])
        if not requirements:
            warnings.append(
                f"sensor '{sensor_name}' receiver type '{receiver['type']}' "
                "has no built-in topic requirement checks"
            )
            if require_receivers and not topics:
                errors.append(
                    f"sensor '{sensor_name}' receiver has no bridge topics to pass through"
                )
            continue

        topic_types = [topic.get("ros_type_name") for topic in topics]
        for parameter_name, ros_type in requirements.items():
            if ros_type not in topic_types:
                errors.append(
                    f"sensor '{sensor_name}' receiver type '{receiver['type']}' "
                    f"requires {ros_type} for parameter '{parameter_name}'"
                )


def _check_tf(profile, urdf_links, errors):
    static_tfs = []
    for sensor_name, sensor in profile["sensors"].items():
        tfs = sensor.get("static_tfs", [])
        if not isinstance(tfs, list):
            errors.append(f"sensors.{sensor_name}.static_tfs must be a list")
            continue
        for index, tf_config in enumerate(tfs):
            if not isinstance(tf_config, dict):
                errors.append(f"sensors.{sensor_name}.static_tfs[{index}] must be a mapping")
                continue
            missing = [
                field
                for field in ("name", "parent_frame", "child_frame")
                if not tf_config.get(field)
            ]
            if missing:
                errors.append(
                    f"sensors.{sensor_name}.static_tfs[{index}] missing fields: "
                    + ", ".join(missing)
                )
                continue
            static_tfs.append(tf_config)

    parent_frames = {str(tf["parent_frame"]).lstrip("/") for tf in static_tfs}
    child_frames = {str(tf["child_frame"]).lstrip("/") for tf in static_tfs}
    known_frames = set(urdf_links) | parent_frames | child_frames

    for tf_config in static_tfs:
        parent = str(tf_config["parent_frame"]).lstrip("/")
        if parent not in urdf_links and parent not in child_frames:
            errors.append(
                f"static TF '{tf_config['name']}' parent_frame '{parent}' "
                "is not in URDF links or profile static TF graph"
            )

    for frame in profile["smoke"]["tf"].get("required_frames", []):
        normalized = str(frame).lstrip("/")
        if normalized not in known_frames:
            errors.append(f"smoke required TF frame '{frame}' is not declared")


def lint_profile(args):
    errors = []
    warnings = []
    try:
        mode = load_sim_mode(args.mode)
        profile = load_sim_profile(
            args.profile,
            args.profile_file,
            require_moveit=args.require_moveit,
            include_optional_moveit=True,
        )
        sensors = _sensor_states(
            profile,
            bool(mode["sensors_default"]),
            args.sensor_overrides,
        )
    except Exception as exc:
        return {
            "ok": False,
            "profile": args.profile,
            "errors": [str(exc)],
            "warnings": [],
        }

    urdf_links = _check_xacro(profile, mode, sensors, errors, warnings)
    _check_controllers(profile, errors)
    _check_bridges(profile, errors)
    _check_receivers(profile, sensors, args.require_receivers, errors, warnings)
    _check_tf(profile, urdf_links, errors)

    return {
        "ok": not errors,
        "profile": profile["name"],
        "profile_path": profile["path"],
        "mode": mode["name"],
        "require_moveit": bool(args.require_moveit),
        "require_receivers": bool(args.require_receivers),
        "errors": errors,
        "warnings": warnings,
        "bridges": sorted(profile["bridges"]),
        "sensors": sensors,
    }


def _print_text(result):
    if result["ok"]:
        print(
            f"profile_lint OK: profile={result['profile']} "
            f"mode={result.get('mode', '')}"
        )
    else:
        print(
            f"profile_lint FAILED: profile={result.get('profile', '')} "
            f"mode={result.get('mode', '')}",
            file=sys.stderr,
        )
    for warning in result.get("warnings", []):
        print(f"WARNING: {warning}")
    for error in result.get("errors", []):
        print(f"ERROR: {error}", file=sys.stderr)


def build_parser():
    parser = argparse.ArgumentParser(description="Validate a robot_sim sim_profile")
    parser.add_argument("--profile", default="panda")
    parser.add_argument("--profile-file", default="")
    parser.add_argument("--mode", default="light", choices=("light", "full", "mock"))
    parser.add_argument("--sensor-overrides", default="")
    parser.add_argument("--require-moveit", action="store_true")
    parser.add_argument("--require-receivers", action="store_true")
    parser.add_argument("--format", choices=("text", "json"), default="text")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    result = lint_profile(args)
    if args.format == "json":
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        _print_text(result)
    return SUCCESS if result["ok"] else FAILURE


if __name__ == "__main__":
    sys.exit(main())
