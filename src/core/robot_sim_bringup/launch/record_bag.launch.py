import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction

from robot_sim_bringup.sim_config_loader import load_sim_profile

CORE_TOPICS = ["/clock", "/tf", "/tf_static"]
TRAJECTORY_CONTROLLER_TYPE = "joint_trajectory_controller/JointTrajectoryController"


def _get_config(context, name):
    return context.launch_configurations.get(name, "").strip()


def _bool_value(value, name):
    normalized = value.strip().lower()
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off", ""):
        return False
    raise RuntimeError(f"{name} must be true or false; got '{value}'")


def _split_topics(value):
    if not value:
        return []
    return [item for item in value.replace(",", " ").split() if item]


def _dedupe(topics):
    result = []
    seen = set()
    for topic in topics:
        if topic not in seen:
            seen.add(topic)
            result.append(topic)
    return result


def _namespace_value(namespaces, value):
    if not value:
        return ""
    return namespaces.get(value, value)


def _absolute_name(namespace, name):
    if not name:
        return "/"
    if name.startswith("/"):
        return name
    namespace = namespace.strip("/")
    if namespace:
        return f"/{namespace}/{name}"
    return f"/{name}"


def _parse_sensor_overrides(text):
    result = {}
    if not text:
        return result
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
        result[name] = _bool_value(value, f"sensor_overrides.{name}")
    return result


def _layout(profile, layout_name):
    if layout_name not in profile["layouts"]:
        raise RuntimeError(
            f"sim_profile '{profile['name']}' does not define layout '{layout_name}'"
        )
    return profile["layouts"][layout_name]


def _bridge_topics(profile, bridge_names, namespaces):
    topics = []
    for bridge_name in bridge_names:
        if bridge_name not in profile["bridges"]:
            raise RuntimeError(f"Unknown bridge group '{bridge_name}'")
        bridge = profile["bridges"][bridge_name]
        namespace = _namespace_value(namespaces, bridge.get("namespace", ""))
        for item in bridge.get("topics", []):
            ros_topic = item.get("ros_topic_name")
            if ros_topic:
                topics.append(_absolute_name(namespace, str(ros_topic)))
    return topics


def _core_topics(profile, namespaces):
    return [
        *CORE_TOPICS,
        *_bridge_topics(profile, profile.get("startup_bridges", []), namespaces),
    ]


def _control_topics(profile, namespaces, include_action_topics):
    robot_namespace = namespaces.get("robot", "")
    topics = ["/joint_states"]
    if robot_namespace:
        topics.append(_absolute_name(robot_namespace, "joint_states"))

    for spawner in profile["control"]["spawners"]:
        if spawner.get("type") != TRAJECTORY_CONTROLLER_TYPE:
            continue
        controller = _absolute_name(robot_namespace, spawner["name"])
        topics.extend([
            f"{controller}/controller_state",
            f"{controller}/joint_trajectory",
        ])
        if include_action_topics:
            action = f"{controller}/follow_joint_trajectory/_action"
            topics.extend([f"{action}/feedback", f"{action}/status"])
    return topics


def _sensor_topics(profile, namespaces, sensor_overrides):
    sensor_states = {
        name: True
        for name in profile["sensors"]
    }
    for name, enabled in _parse_sensor_overrides(sensor_overrides).items():
        if name not in sensor_states:
            raise RuntimeError(
                f"sensor_overrides references unknown sensor group '{name}'"
            )
        sensor_states[name] = enabled

    bridge_names = []
    for sensor_name, enabled in sensor_states.items():
        if not enabled:
            continue
        bridge_name = profile["sensors"][sensor_name].get("bridge_group")
        if bridge_name:
            bridge_names.append(bridge_name)
    return _bridge_topics(profile, _dedupe(bridge_names), namespaces)


def _topic_group_topics(
    profile,
    layout_name,
    topic_group,
    include_action_topics,
    sensor_overrides,
):
    layout = _layout(profile, layout_name)
    namespaces = layout["namespaces"]
    if topic_group == "control":
        return [
            *_core_topics(profile, namespaces),
            *_control_topics(profile, namespaces, include_action_topics),
        ]
    if topic_group == "sensors":
        return [
            *_core_topics(profile, namespaces),
            *_sensor_topics(profile, namespaces, sensor_overrides),
        ]
    if topic_group == "all":
        return [
            *_core_topics(profile, namespaces),
            *_control_topics(profile, namespaces, include_action_topics),
            *_sensor_topics(profile, namespaces, sensor_overrides),
        ]
    if topic_group == "distributed":
        return [
            *_core_topics(profile, namespaces),
            *_control_topics(profile, namespaces, include_action_topics),
            *_sensor_topics(profile, namespaces, sensor_overrides),
        ]
    if topic_group == "custom":
        return []
    raise RuntimeError(
        "topic_group must be control, sensors, all, distributed, or custom; "
        f"got '{topic_group}'"
    )


def _launch_setup(context, *args, **kwargs):
    topic_group = _get_config(context, "topic_group").lower()
    layout_name = _get_config(context, "layout") or "auto"
    if layout_name == "auto":
        layout_name = "distributed" if topic_group == "distributed" else "single"
    requested_hidden_topics = _bool_value(
        _get_config(context, "include_hidden_topics"), "include_hidden_topics"
    )
    include_action_topics = _bool_value(
        _get_config(context, "include_action_topics"),
        "include_action_topics",
    )
    compression = _bool_value(_get_config(context, "compression"), "compression")

    profile = load_sim_profile(
        _get_config(context, "sim_profile"),
        _get_config(context, "sim_profile_file"),
    )

    topics = _topic_group_topics(
        profile,
        layout_name,
        topic_group,
        include_action_topics,
        _get_config(context, "sensor_overrides"),
    )
    topics.extend(_split_topics(_get_config(context, "extra_topics")))
    topics = _dedupe(topics)
    if not topics:
        raise RuntimeError(
            "No topics selected. Use topic_group:=control|sensors|all|distributed "
            "or pass extra_topics:='/topic_a /topic_b'."
        )

    output_dir = os.path.expanduser(_get_config(context, "output_dir"))
    if not output_dir:
        output_dir = os.path.join(os.environ.get("HOME", "."), "robot_sim_bags")
    os.makedirs(output_dir, exist_ok=True)

    bag_name = _get_config(context, "bag_name")
    if not bag_name or bag_name.lower() == "auto":
        bag_name = f"robot_sim_{topic_group}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    output_path = bag_name if os.path.isabs(bag_name) else os.path.join(output_dir, bag_name)

    cmd = [
        "ros2",
        "bag",
        "record",
        "--storage",
        _get_config(context, "storage_id") or "sqlite3",
        "-o",
        output_path,
    ]

    max_bag_size = _get_config(context, "max_bag_size")
    if max_bag_size and max_bag_size != "0":
        cmd.extend(["--max-bag-size", max_bag_size])

    max_bag_duration = _get_config(context, "max_bag_duration")
    if max_bag_duration and max_bag_duration != "0":
        cmd.extend(["--max-bag-duration", max_bag_duration])

    if compression:
        cmd.extend([
            "--compression-mode",
            _get_config(context, "compression_mode") or "file",
            "--compression-format",
            _get_config(context, "compression_format") or "zstd",
        ])

    action_topics_selected = include_action_topics and topic_group in (
        "control",
        "all",
        "distributed",
    )
    include_hidden_topics = requested_hidden_topics or action_topics_selected
    if include_hidden_topics:
        cmd.append("--include-hidden-topics")

    cmd.extend(topics)

    return [
        LogInfo(msg=f"Recording rosbag2 to: {output_path}"),
        LogInfo(
            msg=(
                f"Profile: {profile['name']}; layout: {layout_name}; "
                f"topic group: {topic_group}; topics: {' '.join(topics)}"
            )
        ),
        ExecuteProcess(cmd=cmd, output="screen", emulate_tty=True),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "topic_group",
            default_value="all",
            description="Topic preset: control, sensors, all, distributed, or custom.",
        ),
        DeclareLaunchArgument("sim_profile", default_value="panda"),
        DeclareLaunchArgument("sim_profile_file", default_value=""),
        DeclareLaunchArgument(
            "layout",
            default_value="auto",
            description="Profile layout used for namespaces. auto selects distributed for topic_group:=distributed, otherwise single.",
        ),
        DeclareLaunchArgument("sensor_overrides", default_value=""),
        DeclareLaunchArgument(
            "extra_topics",
            default_value="",
            description="Additional topics separated by spaces or commas.",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="~/robot_sim_bags",
            description="Directory for relative bag names.",
        ),
        DeclareLaunchArgument(
            "bag_name",
            default_value="auto",
            description="Bag directory name. Use auto for timestamped names.",
        ),
        DeclareLaunchArgument("storage_id", default_value="sqlite3"),
        DeclareLaunchArgument(
            "include_hidden_topics",
            default_value="false",
            description="Pass --include-hidden-topics to ros2 bag record.",
        ),
        DeclareLaunchArgument(
            "include_action_topics",
            default_value="true",
            description="Include FollowJointTrajectory feedback/status topics.",
        ),
        DeclareLaunchArgument(
            "compression",
            default_value="false",
            description="Enable rosbag2 compression.",
        ),
        DeclareLaunchArgument("compression_mode", default_value="file"),
        DeclareLaunchArgument("compression_format", default_value="zstd"),
        DeclareLaunchArgument(
            "max_bag_size",
            default_value="0",
            description="Maximum bag size in bytes. 0 disables splitting by size.",
        ),
        DeclareLaunchArgument(
            "max_bag_duration",
            default_value="0",
            description="Maximum bag duration in seconds. 0 disables splitting by time.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
