from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace

from robot_sim_bringup.robot_domain.sim_config_loader import load_sim_profile


RECEIVER_TOPIC_REQUIREMENTS = {
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
}


def _config(context, name):
    return context.launch_configurations.get(name, "").strip()


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


def _sensor_states(profile, overrides_text):
    sensors = {
        name: bool(sensor.get("default_enabled", True))
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


def _layout(profile, layout_name):
    if layout_name not in profile["layouts"]:
        raise RuntimeError(
            f"sim_profile '{profile['name']}' does not define layout '{layout_name}'"
        )
    return profile["layouts"][layout_name]


def _namespace_value(namespaces, value):
    if not value:
        return ""
    return namespaces.get(value, value)


def _namespace_group(namespace, actions):
    if namespace:
        return GroupAction([PushRosNamespace(namespace), *actions])
    if len(actions) == 1:
        return actions[0]
    return GroupAction(actions)


def _absolute_topic(namespace, topic):
    if topic.startswith("/"):
        return topic
    topic = topic.strip("/")
    namespace = namespace.strip("/")
    if namespace:
        return f"/{namespace}/{topic}"
    return f"/{topic}"


def _sensor_bridge_topics(profile, sensor, namespaces):
    bridge_group = sensor.get("bridge_group")
    if not bridge_group:
        return []
    if bridge_group not in profile["bridges"]:
        raise RuntimeError(
            f"sensor receiver references missing bridge group '{bridge_group}'"
        )

    bridge = profile["bridges"][bridge_group]
    bridge_namespace = _namespace_value(namespaces, bridge.get("namespace", ""))
    topics = []
    for topic in bridge.get("topics", []):
        resolved = dict(topic)
        resolved["resolved_ros_topic_name"] = _absolute_topic(
            bridge_namespace,
            topic["ros_topic_name"],
        )
        topics.append(resolved)
    return topics


def _topic_for_type(sensor_name, topics, parameter_name, ros_type):
    for topic in topics:
        if topic.get("ros_type_name") == ros_type:
            return topic["resolved_ros_topic_name"]
    raise RuntimeError(
        f"sensor '{sensor_name}' receiver parameter '{parameter_name}' "
        f"requires bridge topic with ros_type_name '{ros_type}'"
    )


def _receiver_parameters(sensor_name, sensor, topics):
    receiver = sensor["receiver"]
    parameters = {
        "use_sim_time": True,
        "sensor_name": sensor_name,
        "receiver_type": receiver["type"],
        "expected_min_hz": receiver["expected_min_hz"],
        "log_period_sec": receiver["log_period_sec"],
        "bridge_group": sensor.get("bridge_group") or "",
        "topics": [topic["resolved_ros_topic_name"] for topic in topics],
        "topic_types": [topic["ros_type_name"] for topic in topics],
    }

    requirements = RECEIVER_TOPIC_REQUIREMENTS.get(receiver["type"], {})
    for parameter_name, ros_type in requirements.items():
        parameters[parameter_name] = _topic_for_type(
            sensor_name,
            topics,
            parameter_name,
            ros_type,
        )

    parameters.update(receiver.get("parameters", {}))
    return parameters


def _receiver_nodes(profile, layout, sensors):
    namespaces = layout["namespaces"]
    nodes = []
    enabled_names = []
    for sensor_name, enabled in sensors.items():
        if not enabled:
            continue
        sensor = profile["sensors"][sensor_name]
        receiver = sensor.get("receiver")
        if receiver is None:
            raise RuntimeError(f"enabled sensor '{sensor_name}' is missing receiver config")

        topics = _sensor_bridge_topics(profile, sensor, namespaces)
        parameters = _receiver_parameters(sensor_name, sensor, topics)
        namespace = _namespace_value(namespaces, receiver.get("namespace", ""))
        node = Node(
            package=receiver["package"],
            executable=receiver["executable"],
            name=receiver["node_name"],
            output=receiver["output"],
            parameters=[parameters],
        )
        nodes.append(_namespace_group(namespace, [node]))
        enabled_names.append(sensor_name)
    return nodes, enabled_names


def _launch_setup(context, *args, **kwargs):
    profile = load_sim_profile(
        _config(context, "sim_profile"),
        _config(context, "sim_profile_file"),
        include_optional_moveit=True,
    )
    layout = _layout(profile, _config(context, "layout") or "single")
    sensors = _sensor_states(profile, _config(context, "sensor_overrides"))
    nodes, enabled_names = _receiver_nodes(profile, layout, sensors)
    return [
        LogInfo(
            msg=(
                "Starting sensor receivers: "
                f"profile={profile['name']}, layout={_config(context, 'layout') or 'single'}, "
                f"sensors={','.join(enabled_names) or 'none'}"
            )
        ),
        *nodes,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim_profile", default_value="panda"),
        DeclareLaunchArgument("sim_profile_file", default_value=""),
        DeclareLaunchArgument("layout", default_value="single"),
        DeclareLaunchArgument("sensor_overrides", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
