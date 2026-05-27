import os

from launch.actions import (
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

from robot_sim_bringup.sim_config_loader import load_sim_mode, load_sim_profile


def build_sim_launch_actions(context, layout_name):
    mode = load_sim_mode(_config(context, "sim_mode"))

    use_sim_time = _auto_bool(context, "use_sim_time", bool(mode["use_sim_time"]))
    use_moveit = _auto_bool(context, "use_moveit", bool(mode["use_moveit"]))
    use_rviz = _auto_bool(context, "rviz", bool(mode["rviz"]))
    headless = _auto_bool(context, "headless", bool(mode["headless"]))
    use_gripper = _bool_value(_normalized(context, "use_gripper"), "use_gripper")
    rqt_graph = _bool_value(_normalized(context, "rqt_graph"), "rqt_graph")

    profile = load_sim_profile(
        _config(context, "sim_profile"),
        _config(context, "sim_profile_file"),
        require_moveit=use_moveit,
    )
    layout = _layout(profile, layout_name)
    namespaces = layout["namespaces"]
    use_gazebo = bool(mode["use_gazebo"])
    sensors = _sensor_states(
        profile,
        bool(mode["sensors_default"]),
        _config(context, "sensor_overrides"),
    )

    controllers_yaml = profile["control"]["controllers_file"]
    hardware_plugin = (
        profile["control"]["hardware_plugins"]["gazebo"]
        if use_gazebo
        else profile["control"]["hardware_plugins"]["mock"]
    )
    robot_namespace = namespaces.get("robot", "")
    robot_description = _robot_description(
        profile,
        hardware_plugin,
        controllers_yaml,
        use_gazebo,
        sensors,
        robot_namespace,
    )

    robot_stack_actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }],
        )
    ]
    if use_gazebo:
        robot_stack_actions.extend(_sensor_frame_tf_nodes(profile, sensors))
    else:
        robot_stack_actions.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": robot_description,
                    },
                    controllers_yaml,
                ],
            )
        )

    actions = [
        LogInfo(msg=_log_message(profile, mode, layout_name, use_sim_time, sensors)),
        *_resource_path_actions(profile),
        _namespace_group(robot_namespace, robot_stack_actions),
    ]

    if use_gazebo:
        world_key = layout["world"]
        if world_key not in profile["worlds"]:
            raise RuntimeError(
                f"layout '{layout_name}' references missing world '{world_key}'"
            )
        robot_description_topic = layout.get("robot_description_topic")
        if not robot_description_topic:
            robot_description_topic = _absolute_topic(robot_namespace, "robot_description")

        actions.extend([
            TimerAction(
                period=mode["delays"]["gazebo"],
                actions=[_gazebo_launch(profile, profile["worlds"][world_key], headless)],
            ),
            TimerAction(
                period=mode["delays"]["spawn"],
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        name=profile["robot"]["spawn_node_name"],
                        output="screen",
                        arguments=_spawn_arguments(profile, robot_description_topic),
                    )
                ],
            ),
        ])

        bridge_nodes = _bridge_nodes(profile, sensors, namespaces)
        if bridge_nodes:
            actions.append(
                TimerAction(period=mode["delays"]["bridge"], actions=bridge_nodes)
            )

    controller_manager_name = _controller_manager_path(profile, robot_namespace)
    actions.append(
        TimerAction(
            period=mode["delays"]["controller"],
            actions=_spawner_nodes(
                profile,
                controller_manager_name,
                controllers_yaml,
                {"use_gripper": use_gripper},
            ),
        )
    )

    if use_moveit:
        actions.append(
            TimerAction(
                period=mode["delays"]["moveit"],
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(profile["moveit"]["launch"]),
                        launch_arguments=_moveit_launch_arguments(
                            profile,
                            layout,
                            use_sim_time,
                            use_rviz,
                        ).items(),
                    )
                ],
            )
        )

    if rqt_graph and layout.get("rqt_graph"):
        rqt_config = layout["rqt_graph"]
        actions.append(
            Node(
                package=rqt_config.get("package", "rqt_graph"),
                executable=rqt_config.get("executable", "rqt_graph"),
                name=rqt_config.get("node_name", "graph"),
                namespace=_namespace_value(namespaces, rqt_config.get("namespace", "")),
            )
        )

    return actions


def _config(context, name):
    return context.launch_configurations.get(name, "").strip()


def _normalized(context, name):
    return _config(context, name).lower()


def _bool_value(value, name):
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off", ""):
        return False
    raise RuntimeError(f"{name} must be true or false; got '{value}'")


def _auto_bool(context, name, default):
    value = _normalized(context, name)
    if value == "auto":
        return default
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False
    raise RuntimeError(f"{name} must be auto, true, or false; got '{value}'")


def _bool_text(value):
    return "true" if value else "false"


def _layout(profile, layout_name):
    if layout_name not in profile["layouts"]:
        raise RuntimeError(
            f"sim_profile '{profile['name']}' does not define layout '{layout_name}'"
        )
    return profile["layouts"][layout_name]


def _sensor_states(profile, mode_default, overrides_text):
    sensors = {}
    for name, sensor in profile["sensors"].items():
        sensors[name] = bool(mode_default and sensor.get("default_enabled", True))

    for name, enabled in _parse_sensor_overrides(overrides_text).items():
        if name not in profile["sensors"]:
            raise RuntimeError(
                f"sensor_overrides references unknown sensor group '{name}' "
                f"for sim_profile '{profile['name']}'"
            )
        sensors[name] = enabled
    return sensors


def _parse_sensor_overrides(text):
    overrides = {}
    if not text:
        return overrides

    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            raise RuntimeError(
                "sensor_overrides entries must use name=true or name=false"
            )
        name, value = item.split("=", 1)
        name = name.strip()
        if not name:
            raise RuntimeError("sensor_overrides contains an empty sensor name")
        overrides[name] = _bool_value(value.strip().lower(), f"sensor_overrides.{name}")
    return overrides


def _robot_description(
    profile,
    hardware_plugin,
    controllers_yaml,
    use_gazebo,
    sensors,
    ros_namespace,
):
    xacro_args = dict(profile["robot"].get("xacro_args", {}))
    xacro_args.update({
        "hardware_plugin": hardware_plugin,
        "controllers_file": controllers_yaml,
        "controller_manager_name": profile["control"]["controller_manager_name"],
        "use_gz_ros2_control": _bool_text(use_gazebo),
        "ros_namespace": ros_namespace,
    })
    for group, enabled in sensors.items():
        sensor = profile["sensors"][group]
        if sensor.get("xacro_arg"):
            xacro_args[sensor["xacro_arg"]] = _bool_text(enabled)

    command = [FindExecutable(name="xacro"), " ", profile["robot"]["xacro"]]
    for name, value in xacro_args.items():
        command.extend([" ", f"{name}:=", str(value)])
    return ParameterValue(Command(command), value_type=str)


def _vector3(value, default):
    if value is None:
        return default
    if isinstance(value, str):
        parts = value.split()
    else:
        parts = list(value)
    if len(parts) != 3:
        raise RuntimeError(f"Expected 3 values, got {value}")
    return [str(part) for part in parts]


def _sensor_frame_tf_node(tf_config):
    xyz = _vector3(tf_config.get("xyz"), ["0", "0", "0"])
    rpy = _vector3(tf_config.get("rpy"), ["0", "0", "0"])
    name = tf_config["name"]
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{name}_frame_tf",
        output="log",
        arguments=[
            "--x", xyz[0], "--y", xyz[1], "--z", xyz[2],
            "--roll", rpy[0], "--pitch", rpy[1], "--yaw", rpy[2],
            "--frame-id", tf_config["parent_frame"],
            "--child-frame-id", tf_config["child_frame"],
        ],
    )


def _sensor_frame_tf_nodes(profile, sensors):
    nodes = []
    for group, enabled in sensors.items():
        if not enabled:
            continue
        for tf_config in profile["sensors"][group].get("static_tfs", []):
            nodes.append(_sensor_frame_tf_node(tf_config))
    return nodes


def _bridge_nodes(profile, sensors, namespaces):
    bridge_names = set(profile.get("startup_bridges", []))
    for group, enabled in sensors.items():
        if not enabled:
            continue
        bridge_group = profile["sensors"][group].get("bridge_group")
        if not bridge_group:
            continue
        if bridge_group not in profile["bridges"]:
            raise RuntimeError(
                f"sensor group '{group}' references missing bridge group '{bridge_group}'"
            )
        bridge_names.add(bridge_group)

    nodes = []
    for name in sorted(bridge_names):
        bridge = profile["bridges"][name]
        node = Node(
            package=bridge["package"],
            executable=bridge["executable"],
            name=bridge["node_name"],
            output=bridge["output"],
            parameters=[{"config_file": bridge["config"]}],
        )
        namespace = _namespace_value(namespaces, bridge.get("namespace", ""))
        nodes.append(_namespace_group(namespace, [node]))
    return nodes


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


def _resource_path_actions(profile):
    paths = profile["gazebo"].get("resource_paths", [])
    env_vars = profile["gazebo"].get("resource_env_vars", [])
    if not paths or not env_vars:
        return []

    def env_value(env_name):
        value = []
        for path in paths:
            value.extend([path, os.pathsep])
        value.append(os.environ.get(env_name, ""))
        return value

    return [
        SetEnvironmentVariable(env_name, env_value(env_name))
        for env_name in env_vars
    ]


def _gazebo_launch(profile, world, headless):
    gazebo = profile["gazebo"]
    gz_mode_args = gazebo["args"]["headless"] if headless else gazebo["args"]["gui"]
    world_path = world["path"] if isinstance(world, dict) else world
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo["launch"]),
        launch_arguments={
            "gz_args": [gz_mode_args, world_path],
            "gz_version": gazebo["gz_version"],
            "on_exit_shutdown": gazebo["on_exit_shutdown"],
        }.items(),
    )


def _controller_manager_path(profile, namespace):
    manager_name = profile["control"]["controller_manager_name"]
    if manager_name.startswith("/"):
        return manager_name
    if namespace:
        return f"/{namespace}/{manager_name}"
    return f"/{manager_name}"


def _spawner_nodes(profile, controller_manager_name, controllers_yaml, features):
    nodes = []
    for spawner in profile["control"]["spawners"]:
        enabled_by = spawner.get("enabled_by")
        if enabled_by and enabled_by not in features:
            raise RuntimeError(f"Unsupported controller spawner gate: {enabled_by}")
        if enabled_by and not features[enabled_by]:
            continue

        arguments = [
            spawner["name"],
            "-c",
            controller_manager_name,
            "-p",
            controllers_yaml,
        ]
        if spawner.get("type"):
            arguments.extend(["-t", spawner["type"]])
        arguments.extend([
            "--controller-manager-timeout",
            str(spawner.get("timeout", 90)),
        ])
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=arguments,
            )
        )
    return nodes


def _spawn_arguments(profile, robot_description_topic):
    robot = profile["robot"]
    arguments = [
        "-name",
        robot["spawn_name"],
        "-topic",
        robot_description_topic,
        "-allow_renaming",
        _bool_text(robot.get("allow_renaming", False)),
    ]
    pose = robot.get("spawn_pose") or {}
    pose_args = {
        "x": "-x",
        "y": "-y",
        "z": "-z",
        "roll": "-R",
        "pitch": "-P",
        "yaw": "-Y",
    }
    for key, flag in pose_args.items():
        if key in pose:
            arguments.extend([flag, str(pose[key])])
    return arguments


def _moveit_launch_arguments(profile, layout, use_sim_time, use_rviz):
    arguments = dict(profile["moveit"]["arguments"])
    arguments.update({
        "use_sim_time": _bool_text(use_sim_time),
        "rviz": _bool_text(use_rviz),
    })
    arguments.update(layout.get("moveit", {}))
    return {name: str(value) for name, value in arguments.items()}


def _absolute_topic(namespace, topic):
    if namespace:
        return f"/{namespace}/{topic}"
    return f"/{topic}"


def _log_message(profile, mode, layout_name, use_sim_time, sensors):
    sensor_text = ",".join(
        f"{name}={_bool_text(enabled)}" for name, enabled in sorted(sensors.items())
    )
    layout = profile["layouts"].get(layout_name, {})
    world = profile["worlds"].get(layout.get("world", ""), {})
    world_text = world.get("name", layout.get("world", "")) if isinstance(world, dict) else world
    return (
        "robot_sim_bringup: "
        f"sim_profile={profile['name']}, layout={layout_name}, "
        f"sim_mode={mode['name']}, gazebo={_bool_text(bool(mode['use_gazebo']))}, "
        f"use_sim_time={_bool_text(use_sim_time)}, world={world_text}, sensors={sensor_text}"
    )
