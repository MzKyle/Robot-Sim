import argparse
import json
import shlex
import subprocess
import sys
import time
from collections import defaultdict, deque
from xml.etree import ElementTree as ET

import yaml

from robot_sim_bringup.sim_config_loader import load_sim_mode, load_sim_profile


SUCCESS = 0
FAILURE = 1


def _bool_value(value, name):
    normalized = str(value).strip().lower()
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off", ""):
        return False
    raise RuntimeError(f"{name} must be true or false; got '{value}'")


def _bool_text(value):
    return "true" if value else "false"


def _parse_sensor_overrides(text):
    overrides = {}
    if not text:
        return overrides
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            raise RuntimeError("sensor overrides must use name=true or name=false")
        name, value = item.split("=", 1)
        name = name.strip()
        if not name:
            raise RuntimeError("sensor overrides contains an empty sensor name")
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


def _layout(profile):
    if "single" not in profile["layouts"]:
        raise RuntimeError(f"sim_profile '{profile['name']}' does not define layout 'single'")
    return profile["layouts"]["single"]


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


def _controller_manager_path(profile, namespace):
    manager_name = profile["control"]["controller_manager_name"]
    if manager_name.startswith("/"):
        return manager_name
    return _absolute_name(namespace, manager_name)


def _enabled_spawners(profile):
    result = []
    for spawner in profile["control"]["spawners"]:
        if spawner.get("enabled_by"):
            continue
        result.append(spawner)
    return result


def _primary_trajectory_controller(profile):
    for spawner in _enabled_spawners(profile):
        if spawner.get("type") == "joint_trajectory_controller/JointTrajectoryController":
            return spawner["name"]
    raise RuntimeError("No enabled JointTrajectoryController spawner found in sim_profile")


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    return raw or {}


def _controller_parameters(profile, controller_name):
    raw = _load_yaml(profile["control"]["controllers_file"])
    for key, value in raw.items():
        if key.strip("/").split("/")[-1] == controller_name:
            if isinstance(value, dict):
                return value.get("ros__parameters", {})
    return {}


def _primary_joints(profile):
    controller_name = _primary_trajectory_controller(profile)
    params = _controller_parameters(profile, controller_name)
    joints = params.get("joints", [])
    if not joints:
        raise RuntimeError(
            f"Controller '{controller_name}' does not define ros__parameters.joints"
        )
    return [str(joint) for joint in joints]


def _bridge_config_topics(profile, bridge_name, namespace):
    bridge = profile["bridges"][bridge_name]
    bridge_namespace = _namespace_value(namespace, bridge.get("namespace", ""))
    raw = _load_yaml(bridge["config"])
    if not isinstance(raw, list):
        raise RuntimeError(f"Bridge config must be a list: {bridge['config']}")

    topics = []
    for item in raw:
        if not isinstance(item, dict):
            continue
        ros_topic = item.get("ros_topic_name")
        ros_type = item.get("ros_type_name")
        if not ros_topic or not ros_type:
            continue
        topics.append({
            "name": _absolute_name(bridge_namespace, str(ros_topic)),
            "type": str(ros_type),
            "bridge_group": bridge_name,
        })
    return topics


def _sensor_topics(profile, sensors, namespaces):
    topics = []
    seen = set()
    for sensor_name, enabled in sensors.items():
        if not enabled:
            continue
        bridge_name = profile["sensors"][sensor_name].get("bridge_group")
        if not bridge_name:
            continue
        for topic in _bridge_config_topics(profile, bridge_name, namespaces):
            key = (topic["name"], topic["type"])
            if key in seen:
                continue
            seen.add(key)
            topic["sensor_group"] = sensor_name
            topics.append(topic)
    return topics


def _load_context(args, require_moveit=False):
    mode = load_sim_mode(args.mode)
    profile = load_sim_profile(
        args.profile,
        args.profile_file,
        require_moveit=require_moveit,
    )
    layout = _layout(profile)
    namespaces = layout["namespaces"]
    sensors = _sensor_states(
        profile,
        bool(mode["sensors_default"]),
        args.sensor_overrides,
    )
    robot_namespace = namespaces.get("robot", "")
    primary_controller = _primary_trajectory_controller(profile)
    moveit_namespace = str(layout.get("moveit", {}).get("namespace", ""))
    return {
        "mode": mode,
        "profile": profile,
        "layout": layout,
        "namespaces": namespaces,
        "sensors": sensors,
        "robot_namespace": robot_namespace,
        "controller_manager": _controller_manager_path(profile, robot_namespace),
        "primary_controller": primary_controller,
        "primary_joints": _primary_joints(profile),
        "action_name": _absolute_name(
            robot_namespace,
            f"{primary_controller}/follow_joint_trajectory",
        ),
        "move_action": _absolute_name(moveit_namespace, "move_action"),
    }


def _shell_quote_list(values):
    return " ".join(str(value) for value in values)


def command_shell_env(args):
    context = _load_context(args, require_moveit=args.with_moveit)
    profile = context["profile"]
    layout = context["layout"]
    namespaces = context["namespaces"]
    robot_namespace = context["robot_namespace"]
    primary_controller = context["primary_controller"]
    moveit_namespace = ""
    if args.with_moveit:
        moveit_namespace = str(layout.get("moveit", {}).get("namespace", ""))

    sensor_topics = _sensor_topics(profile, context["sensors"], namespaces)
    active_controllers = [spawner["name"] for spawner in _enabled_spawners(profile)]

    values = {
        "SMOKE_PROFILE_NAME": profile["name"],
        "SMOKE_SPAWN_NAME": profile["robot"]["spawn_name"],
        "SMOKE_USE_GAZEBO": _bool_text(bool(context["mode"]["use_gazebo"])),
        "SMOKE_CONTROLLER_MANAGER": context["controller_manager"],
        "SMOKE_REQUIRED_CONTROLLERS": _shell_quote_list(active_controllers),
        "SMOKE_PRIMARY_CONTROLLER": primary_controller,
        "SMOKE_PRIMARY_JOINTS": _shell_quote_list(context["primary_joints"]),
        "SMOKE_ACTION_NAME": _absolute_name(
            robot_namespace,
            f"{primary_controller}/follow_joint_trajectory",
        ),
        "SMOKE_SENSOR_TOPICS": _shell_quote_list(topic["name"] for topic in sensor_topics),
        "SMOKE_MOVE_ACTION": _absolute_name(moveit_namespace, "move_action"),
    }
    for name, value in values.items():
        print(f"{name}={shlex.quote(str(value))}")
    return SUCCESS


def command_profile_json(args):
    context = _load_context(args, require_moveit=args.with_moveit)
    profile = context["profile"]
    sensor_topics = _sensor_topics(profile, context["sensors"], context["namespaces"])
    print(json.dumps({
        "profile": profile["name"],
        "spawn_name": profile["robot"]["spawn_name"],
        "mode": context["mode"]["name"],
        "use_gazebo": bool(context["mode"]["use_gazebo"]),
        "controller_manager": context["controller_manager"],
        "required_controllers": [
            spawner["name"] for spawner in _enabled_spawners(profile)
        ],
        "primary_controller": context["primary_controller"],
        "primary_joints": context["primary_joints"],
        "sensors": context["sensors"],
        "sensor_topics": sensor_topics,
    }, indent=2, sort_keys=True))
    return SUCCESS


def command_render_urdf(args):
    context = _load_context(args)
    profile = context["profile"]
    use_gazebo = bool(context["mode"]["use_gazebo"])
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
        "ros_namespace": context["robot_namespace"],
    })
    for group, enabled in context["sensors"].items():
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
    with open(args.output, "w", encoding="utf-8") as handle:
        handle.write(result.stdout)
    return SUCCESS


def _spin_until(node, timeout, predicate):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        import rclpy

        rclpy.spin_once(node, timeout_sec=0.1)
    return predicate()


def _wait_for_joint_state(node, topics, required_joints, timeout):
    from sensor_msgs.msg import JointState

    state = {"message": None, "topic": None}
    required = set(required_joints)
    subscriptions = []

    def make_callback(topic):
        def callback(msg):
            if required.issubset(set(msg.name)):
                state["message"] = msg
                state["topic"] = topic
        return callback

    for topic in topics:
        subscriptions.append(
            node.create_subscription(JointState, topic, make_callback(topic), 10)
        )

    ok = _spin_until(node, timeout, lambda: state["message"] is not None)
    for subscription in subscriptions:
        node.destroy_subscription(subscription)

    if not ok:
        raise RuntimeError(
            "Timed out waiting for joint_states containing joints: "
            + ", ".join(required_joints)
        )
    return state["message"], state["topic"]


def _joint_state_topics(robot_namespace):
    topics = ["/joint_states"]
    if robot_namespace:
        topics.append(_absolute_name(robot_namespace, "joint_states"))
    return list(dict.fromkeys(topics))


def command_wait_joint_state(args):
    import rclpy

    context = _load_context(args)
    rclpy.init(args=None)
    node = rclpy.create_node("sim_smoke_joint_state_check")
    try:
        _, topic = _wait_for_joint_state(
            node,
            _joint_state_topics(context["robot_namespace"]),
            context["primary_joints"],
            args.timeout,
        )
        print(f"joint_states OK on {topic}")
        return SUCCESS
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command_wait_controllers(args):
    import rclpy
    from controller_manager_msgs.srv import ListControllers

    context = _load_context(args)
    required = [spawner["name"] for spawner in _enabled_spawners(context["profile"])]
    service_name = _absolute_name(context["controller_manager"], "list_controllers")

    rclpy.init(args=None)
    node = rclpy.create_node("sim_smoke_controller_check")
    client = node.create_client(ListControllers, service_name)
    try:
        deadline = time.monotonic() + args.timeout
        last_state = {}
        while time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=1.0):
                continue
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            if future.done() and future.result() is not None:
                last_state = {
                    controller.name: controller.state
                    for controller in future.result().controller
                }
                if all(last_state.get(name) == "active" for name in required):
                    print("controllers active: " + ", ".join(required))
                    return SUCCESS
            rclpy.spin_once(node, timeout_sec=0.2)
        raise RuntimeError(
            "Timed out waiting for active controllers. Last state: "
            + json.dumps(last_state, sort_keys=True)
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command_send_trajectory(args):
    import rclpy
    from action_msgs.msg import GoalStatus
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from trajectory_msgs.msg import JointTrajectoryPoint

    context = _load_context(args)
    joints = context["primary_joints"]

    rclpy.init(args=None)
    node = rclpy.create_node("sim_smoke_trajectory_check")
    action_client = ActionClient(node, FollowJointTrajectory, context["action_name"])
    try:
        joint_state, _ = _wait_for_joint_state(
            node,
            _joint_state_topics(context["robot_namespace"]),
            joints,
            args.timeout,
        )
        positions_by_name = dict(zip(joint_state.name, joint_state.position))
        positions = [float(positions_by_name[joint]) for joint in joints]

        if not action_client.wait_for_server(timeout_sec=args.timeout):
            raise RuntimeError(f"Action server not available: {context['action_name']}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=max(1, int(args.duration)))
        goal.trajectory.points.append(point)

        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=args.timeout)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Trajectory goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.timeout)
        result = result_future.result()
        if result is None:
            raise RuntimeError("Timed out waiting for trajectory action result")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Trajectory action failed with status {result.status}")
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                "Trajectory controller returned error "
                f"{result.result.error_code}: {result.result.error_string}"
            )
        print(f"trajectory action OK: {context['action_name']}")
        return SUCCESS
    finally:
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()


def _sensor_qos():
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _measure_topic_hz(node, topic, type_name, timeout, min_samples):
    from rosidl_runtime_py.utilities import get_message

    msg_type = get_message(type_name)
    stamps = []

    def callback(_msg):
        stamps.append(time.monotonic())

    subscription = node.create_subscription(
        msg_type,
        topic,
        callback,
        _sensor_qos(),
    )
    try:
        _spin_until(node, timeout, lambda: len(stamps) >= min_samples)
    finally:
        node.destroy_subscription(subscription)

    if len(stamps) < 2:
        return 0.0, len(stamps)
    elapsed = stamps[-1] - stamps[0]
    if elapsed <= 0:
        return float("inf"), len(stamps)
    return (len(stamps) - 1) / elapsed, len(stamps)


def command_check_sensors(args):
    import rclpy

    context = _load_context(args)
    topics = _sensor_topics(
        context["profile"],
        context["sensors"],
        context["namespaces"],
    )
    if not topics:
        print("no enabled sensor topics; skipping sensor hz check")
        return SUCCESS

    rclpy.init(args=None)
    node = rclpy.create_node("sim_smoke_sensor_check")
    try:
        failures = []
        for topic in topics:
            hz, samples = _measure_topic_hz(
                node,
                topic["name"],
                topic["type"],
                args.topic_timeout,
                args.min_samples,
            )
            print(f"{topic['name']} [{topic['type']}]: {hz:.2f} Hz ({samples} samples)")
            if hz < args.min_hz:
                failures.append(f"{topic['name']}={hz:.2f}Hz")
        if failures:
            raise RuntimeError(
                "Sensor topic hz below threshold "
                f"{args.min_hz}: " + ", ".join(failures)
            )
        return SUCCESS
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _urdf_links(path):
    root = ET.parse(path).getroot()
    return {
        element.attrib["name"].lstrip("/")
        for element in root.findall("link")
        if element.attrib.get("name")
    }


def _required_tf_frames(context, urdf_path):
    frames = set(_urdf_links(urdf_path))
    for group, enabled in context["sensors"].items():
        if not enabled:
            continue
        for tf_config in context["profile"]["sensors"][group].get("static_tfs", []):
            frames.add(str(tf_config["parent_frame"]).lstrip("/"))
            frames.add(str(tf_config["child_frame"]).lstrip("/"))
    return frames


def command_check_tf(args):
    import rclpy
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from tf2_msgs.msg import TFMessage

    context = _load_context(args)
    required = _required_tf_frames(context, args.urdf)
    edges = set()

    def add_transforms(msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if parent and child:
                edges.add((parent, child))

    rclpy.init(args=None)
    node = rclpy.create_node("sim_smoke_tf_check")
    tf_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    static_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    tf_sub = node.create_subscription(TFMessage, "/tf", add_transforms, tf_qos)
    static_sub = node.create_subscription(TFMessage, "/tf_static", add_transforms, static_qos)
    try:
        deadline = time.monotonic() + args.wait_time
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(tf_sub)
        node.destroy_subscription(static_sub)
        node.destroy_node()
        rclpy.shutdown()

    graph = defaultdict(set)
    frames_seen = set()
    for parent, child in edges:
        graph[parent].add(child)
        graph[child].add(parent)
        frames_seen.add(parent)
        frames_seen.add(child)

    missing = sorted(required - frames_seen)
    if missing:
        raise RuntimeError("TF frames missing: " + ", ".join(missing))

    start = next(iter(required), None)
    visited = set()
    if start:
        queue = deque([start])
        while queue:
            frame = queue.popleft()
            if frame in visited:
                continue
            visited.add(frame)
            queue.extend(graph[frame] - visited)

    disconnected = sorted(required - visited)
    if disconnected:
        raise RuntimeError("TF tree disconnected frames: " + ", ".join(disconnected))
    print(f"TF tree OK: {len(required)} required frames connected")
    return SUCCESS


def _srdf_group_state(srdf_path):
    root = ET.parse(srdf_path).getroot()
    groups = [group.attrib.get("name") for group in root.findall("group")]
    groups = [group for group in groups if group]
    if not groups:
        raise RuntimeError(f"No MoveIt planning group found in SRDF: {srdf_path}")
    group_name = groups[0]
    for state in root.findall("group_state"):
        if state.attrib.get("group") != group_name:
            continue
        joints = []
        positions = []
        for joint in state.findall("joint"):
            name = joint.attrib.get("name")
            value = joint.attrib.get("value")
            if name is None or value is None:
                continue
            joints.append(name)
            positions.append(float(value))
        if joints:
            return group_name, joints, positions
    return group_name, [], []


def command_moveit(args):
    import rclpy
    from action_msgs.msg import GoalStatus
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
    from rclpy.action import ActionClient

    context = _load_context(args, require_moveit=True)
    srdf_path = context["profile"]["moveit"]["arguments"]["srdf_file"]
    group_name, goal_joints, goal_positions = _srdf_group_state(srdf_path)
    if not goal_joints:
        goal_joints = context["primary_joints"]

    rclpy.init(args=None)
    node = rclpy.create_node("sim_smoke_moveit_check")
    action_client = ActionClient(node, MoveGroup, context["move_action"])
    try:
        joint_state, _ = _wait_for_joint_state(
            node,
            _joint_state_topics(context["robot_namespace"]),
            context["primary_joints"],
            args.timeout,
        )
        current = dict(zip(joint_state.name, joint_state.position))
        if not goal_positions:
            goal_positions = [float(current[joint]) for joint in goal_joints]

        if not action_client.wait_for_server(timeout_sec=args.timeout):
            raise RuntimeError(f"MoveIt action server not available: {context['move_action']}")

        constraints = Constraints()
        constraints.name = "sim_smoke_goal"
        for joint, position in zip(goal_joints, goal_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = args.tolerance
            joint_constraint.tolerance_below = args.tolerance
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.start_state.joint_state = joint_state
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=args.timeout)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveIt goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.timeout)
        result = result_future.result()
        if result is None:
            raise RuntimeError("Timed out waiting for MoveIt result")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"MoveIt action failed with status {result.status}")
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"MoveIt returned error code {result.result.error_code.val}"
            )
        print(f"MoveIt plan/execute OK: group={group_name}")
        return SUCCESS
    finally:
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()


def _add_common_args(parser):
    parser.add_argument("--profile", default="panda")
    parser.add_argument("--profile-file", default="")
    parser.add_argument("--mode", default="full", choices=("full", "light", "mock"))
    parser.add_argument("--sensor-overrides", default="")


def build_parser():
    parser = argparse.ArgumentParser(description="robot_sim smoke test helper")
    subparsers = parser.add_subparsers(dest="command", required=True)

    shell_env = subparsers.add_parser("shell-env")
    _add_common_args(shell_env)
    shell_env.add_argument("--with-moveit", action="store_true")
    shell_env.set_defaults(func=command_shell_env)

    profile_json = subparsers.add_parser("profile-json")
    _add_common_args(profile_json)
    profile_json.add_argument("--with-moveit", action="store_true")
    profile_json.set_defaults(func=command_profile_json)

    render_urdf = subparsers.add_parser("render-urdf")
    _add_common_args(render_urdf)
    render_urdf.add_argument("--output", required=True)
    render_urdf.set_defaults(func=command_render_urdf)

    wait_joint_state = subparsers.add_parser("wait-joint-state")
    _add_common_args(wait_joint_state)
    wait_joint_state.add_argument("--timeout", type=float, default=60.0)
    wait_joint_state.set_defaults(func=command_wait_joint_state)

    wait_controllers = subparsers.add_parser("wait-controllers")
    _add_common_args(wait_controllers)
    wait_controllers.add_argument("--timeout", type=float, default=60.0)
    wait_controllers.set_defaults(func=command_wait_controllers)

    send_trajectory = subparsers.add_parser("send-trajectory")
    _add_common_args(send_trajectory)
    send_trajectory.add_argument("--timeout", type=float, default=60.0)
    send_trajectory.add_argument("--duration", type=float, default=1.0)
    send_trajectory.set_defaults(func=command_send_trajectory)

    check_sensors = subparsers.add_parser("check-sensors")
    _add_common_args(check_sensors)
    check_sensors.add_argument("--topic-timeout", type=float, default=6.0)
    check_sensors.add_argument("--min-hz", type=float, default=1.0)
    check_sensors.add_argument("--min-samples", type=int, default=3)
    check_sensors.set_defaults(func=command_check_sensors)

    check_tf = subparsers.add_parser("check-tf")
    _add_common_args(check_tf)
    check_tf.add_argument("--urdf", required=True)
    check_tf.add_argument("--wait-time", type=float, default=5.0)
    check_tf.set_defaults(func=command_check_tf)

    moveit = subparsers.add_parser("moveit")
    _add_common_args(moveit)
    moveit.add_argument("--timeout", type=float, default=90.0)
    moveit.add_argument("--tolerance", type=float, default=0.02)
    moveit.set_defaults(func=command_moveit)

    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return FAILURE


if __name__ == "__main__":
    sys.exit(main())
