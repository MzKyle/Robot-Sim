import argparse
import json
import math
import random
import shlex
import subprocess
import sys
import time
from collections import defaultdict, deque
from xml.etree import ElementTree as ET

import yaml

from robot_sim_bringup.sim_config_loader import load_sim_mode, load_sim_profile
from robot_sim_bringup.validation_cases import (
    collision_primitives_from_scene,
    load_validation_case,
    min_clearance_to_primitives,
    quaternion_from_rpy,
)


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
    required = profile.get("smoke", {}).get("controllers", {}).get("required")
    if required is not None:
        spawners_by_name = {
            spawner["name"]: spawner
            for spawner in profile["control"]["spawners"]
        }
        missing = [name for name in required if name not in spawners_by_name]
        if missing:
            raise RuntimeError(
                "smoke.controllers.required references unknown spawners: "
                + ", ".join(missing)
            )
        return [
            spawners_by_name[name]
            for name in required
        ]

    result = []
    for spawner in profile["control"]["spawners"]:
        if spawner.get("enabled_by"):
            continue
        result.append(spawner)
    return result


def _primary_trajectory_controller(profile):
    primary = profile.get("smoke", {}).get("controllers", {}).get("primary_trajectory")
    if primary:
        return primary

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

    topics = []
    for item in bridge.get("topics", []):
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
        "smoke": profile.get("smoke", {}),
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
    smoke_sensors = context["profile"].get("smoke", {}).get("sensors", {})
    topic_timeout = (
        args.topic_timeout
        if args.topic_timeout is not None
        else float(smoke_sensors.get("topic_timeout", 6.0))
    )
    min_hz = (
        args.min_hz
        if args.min_hz is not None
        else float(smoke_sensors.get("min_hz", 1.0))
    )
    min_samples = (
        args.min_samples
        if args.min_samples is not None
        else int(smoke_sensors.get("min_samples", 3))
    )
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
                topic_timeout,
                min_samples,
            )
            print(f"{topic['name']} [{topic['type']}]: {hz:.2f} Hz ({samples} samples)")
            if hz < min_hz:
                failures.append(f"{topic['name']}={hz:.2f}Hz")
        if failures:
            raise RuntimeError(
                "Sensor topic hz below threshold "
                f"{min_hz}: " + ", ".join(failures)
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
    for frame in context["profile"].get("smoke", {}).get("tf", {}).get("required_frames", []):
        frames.add(str(frame).lstrip("/"))
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


def command_validation_case_env(args):
    case = load_validation_case(args.validation_case)
    values = {
        "VALIDATION_CASE_NAME": case["name"],
        "VALIDATION_CASE_PROFILE": case["profile"],
        "VALIDATION_CASE_PROFILE_FILE": case.get("profile_file", ""),
        "VALIDATION_CASE_MODE": case["mode"],
        "VALIDATION_CASE_LAYOUT": case.get("layout", "single"),
        "VALIDATION_CASE_TIMEOUT": case.get("timeout_sec", 120.0),
        "VALIDATION_CASE_SENSOR_OVERRIDES": case["sensor_overrides"],
    }
    for name, value in values.items():
        print(f"{name}={shlex.quote(str(value))}")
    return SUCCESS


def command_validation_case_json(args):
    case = load_validation_case(args.validation_case)
    print(json.dumps(_case_summary(case), indent=2, sort_keys=True))
    return SUCCESS


def command_validate_case(args):
    metrics = {
        "case_name": str(args.validation_case),
        "passed": False,
    }
    try:
        metrics = _run_validation_case(args)
    except Exception as exc:
        metrics["error"] = str(exc)
        _write_metrics(args.metrics_output, metrics)
        raise

    _write_metrics(args.metrics_output, metrics)
    if not metrics.get("passed", False):
        raise RuntimeError(
            "Validation case failed: "
            + "; ".join(str(item) for item in metrics.get("failures", []))
        )
    print(f"validation case OK: {metrics['case_name']}")
    print(f"metrics: {args.metrics_output}")
    return SUCCESS


def _case_summary(case):
    return {
        "name": case["name"],
        "path": case["path"],
        "profile": case["profile"],
        "profile_file": case.get("profile_file", ""),
        "mode": case["mode"],
        "layout": case.get("layout", "single"),
        "timeout_sec": case.get("timeout_sec", 120.0),
        "scene": case["scene"].name,
        "scene_path": str(case["scene"].path),
        "seed": case["seed"],
        "sensor_overrides": case["sensor_overrides"],
        "moveit": case["moveit"],
        "start_region": case["start_region"],
        "goal_region": case["goal_region"],
        "planning_scene": case["planning_scene"],
        "pass_criteria": case["pass_criteria"],
        "expected_topics": case.get("expected_topics", []),
        "artifacts": case.get("artifacts", {}),
    }


def _run_validation_case(args):
    import rclpy
    from control_msgs.msg import JointTrajectoryControllerState
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import MoveItErrorCodes
    from rclpy.action import ActionClient
    from tf2_ros import Buffer, TransformListener

    case = load_validation_case(args.validation_case)
    if args.profile != case["profile"] and not args.profile_file:
        raise RuntimeError(
            f"validation case '{case['name']}' expects profile '{case['profile']}', "
            f"got '{args.profile}'"
        )
    if args.mode != case["mode"]:
        raise RuntimeError(
            f"validation case '{case['name']}' expects mode '{case['mode']}', "
            f"got '{args.mode}'"
        )

    context = _load_context(args, require_moveit=True)
    criteria = case["pass_criteria"]
    rng = random.Random(case["seed"])
    start_pose = case["scene"].sample_region(case["start_region"], rng=rng)
    goal_pose = case["scene"].sample_region(case["goal_region"], rng=rng)
    collision_primitives = collision_primitives_from_scene(
        case["scene"],
        case["planning_scene"],
    )

    rclpy.init(args=None)
    node = rclpy.create_node("sim_validation_case_runner")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    action_client = ActionClient(node, MoveGroup, context["move_action"])
    controller_state = {
        "peak_error": None,
        "latest_error": None,
        "settled_errors": None,
    }
    tcp_points = []

    def controller_state_callback(msg):
        errors = list(getattr(msg.error, "positions", []))
        if not errors:
            return
        value = max(abs(float(error)) for error in errors)
        controller_state["latest_error"] = value
        current = controller_state["peak_error"]
        controller_state["peak_error"] = value if current is None else max(current, value)
        if controller_state["settled_errors"] is not None:
            controller_state["settled_errors"].append(value)

    controller_state_topic = _absolute_name(
        context["robot_namespace"],
        f"{context['primary_controller']}/controller_state",
    )
    controller_sub = node.create_subscription(
        JointTrajectoryControllerState,
        controller_state_topic,
        controller_state_callback,
        10,
    )

    try:
        if not action_client.wait_for_server(timeout_sec=args.timeout):
            raise RuntimeError(f"MoveIt action server not available: {context['move_action']}")

        applied_ids = []
        if case["planning_scene"]["apply"]:
            applied_ids = _apply_scene_collision_objects(
                node,
                case,
                collision_primitives,
                args.timeout,
            )

        start_result = _execute_pose_goal(
            node,
            action_client,
            context,
            case,
            start_pose,
            args.timeout,
            tf_buffer,
            tcp_points,
            "start",
        )
        goal_result = _execute_pose_goal(
            node,
            action_client,
            context,
            case,
            goal_pose,
            args.timeout,
            tf_buffer,
            tcp_points,
            "goal",
        )
        settled_controller_error = _settled_controller_error(node, controller_state)

        sensor_hz = _validation_sensor_hz(node, context, criteria)
        final_point = _lookup_tf_point(
            tf_buffer,
            case["moveit"]["frame"],
            case["moveit"]["target_link"],
        )
        tf_ok = final_point is not None
        if final_point is not None:
            tcp_points.append(final_point)

        clearances = [
            min_clearance_to_primitives(point, collision_primitives)
            for point in tcp_points
        ]
        clearances = [value for value in clearances if value is not None]
        min_clearance = min(clearances) if clearances else None
        goal_error = (
            _distance(final_point, goal_pose[:3])
            if final_point is not None
            else None
        )

        metrics = {
            "case_name": case["name"],
            "profile": context["profile"]["name"],
            "scene": case["scene"].name,
            "seed": case["seed"],
            "planning_scene": {
                "applied_collision_objects": len(applied_ids),
                "collision_object_ids": applied_ids,
            },
            "start": start_result,
            "goal": goal_result,
            "moveit_error_code": goal_result.get("moveit_error_code"),
            "plan_success_rate": (
                sum(1 for result in (start_result, goal_result) if result.get("success")) / 2.0
            ),
            "planning_time_sec": (
                float(start_result.get("planning_time_sec") or 0.0)
                + float(goal_result.get("planning_time_sec") or 0.0)
            ),
            "execution_time_sec": (
                float(start_result.get("execution_time_sec") or 0.0)
                + float(goal_result.get("execution_time_sec") or 0.0)
            ),
            "goal_position_error_m": goal_error,
            "min_tcp_clearance_m": min_clearance,
            "max_controller_error_rad": settled_controller_error,
            "peak_controller_error_rad": controller_state["peak_error"],
            "sensor_hz": sensor_hz,
            "expected_topics": _expected_topic_metrics(case, sensor_hz, criteria),
            "tf_ok": tf_ok,
            "passed": False,
            "failures": [],
        }
        metrics["passed"] = _validation_passed(metrics, criteria, MoveItErrorCodes.SUCCESS)
        return metrics
    finally:
        node.destroy_subscription(controller_sub)
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        del tf_listener


def _apply_scene_collision_objects(node, case, collision_primitives, timeout):
    import rclpy
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene

    service_name = _absolute_name(
        str(case["moveit"].get("namespace", "")),
        "apply_planning_scene",
    )
    client = node.create_client(ApplyPlanningScene, service_name)
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"ApplyPlanningScene service not available: {service_name}")

    planning_scene = PlanningScene()
    planning_scene.is_diff = True
    planning_scene.world.collision_objects = [
        _collision_object_message(case["moveit"]["frame"], primitive)
        for primitive in collision_primitives
    ]
    request = ApplyPlanningScene.Request()
    request.scene = planning_scene
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError("Timed out applying validation planning scene")
    if not future.result().success:
        raise RuntimeError("MoveIt rejected validation planning scene")
    return [primitive["id"] for primitive in collision_primitives]


def _collision_object_message(frame_id, primitive_info):
    from moveit_msgs.msg import CollisionObject

    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = primitive_info["id"]
    obj.primitives.append(_solid_primitive_message(primitive_info["geometry"]))
    obj.primitive_poses.append(_pose_message(primitive_info["pose"]))
    obj.operation = CollisionObject.ADD
    return obj


def _solid_primitive_message(geometry):
    from shape_msgs.msg import SolidPrimitive

    primitive = SolidPrimitive()
    if geometry["type"] == "box":
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in geometry["size"]]
        return primitive
    if geometry["type"] == "cylinder":
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [float(geometry["length"]), float(geometry["radius"])]
        return primitive
    raise RuntimeError(f"Unsupported MoveIt collision primitive: {geometry['type']}")


def _execute_pose_goal(
    node,
    action_client,
    context,
    case,
    target_pose,
    timeout,
    tf_buffer,
    tcp_points,
    phase,
):
    import rclpy
    from action_msgs.msg import GoalStatus
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import MoveItErrorCodes

    joint_state, _ = _wait_for_joint_state(
        node,
        _joint_state_topics(context["robot_namespace"]),
        context["primary_joints"],
        timeout,
    )
    goal = MoveGroup.Goal()
    goal.request.group_name = case["moveit"]["group"]
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = case["moveit"]["planning_time_sec"]
    goal.request.max_velocity_scaling_factor = case["moveit"]["velocity_scaling"]
    goal.request.max_acceleration_scaling_factor = case["moveit"]["acceleration_scaling"]
    goal.request.start_state.joint_state = joint_state
    goal.request.start_state.is_diff = True
    goal.request.goal_constraints.append(_pose_constraints(case, target_pose, phase))
    workspace = context["profile"]["worlds"][context["layout"]["world"]].get("scene", {}).get("workspace")
    if workspace:
        goal.request.workspace_parameters.header.frame_id = workspace["frame"]
        goal.request.workspace_parameters.min_corner.x = float(workspace["bounds"]["min"][0])
        goal.request.workspace_parameters.min_corner.y = float(workspace["bounds"]["min"][1])
        goal.request.workspace_parameters.min_corner.z = float(workspace["bounds"]["min"][2])
        goal.request.workspace_parameters.max_corner.x = float(workspace["bounds"]["max"][0])
        goal.request.workspace_parameters.max_corner.y = float(workspace["bounds"]["max"][1])
        goal.request.workspace_parameters.max_corner.z = float(workspace["bounds"]["max"][2])
    goal.planning_options.plan_only = False
    goal.planning_options.look_around = False
    goal.planning_options.replan = False

    def sample_tf():
        point = _lookup_tf_point(
            tf_buffer,
            case["moveit"]["frame"],
            case["moveit"]["target_link"],
        )
        if point is not None:
            tcp_points.append(point)

    started = time.monotonic()
    send_future = action_client.send_goal_async(goal)
    _spin_until_future(node, send_future, timeout, sample_tf)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        return {
            "success": False,
            "target_pose": list(target_pose),
            "moveit_error_code": None,
            "planning_time_sec": 0.0,
            "execution_time_sec": time.monotonic() - started,
            "status": "rejected",
        }

    result_future = goal_handle.get_result_async()
    _spin_until_future(node, result_future, timeout, sample_tf)
    result = result_future.result()
    elapsed = time.monotonic() - started
    if result is None:
        return {
            "success": False,
            "target_pose": list(target_pose),
            "moveit_error_code": None,
            "planning_time_sec": 0.0,
            "execution_time_sec": elapsed,
            "status": "timeout",
        }
    error_code = result.result.error_code.val
    return {
        "success": (
            result.status == GoalStatus.STATUS_SUCCEEDED
            and error_code == MoveItErrorCodes.SUCCESS
        ),
        "target_pose": list(target_pose),
        "moveit_error_code": int(error_code),
        "planning_time_sec": float(result.result.planning_time),
        "execution_time_sec": float(elapsed),
        "status": int(result.status),
    }


def _pose_constraints(case, target_pose, phase):
    from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
    from shape_msgs.msg import SolidPrimitive

    criteria = case["pass_criteria"]
    frame_id = case["moveit"]["frame"]
    target_link = case["moveit"]["target_link"]
    tolerance = criteria["position_tolerance_m"]

    constraints = Constraints()
    constraints.name = f"{case['name']}_{phase}"

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [tolerance * 2.0, tolerance * 2.0, tolerance * 2.0]

    position = PositionConstraint()
    position.header.frame_id = frame_id
    position.link_name = target_link
    position.constraint_region.primitives.append(box)
    position.constraint_region.primitive_poses.append(_pose_message((
        target_pose[0],
        target_pose[1],
        target_pose[2],
        0.0,
        0.0,
        0.0,
    )))
    position.weight = 1.0
    constraints.position_constraints.append(position)

    orientation = OrientationConstraint()
    orientation.header.frame_id = frame_id
    orientation.link_name = target_link
    orientation.orientation = _orientation_message(target_pose[3:])
    orientation.absolute_x_axis_tolerance = criteria["orientation_tolerance_rad"]
    orientation.absolute_y_axis_tolerance = criteria["orientation_tolerance_rad"]
    orientation.absolute_z_axis_tolerance = criteria["orientation_tolerance_rad"]
    orientation.weight = 0.5
    constraints.orientation_constraints.append(orientation)
    return constraints


def _validation_sensor_hz(node, context, criteria):
    result = {}
    for topic in _sensor_topics(context["profile"], context["sensors"], context["namespaces"]):
        hz, samples = _measure_topic_hz(
            node,
            topic["name"],
            topic["type"],
            float(context["profile"]["smoke"]["sensors"].get("topic_timeout", 6.0)),
            int(context["profile"]["smoke"]["sensors"].get("min_samples", 3)),
        )
        result[topic["name"]] = {
            "hz": hz,
            "samples": samples,
            "type": topic["type"],
            "ok": hz >= criteria["required_sensor_min_hz"],
        }
    return result


def _expected_topic_metrics(case, sensor_hz, criteria):
    result = {}
    for topic in case.get("expected_topics", []):
        name = topic["name"]
        min_hz = float(topic.get("min_hz", criteria["required_sensor_min_hz"]))
        measured = sensor_hz.get(name, {})
        hz = measured.get("hz")
        result[name] = {
            "min_hz": min_hz,
            "hz": hz,
            "samples": measured.get("samples"),
            "ok": hz is not None and hz >= min_hz,
        }
    return result


def _validation_passed(metrics, criteria, success_code):
    failures = metrics["failures"]
    if not metrics["start"].get("success"):
        failures.append("start move failed")
    if not metrics["goal"].get("success"):
        failures.append("goal move failed")
    if metrics.get("moveit_error_code") != success_code:
        failures.append(f"MoveIt error code {metrics.get('moveit_error_code')}")
    if criteria["require_tf_ok"] and not metrics.get("tf_ok"):
        failures.append("TF lookup failed")

    goal_error = metrics.get("goal_position_error_m")
    if goal_error is None:
        failures.append("goal position error unavailable")
    elif goal_error > criteria["max_goal_position_error_m"]:
        failures.append(
            f"goal error {goal_error:.3f} > {criteria['max_goal_position_error_m']:.3f}"
        )

    clearance = metrics.get("min_tcp_clearance_m")
    if clearance is not None and clearance < criteria["min_tcp_clearance_m"]:
        failures.append(
            f"TCP clearance {clearance:.3f} < {criteria['min_tcp_clearance_m']:.3f}"
        )

    controller_error = metrics.get("max_controller_error_rad")
    if (
        controller_error is not None
        and controller_error > criteria["max_controller_error_rad"]
    ):
        failures.append(
            f"controller error {controller_error:.3f} > "
            f"{criteria['max_controller_error_rad']:.3f}"
        )

    bad_topics = [
        name
        for name, hz_metrics in metrics.get("sensor_hz", {}).items()
        if not hz_metrics.get("ok", False)
    ]
    if bad_topics:
        failures.append("sensor hz below threshold: " + ", ".join(sorted(bad_topics)))

    bad_expected = [
        name
        for name, topic_metrics in metrics.get("expected_topics", {}).items()
        if not topic_metrics.get("ok", False)
    ]
    if bad_expected:
        failures.append("expected topic hz below threshold: " + ", ".join(sorted(bad_expected)))

    return not failures


def _spin_until_future(node, future, timeout, sample_fn):
    import rclpy

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        sample_fn()
        if future.done():
            return future.result()
    raise RuntimeError("Timed out waiting for asynchronous validation operation")


def _settled_controller_error(node, controller_state, settle_sec=1.0):
    import rclpy

    controller_state["settled_errors"] = []
    deadline = time.monotonic() + settle_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    settled_errors = controller_state["settled_errors"]
    controller_state["settled_errors"] = None
    if settled_errors:
        return max(settled_errors)
    return controller_state["latest_error"]


def _lookup_tf_point(tf_buffer, frame_id, target_link):
    import rclpy

    try:
        transform = tf_buffer.lookup_transform(
            frame_id,
            target_link,
            rclpy.time.Time(),
        )
    except Exception:
        return None
    translation = transform.transform.translation
    return (float(translation.x), float(translation.y), float(translation.z))


def _pose_message(pose):
    from geometry_msgs.msg import Pose

    msg = Pose()
    msg.position.x = float(pose[0])
    msg.position.y = float(pose[1])
    msg.position.z = float(pose[2])
    msg.orientation = _orientation_message(pose[3:])
    return msg


def _orientation_message(rpy):
    from geometry_msgs.msg import Quaternion

    q = quaternion_from_rpy(*rpy)
    msg = Quaternion()
    msg.x = q[0]
    msg.y = q[1]
    msg.z = q[2]
    msg.w = q[3]
    return msg


def _distance(first, second):
    return math.sqrt(
        sum((float(first[index]) - float(second[index])) ** 2 for index in range(3))
    )


def _write_metrics(path, metrics):
    if not path:
        return
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2, sort_keys=True)
        handle.write("\n")


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
    check_sensors.add_argument("--topic-timeout", type=float, default=None)
    check_sensors.add_argument("--min-hz", type=float, default=None)
    check_sensors.add_argument("--min-samples", type=int, default=None)
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

    validation_case_env = subparsers.add_parser("validation-case-env")
    validation_case_env.add_argument("--validation-case", required=True)
    validation_case_env.set_defaults(func=command_validation_case_env)

    validation_case_json = subparsers.add_parser("validation-case-json")
    validation_case_json.add_argument("--validation-case", required=True)
    validation_case_json.set_defaults(func=command_validation_case_json)

    validate_case = subparsers.add_parser("validate-case")
    _add_common_args(validate_case)
    validate_case.add_argument("--validation-case", required=True)
    validate_case.add_argument("--metrics-output", required=True)
    validate_case.add_argument("--timeout", type=float, default=120.0)
    validate_case.set_defaults(func=command_validate_case)

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
