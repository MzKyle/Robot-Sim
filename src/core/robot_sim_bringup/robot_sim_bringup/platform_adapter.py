from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any, Mapping

import yaml

from robot_sim_bringup.platform_assertions import assign_fields


SUPPORTED_ADAPTERS = {
    "topic_replay",
    "image_camera_replay",
    "service_stub",
    "tf_static_publisher",
    "joint_state_replay",
    "message_sequence_publisher",
    "process_supervisor",
    "service_proxy",
    "action_stub",
    "rosbag_replay",
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run one generic robot_sim platform adapter.")
    parser.add_argument("--adapter-file", required=True)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    config = _load_mapping(Path(args.adapter_file))
    adapter_type = str(config.get("type", ""))
    if adapter_type not in SUPPORTED_ADAPTERS:
        valid = ", ".join(sorted(SUPPORTED_ADAPTERS))
        raise RuntimeError(f"unknown platform adapter '{adapter_type}'. Valid adapters: {valid}")
    if adapter_type in ("topic_replay", "message_sequence_publisher"):
        _run_topic_replay(config)
    elif adapter_type == "image_camera_replay":
        _run_image_camera_replay(config)
    elif adapter_type == "service_stub":
        _run_service_stub(config)
    elif adapter_type == "tf_static_publisher":
        _run_tf_static_publisher(config)
    elif adapter_type == "joint_state_replay":
        _run_joint_state_replay(config)
    elif adapter_type == "rosbag_replay":
        _run_rosbag_replay(config)
    else:
        raise RuntimeError(f"adapter '{adapter_type}' is declared but not implemented yet")
    return 0


def adapter_dependencies(config: Mapping[str, Any]) -> list[str]:
    adapter_type = str(config.get("type", ""))
    if adapter_type in ("topic_replay", "message_sequence_publisher"):
        return [str(config.get("message_type", ""))]
    if adapter_type == "image_camera_replay":
        return ["sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo"]
    if adapter_type == "service_stub":
        return [str(config.get("service_type", ""))]
    if adapter_type == "tf_static_publisher":
        return ["geometry_msgs/msg/TransformStamped"]
    if adapter_type == "joint_state_replay":
        return ["sensor_msgs/msg/JointState"]
    return []


def preflight_adapter(config: Mapping[str, Any]) -> None:
    from rosidl_runtime_py.utilities import get_message, get_service

    for type_name in adapter_dependencies(config):
        if not type_name:
            continue
        if "/srv/" in type_name:
            get_service(type_name)
        elif "/msg/" in type_name:
            get_message(type_name)


def _run_topic_replay(config: Mapping[str, Any]) -> None:
    import rclpy
    from rosidl_runtime_py.utilities import get_message

    Message = get_message(str(config["message_type"]))
    topic = str(config["topic"])
    rate_hz = float(config.get("rate_hz", 10.0))
    repeat = bool(config.get("repeat", False))
    messages = _load_messages(config)
    if not messages:
        messages = [dict(config.get("message", {}))]

    rclpy.init(args=None)
    node = rclpy.create_node(str(config.get("node_name", "robot_sim_topic_replay")))
    publisher = node.create_publisher(Message, topic, int(config.get("qos_depth", 10)))
    state = {"index": 0}

    def publish() -> None:
        if state["index"] >= len(messages):
            if not repeat:
                return
            state["index"] = 0
        msg = Message()
        assign_fields(msg, messages[state["index"]])
        publisher.publish(msg)
        state["index"] += 1

    node.create_timer(1.0 / max(rate_hz, 0.1), publish)
    node.get_logger().info(f"topic_replay publishing {len(messages)} messages on {topic}")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _run_service_stub(config: Mapping[str, Any]) -> None:
    import rclpy
    from rosidl_runtime_py.utilities import get_service

    Service = get_service(str(config["service_type"]))
    service = str(config["service"])
    response_fields = dict(config.get("response", {}) or {})
    delay_sec = float(config.get("delay_sec", 0.0) or 0.0)

    rclpy.init(args=None)
    node = rclpy.create_node(str(config.get("node_name", "robot_sim_service_stub")))

    def callback(_request, response):
        if delay_sec > 0:
            import time

            time.sleep(delay_sec)
        assign_fields(response, response_fields)
        return response

    node.create_service(Service, service, callback)
    node.get_logger().info(f"service_stub ready on {service}")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _run_joint_state_replay(config: Mapping[str, Any]) -> None:
    import rclpy
    from sensor_msgs.msg import JointState

    topic = str(config.get("topic", "/joint_states"))
    rate_hz = float(config.get("rate_hz", 20.0))
    repeat = bool(config.get("repeat", True))
    states = _load_messages(config)
    if not states:
        states = [dict(config.get("message", {}))]

    rclpy.init(args=None)
    node = rclpy.create_node(str(config.get("node_name", "robot_sim_joint_state_replay")))
    publisher = node.create_publisher(JointState, topic, int(config.get("qos_depth", 10)))
    state = {"index": 0}

    def publish() -> None:
        if state["index"] >= len(states):
            if not repeat:
                return
            state["index"] = 0
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        assign_fields(msg, states[state["index"]])
        publisher.publish(msg)
        state["index"] += 1

    node.create_timer(1.0 / max(rate_hz, 0.1), publish)
    node.get_logger().info(f"joint_state_replay publishing on {topic}")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _run_image_camera_replay(config: Mapping[str, Any]) -> None:
    import rclpy
    from sensor_msgs.msg import CameraInfo, Image

    np = __import__("numpy")
    cv2 = None
    try:
        cv2 = __import__("cv2")
    except Exception:
        pass

    image_topic = str(config.get("image_topic", "/image_raw"))
    camera_info_topic = str(config.get("camera_info_topic", "/camera_info"))
    frame_id = str(config.get("frame_id", "camera_optical_frame"))
    rate_hz = float(config.get("rate_hz", 10.0))
    image_paths = [str(path) for path in config.get("images", []) or []]
    if config.get("image"):
        image_paths.append(str(config["image"]))
    video_path = str(config.get("video", ""))
    width = int(config.get("width", 640))
    height = int(config.get("height", 480))
    repeat = bool(config.get("repeat", True))
    encoding = str(config.get("encoding", "bgr8"))
    capture = cv2.VideoCapture(video_path) if video_path and cv2 is not None else None
    if capture is not None and config.get("start_sec"):
        capture.set(cv2.CAP_PROP_POS_MSEC, float(config.get("start_sec", 0.0)) * 1000.0)

    rclpy.init(args=None)
    node = rclpy.create_node(str(config.get("node_name", "robot_sim_image_camera_replay")))
    image_pub = node.create_publisher(Image, image_topic, 10)
    info_pub = node.create_publisher(CameraInfo, camera_info_topic, 10)
    state = {"index": 0}

    def publish() -> None:
        if capture is not None:
            ok, image = capture.read()
            if not ok and repeat:
                capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, image = capture.read()
            if not ok:
                image = None
        elif image_paths:
            if state["index"] >= len(image_paths):
                if not repeat:
                    return
                state["index"] = 0
            image = cv2.imread(image_paths[state["index"]]) if cv2 is not None else None
            state["index"] += 1
        else:
            image = np.zeros((height, width, 3), dtype=np.uint8)
        if image is None:
            image = np.zeros((height, width, 3), dtype=np.uint8)
        stamp = node.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(image.shape[1] * 3)
        msg.data = image.reshape(-1).tolist()
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.height = msg.height
        info.width = msg.width
        assign_fields(info, config.get("camera_info", {}))
        image_pub.publish(msg)
        info_pub.publish(info)

    node.create_timer(1.0 / max(rate_hz, 0.1), publish)
    node.get_logger().info(f"image_camera_replay publishing {image_topic} and {camera_info_topic}")
    try:
        rclpy.spin(node)
    finally:
        if capture is not None:
            capture.release()
        node.destroy_node()
        rclpy.shutdown()


def _run_tf_static_publisher(config: Mapping[str, Any]) -> None:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    transforms = config.get("transforms", []) or []
    rclpy.init(args=None)
    node = rclpy.create_node(str(config.get("node_name", "robot_sim_tf_static_publisher")))
    broadcaster = StaticTransformBroadcaster(node)
    messages = []
    for item in transforms:
        msg = TransformStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        assign_fields(msg, item)
        messages.append(msg)
    if messages:
        broadcaster.sendTransform(messages)
    node.get_logger().info(f"tf_static_publisher published {len(messages)} transforms")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _run_rosbag_replay(config: Mapping[str, Any]) -> None:
    import os
    import signal
    import subprocess
    import time

    command = ["ros2", "bag", "play", str(config["bag"])]
    if config.get("clock"):
        command.append("--clock")
    process = subprocess.Popen(command, preexec_fn=os.setsid)
    try:
        while process.poll() is None:
            time.sleep(0.5)
    finally:
        if process.poll() is None:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)


def _load_messages(config: Mapping[str, Any]) -> list[dict[str, Any]]:
    if config.get("messages"):
        return [dict(item) for item in config.get("messages", [])]
    if config.get("path"):
        path = Path(str(config["path"])).expanduser()
        if path.suffix == ".json":
            data = json.loads(path.read_text(encoding="utf-8"))
            records_key = str(config.get("records_key", ""))
            if records_key:
                data = _nested_value(data, records_key)
            if isinstance(data, list):
                return [_map_record(dict(item), config.get("field_map", {})) for item in data]
            if isinstance(data, dict) and isinstance(data.get("messages"), list):
                return [_map_record(dict(item), config.get("field_map", {})) for item in data["messages"]]
        if path.suffix in (".yaml", ".yml"):
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
            records_key = str(config.get("records_key", ""))
            if records_key:
                data = _nested_value(data, records_key)
            if isinstance(data, list):
                return [_map_record(dict(item), config.get("field_map", {})) for item in data]
            if isinstance(data, dict) and isinstance(data.get("messages"), list):
                return [_map_record(dict(item), config.get("field_map", {})) for item in data["messages"]]
        if path.suffix == ".csv":
            with path.open("r", encoding="utf-8", newline="") as handle:
                return [
                    _map_record(row, config.get("field_map", {}))
                    for row in csv.DictReader(handle)
                ]
    return []


def _map_record(record: Mapping[str, Any], field_map: Mapping[str, Any]) -> dict[str, Any]:
    typed = {str(key): _typed_value(value) for key, value in record.items()}
    if not field_map:
        return typed
    result: dict[str, Any] = {}
    for target, source in dict(field_map).items():
        if isinstance(source, str):
            value = typed.get(source, source)
        elif isinstance(source, list):
            value = [
                typed.get(str(item), item)
                for item in source
            ]
        else:
            value = source
        _assign_nested(result, str(target), value)
    return result


def _assign_nested(target: dict[str, Any], path: str, value: Any) -> None:
    parts = path.split(".")
    current = target
    for part in parts[:-1]:
        current = current.setdefault(part, {})
    current[parts[-1]] = value


def _nested_value(value: Any, path: str) -> Any:
    current = value
    for part in path.split("."):
        if isinstance(current, Mapping):
            current = current.get(part)
        elif isinstance(current, list) and part.isdigit():
            index = int(part)
            current = current[index] if index < len(current) else None
        else:
            return None
    return current


def _typed_value(value: Any) -> Any:
    if not isinstance(value, str):
        return value
    lowered = value.lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    try:
        if "." in value:
            return float(value)
        return int(value)
    except ValueError:
        return value


def _load_mapping(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, dict):
        raise RuntimeError(f"adapter YAML must be a mapping: {path}")
    return raw


if __name__ == "__main__":
    raise SystemExit(main())
