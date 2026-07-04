from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any, Mapping

import yaml


SUPPORTED_ADAPTERS = {
    "tf_to_tcp_pos",
    "moveit_pose_service",
    "scan3d_service",
    "synthetic_weld_vision",
    "loop_motion_services",
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run one robot_sim module adapter.")
    parser.add_argument("--adapter-file", required=True)
    parser.add_argument("--case-file", default="")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    config = _load_mapping(Path(args.adapter_file))
    adapter_type = str(config.get("type", ""))
    if adapter_type not in SUPPORTED_ADAPTERS:
        valid = ", ".join(sorted(SUPPORTED_ADAPTERS))
        raise RuntimeError(f"unknown module adapter '{adapter_type}'. Valid adapters: {valid}")

    if adapter_type == "tf_to_tcp_pos":
        _run_tf_to_tcp_pos(config)
    elif adapter_type == "scan3d_service":
        _run_scan3d_service(config)
    elif adapter_type == "synthetic_weld_vision":
        _run_synthetic_weld_vision(config)
    elif adapter_type == "loop_motion_services":
        _run_loop_motion_services(config)
    elif adapter_type == "moveit_pose_service":
        _run_moveit_pose_service(config, args.case_file)
    return 0


def adapter_dependencies(config: Mapping[str, Any]) -> list[str]:
    adapter_type = str(config.get("type", ""))
    if adapter_type == "tf_to_tcp_pos":
        return ["weld_interface/msg/TcpPos"]
    if adapter_type == "scan3d_service":
        return ["weld_interface/srv/Scan3d", "weld_interface/msg/TcpPos"]
    if adapter_type == "synthetic_weld_vision":
        return ["weld_interface/msg/WeldVisionResult", "weld_interface/msg/TcpPos"]
    if adapter_type == "loop_motion_services":
        return [
            "weld_interface/srv/SpecialSpeedl",
            "weld_interface/srv/FanucMovRate",
            "std_srvs/srv/SetBool",
            "std_srvs/srv/Empty",
        ]
    if adapter_type == "moveit_pose_service":
        return ["weld_interface/srv/SpecialSpeedl", "moveit_msgs/action/MoveGroup"]
    return []


def preflight_adapter(config: Mapping[str, Any]) -> None:
    for type_name in adapter_dependencies(config):
        if "/srv/" in type_name:
            _get_service(type_name)
        elif "/msg/" in type_name:
            _get_message(type_name)
        elif "/action/" in type_name:
            _get_action(type_name)


def _run_tf_to_tcp_pos(config: Mapping[str, Any]) -> None:
    import rclpy
    from tf2_ros import Buffer, TransformListener

    TcpPos = _get_message(str(config.get("message_type", "weld_interface/msg/TcpPos")))
    topic = str(config.get("topic", "/tool_pos"))
    parent_frame = str(config.get("parent_frame", "world"))
    child_frame = str(config.get("child_frame", "tool0"))
    rate_hz = float(config.get("rate_hz", 20.0))
    fallback_pose = _pose_dict(config.get("fallback_pose", {}))

    rclpy.init(args=None)
    node = rclpy.create_node(str(config.get("node_name", "robot_sim_tf_to_tcp_pos")))
    publisher = node.create_publisher(TcpPos, topic, 10)
    buffer = Buffer()
    TransformListener(buffer, node)

    def publish() -> None:
        pose = dict(fallback_pose)
        try:
            transform = buffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            roll, pitch, yaw = _quaternion_to_rpy(rotation.x, rotation.y, rotation.z, rotation.w)
            pose.update({
                "x": float(translation.x),
                "y": float(translation.y),
                "z": float(translation.z),
                "rx": roll,
                "ry": pitch,
                "rz": yaw,
            })
        except Exception:
            pass
        publisher.publish(_tcp_pos_message(TcpPos, node, pose, parent_frame))

    node.create_timer(1.0 / max(rate_hz, 0.1), publish)
    node.get_logger().info(f"tf_to_tcp_pos publishing {parent_frame}->{child_frame} on {topic}")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _run_scan3d_service(config: Mapping[str, Any]) -> None:
    import numpy as np
    import rclpy

    Scan3d = _get_service(str(config.get("service_type", "weld_interface/srv/Scan3d")))
    TcpPos = _get_message("weld_interface/msg/TcpPos")
    service_name = str(config.get("service", "/scan_3d"))
    source = _mapping(config.get("source", {}))
    image_array, points_array, frame_info = _load_scan3d_source(source)
    image_frame = str(source.get("image_frame_id") or frame_info.get("image_frame_id") or "camera_frame")
    cloud_frame = str(source.get("point_cloud_frame_id") or frame_info.get("point_cloud_frame_id") or "camera")
    scan_pose = _pose_dict(source.get("scan_pose", {}))

    rclpy.init(args=None)
    node = rclpy.create_node(str(config.get("node_name", "robot_sim_scan3d_service")))

    def callback(_request, response):
        stamp = node.get_clock().now().to_msg()
        response.image = _image_message(image_array, stamp, image_frame)
        response.points = _point_cloud2_message(points_array, stamp, cloud_frame)
        response.scan_pose = _tcp_pos_message(TcpPos, node, scan_pose, "world")
        return response

    node.create_service(Scan3d, service_name, callback)
    node.get_logger().info(
        f"scan3d_service ready on {service_name}: image={image_array.shape} points={points_array.shape}"
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _run_synthetic_weld_vision(config: Mapping[str, Any]) -> None:
    import rclpy

    WeldVisionResult = _get_message(
        str(config.get("message_type", "weld_interface/msg/WeldVisionResult"))
    )
    TcpPos = _get_message("weld_interface/msg/TcpPos")
    topic = str(config.get("topic", "/welding/vision_result"))
    tcp_topic = str(config.get("tcp_pose_topic", "/tool_pos"))
    rate_hz = float(config.get("rate_hz", 15.0))
    line = _mapping(config.get("line", {}))
    latest_tcp = {"message": _tcp_pos_plain(TcpPos, _pose_dict(config.get("tcp_pose", {})), "world")}
    frame_counter = {"value": 0}
    drop_every_n = int(config.get("drop_every_n", 0) or 0)

    rclpy.init(args=None)
    node = rclpy.create_node(str(config.get("node_name", "robot_sim_synthetic_weld_vision")))
    publisher = node.create_publisher(WeldVisionResult, topic, 10)
    node.create_subscription(TcpPos, tcp_topic, lambda msg: latest_tcp.__setitem__("message", msg), 10)

    def publish() -> None:
        frame_counter["value"] += 1
        if drop_every_n > 0 and frame_counter["value"] % drop_every_n == 0:
            return
        msg = WeldVisionResult()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = str(config.get("frame_id", "pool_camera"))
        msg.image_width = int(config.get("image_width", 1440))
        msg.image_height = int(config.get("image_height", 1080))
        msg.source = str(config.get("source_name", "robot_sim_synthetic"))
        msg.line_detected = bool(config.get("line_detected", True))
        msg.line_a = float(line.get("a", 1.0))
        msg.line_b = float(line.get("b", 0.0))
        msg.line_c = float(line.get("c", -720.0))
        msg.tab_detected = bool(config.get("tab_detected", False))
        msg.tab_x = float(config.get("tab_x", 0.0))
        msg.tab_y = float(config.get("tab_y", 0.0))
        msg.has_tcp_pose = bool(config.get("has_tcp_pose", True))
        msg.tcp_pose = latest_tcp["message"]
        publisher.publish(msg)

    node.create_timer(1.0 / max(rate_hz, 0.1), publish)
    node.get_logger().info(f"synthetic_weld_vision publishing on {topic}")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _run_loop_motion_services(config: Mapping[str, Any]) -> None:
    import rclpy

    SpecialSpeedl = _get_service("weld_interface/srv/SpecialSpeedl")
    FanucMovRate = _get_service("weld_interface/srv/FanucMovRate")
    SetBool = _get_service("std_srvs/srv/SetBool")
    Empty = _get_service("std_srvs/srv/Empty")

    rclpy.init(args=None)
    node = rclpy.create_node(str(config.get("node_name", "robot_sim_loop_motion_services")))

    def special_speedl(_request, response):
        response.success = True
        return response

    def rate(_request, response):
        response.success = True
        return response

    def enable(request, response):
        response.success = True
        response.message = f"loop {'enabled' if request.data else 'disabled'}"
        return response

    def stop(_request, response):
        return response

    node.create_service(SpecialSpeedl, str(config.get("position_service", "/any_mov_loop_position_set")), special_speedl)
    node.create_service(FanucMovRate, str(config.get("rate_service", "/any_mov_loop_rate_set")), rate)
    node.create_service(SetBool, str(config.get("enable_service", "/any_mov_loop_sign_set")), enable)
    node.create_service(Empty, str(config.get("stop_service", "/stop_mov_jog")), stop)
    node.get_logger().info("loop_motion_services ready")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _run_moveit_pose_service(config: Mapping[str, Any], case_file: str) -> None:
    import rclpy

    SpecialSpeedl = _get_service(str(config.get("service_type", "weld_interface/srv/SpecialSpeedl")))
    service_name = str(config.get("service", "/any_mov_jog"))
    execute_moveit = bool(config.get("execute_moveit", True))
    result_state = {"last_result": None}

    rclpy.init(args=None)
    node = rclpy.create_node(str(config.get("node_name", "robot_sim_moveit_pose_service")))

    def callback(request, response):
        target = [
            float(request.x),
            float(request.y),
            float(request.z),
            float(request.rx),
            float(request.ry),
            float(request.rz),
        ]
        if execute_moveit:
            try:
                result_state["last_result"] = _execute_moveit_target(node, config, case_file, target)
                response.success = bool(result_state["last_result"].get("success", False))
            except Exception as exc:
                node.get_logger().error(f"moveit_pose_service failed: {exc}")
                result_state["last_result"] = {"success": False, "error": str(exc), "target_pose": target}
                response.success = False
        else:
            result_state["last_result"] = {"success": True, "target_pose": target, "skipped": True}
            response.success = True
        return response

    node.create_service(SpecialSpeedl, service_name, callback)
    node.get_logger().info(f"moveit_pose_service ready on {service_name}")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _execute_moveit_target(service_node, config: Mapping[str, Any], case_file: str, target_pose: list[float]) -> dict[str, Any]:
    import rclpy
    from rclpy.action import ActionClient
    from tf2_ros import Buffer, TransformListener

    from robot_sim_bringup import sim_smoke_helper
    from robot_sim_bringup.validation_cases import load_validation_case

    worker_name = str(config.get("worker_node_name", "robot_sim_moveit_pose_worker"))
    worker_node = rclpy.create_node(worker_name)
    try:
        case = load_validation_case(case_file)
        launch = case["launch"]
        args = type("Args", (), {
            "profile": launch["profile"],
            "profile_file": launch.get("profile_file", ""),
            "profile_package": launch.get("profile_package", ""),
            "mode": launch["mode"],
            "sensor_overrides": launch.get("sensor_overrides", ""),
        })()
        context = sim_smoke_helper._load_context(args, require_moveit=True)
        move_action = str(config.get("move_action") or context["move_action"])
        action_client = ActionClient(worker_node, _get_action("moveit_msgs/action/MoveGroup"), move_action)
        if not action_client.wait_for_server(timeout_sec=float(config.get("server_timeout_sec", 15.0))):
            raise RuntimeError(f"MoveIt action server not available: {move_action}")
        tf_buffer = Buffer()
        TransformListener(tf_buffer, worker_node)
        adapter_case = dict(case)
        adapter_case["moveit"] = dict(case["moveit"])
        adapter_case["moveit"]["execute"] = True
        adapter_case["pass_criteria"] = dict(case["pass_criteria"])
        adapter_case["pass_criteria"]["position_tolerance_m"] = float(config.get("position_tolerance_m", 0.08))
        adapter_case["pass_criteria"]["orientation_tolerance_rad"] = float(
            config.get("orientation_tolerance_rad", math.pi)
        )
        return sim_smoke_helper._execute_pose_goal(
            worker_node,
            action_client,
            context,
            adapter_case,
            tuple(target_pose),
            float(config.get("timeout_sec", 60.0)),
            tf_buffer,
            [],
            "module_adapter",
        )
    finally:
        service_node.get_logger().debug("destroying moveit_pose_service worker node")
        worker_node.destroy_node()


def _load_scan3d_source(source: Mapping[str, Any]):
    import numpy as np

    source_type = str(source.get("type", "replay"))
    if source_type not in ("replay", "gazebo_rgbd"):
        raise RuntimeError(f"scan3d_service source.type must be replay or gazebo_rgbd; got {source_type!r}")
    if source_type == "gazebo_rgbd":
        raise RuntimeError("scan3d_service gazebo_rgbd source is reserved for the next adapter revision")

    record_path = Path(str(source.get("record_path", ""))).expanduser()
    if not record_path.exists():
        raise RuntimeError(f"scan3d replay record does not exist: {record_path}")
    record = _load_mapping(record_path)
    image_path = _resolve_scan_path(record_path, record, "image_path", ("artifacts", "image"))
    points_path = _resolve_scan_path(record_path, record, "point_cloud_path", ("artifacts", "point_cloud_xyz"))
    if not image_path.exists():
        raise RuntimeError(f"scan3d replay image does not exist: {image_path}")
    if not points_path.exists():
        raise RuntimeError(f"scan3d replay point cloud does not exist: {points_path}")
    image = _read_image(image_path)
    data = np.load(points_path)
    if "points" not in data:
        raise RuntimeError(f"scan3d replay point cloud npz missing 'points': {points_path}")
    points = np.ascontiguousarray(data["points"], dtype=np.float32)
    if points.ndim != 3 or points.shape[2] != 3:
        raise RuntimeError(f"scan3d replay points must have shape (H, W, 3): {points.shape}")
    return image, points, record


def _read_image(path: Path):
    import numpy as np

    try:
        import cv2
    except Exception as exc:
        raise RuntimeError("scan3d replay image loading requires python3-opencv/cv2") from exc
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f"failed to read image: {path}")
    if image.ndim == 2:
        return np.ascontiguousarray(image)
    if image.shape[2] == 4:
        image = image[:, :, :3]
    return np.ascontiguousarray(image)


def _resolve_scan_path(record_path: Path, record: Mapping[str, Any], key: str, nested: tuple[str, str]) -> Path:
    value = record.get(key)
    if not value:
        parent = _mapping(record.get(nested[0], {}))
        value = parent.get(nested[1], "")
    if not value:
        raise RuntimeError(f"scan3d replay record missing {key} or {'.'.join(nested)}: {record_path}")
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = record_path.parent / path
    return path


def _image_message(image_array, stamp, frame_id: str):
    from sensor_msgs.msg import Image

    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(image_array.shape[0])
    msg.width = int(image_array.shape[1])
    if image_array.ndim == 2:
        msg.encoding = "mono8"
        channels = 1
    else:
        channels = int(image_array.shape[2])
        msg.encoding = "bgr8" if channels == 3 else "passthrough"
    msg.is_bigendian = False
    msg.step = int(msg.width * channels * image_array.dtype.itemsize)
    msg.data = image_array.tobytes()
    return msg


def _point_cloud2_message(points_array, stamp, frame_id: str):
    from sensor_msgs.msg import PointCloud2, PointField

    points = points_array.astype("float32", copy=False)
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(points.shape[0])
    msg.width = int(points.shape[1])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = int(msg.point_step * msg.width)
    msg.data = points.tobytes()
    msg.is_dense = False
    return msg


def _tcp_pos_message(TcpPos, node, pose: Mapping[str, float], frame_id: str):
    msg = _tcp_pos_plain(TcpPos, pose, frame_id)
    msg.header.stamp = node.get_clock().now().to_msg()
    return msg


def _tcp_pos_plain(TcpPos, pose: Mapping[str, float], frame_id: str):
    msg = TcpPos()
    msg.header.frame_id = frame_id
    for name in ("x", "y", "z", "rx", "ry", "rz", "e1", "e2", "e3"):
        setattr(msg, name, float(pose.get(name, 0.0)))
    return msg


def _pose_dict(value: Any) -> dict[str, float]:
    raw = _mapping(value)
    return {name: float(raw.get(name, 0.0)) for name in ("x", "y", "z", "rx", "ry", "rz", "e1", "e2", "e3")}


def _quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _get_message(type_name: str):
    from rosidl_runtime_py.utilities import get_message

    return get_message(type_name)


def _get_service(type_name: str):
    from rosidl_runtime_py.utilities import get_service

    return get_service(type_name)


def _get_action(type_name: str):
    from rosidl_runtime_py.utilities import get_action

    return get_action(type_name)


def _load_mapping(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        if path.suffix == ".json":
            raw = json.load(handle)
        else:
            raw = yaml.safe_load(handle)
    if not isinstance(raw, dict):
        raise RuntimeError(f"expected mapping: {path}")
    return raw


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


if __name__ == "__main__":
    raise SystemExit(main())
