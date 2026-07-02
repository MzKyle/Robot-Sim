import math
import os
import re
from pathlib import Path
from typing import Any, Mapping

import yaml
from robot_sim_scenarios import load_scene

from robot_sim_bringup.schema_validation import validate_config_schema
DEFAULT_EXCLUDE_TAGS = {
    "ground",
    "robot_mount",
    "pedestal",
    "visual_marker",
    "optional",
}


def load_validation_case(name_or_path: str | Path) -> dict[str, Any]:
    path = resolve_validation_case_path(name_or_path)
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, dict):
        raise RuntimeError(f"validation case YAML must be a mapping: {path}")
    validate_config_schema(raw, "validation_case.schema.json", "validation_case", path)

    name = str(raw.get("name") or path.stem)
    launch = _required_mapping(raw, "launch", path)
    profile = _required_string(launch, "profile", path)
    profile_file = str(launch.get("profile_file", ""))
    mode = str(launch.get("mode", "full"))
    if mode not in ("full", "light", "mock"):
        raise RuntimeError(f"mode must be full, light, or mock: {path}")
    layout = str(launch.get("layout", "single"))
    if layout not in ("single", "distributed"):
        raise RuntimeError(f"launch.layout must be single or distributed: {path}")

    scene_ref = _resolve_scene_ref(raw.get("scene"), "scene")
    scene = load_scene(scene_ref)

    task = _required_mapping(raw, "task", path)
    task_type = _required_string(task, "type", path)
    start_region = _required_string(task, "start_region", path)
    goal_region = _required_string(task, "goal_region", path)
    for region_name in (start_region, goal_region):
        if region_name not in scene.regions:
            valid = ", ".join(sorted(scene.regions))
            raise RuntimeError(
                f"validation case region '{region_name}' is not in scene "
                f"'{scene.name}'. Valid regions: {valid}"
            )

    moveit = _required_mapping(task, "moveit", path)
    moveit_config = {
        "group": str(moveit.get("group") or "manipulator"),
        "target_link": str(moveit.get("target_link") or "tool0"),
        "frame": str(moveit.get("frame") or "world"),
        "planning_time_sec": float(moveit.get("planning_time_sec", 8.0)),
        "velocity_scaling": float(moveit.get("velocity_scaling", 0.2)),
        "acceleration_scaling": float(moveit.get("acceleration_scaling", 0.2)),
    }

    planning_scene = _required_mapping(raw, "planning_scene", path)
    expect = _required_mapping(raw, "expect", path)
    artifacts = _required_mapping(raw, "artifacts", path)
    rosbag = _required_mapping(artifacts, "rosbag", path)

    case = {
        "name": name,
        "path": str(path),
        "profile": profile,
        "profile_file": profile_file,
        "mode": mode,
        "layout": layout,
        "timeout_sec": float(launch.get("timeout_sec", 120.0)),
        "scene": scene,
        "scene_ref": str(scene_ref),
        "seed": int(task.get("seed", 1)),
        "sensor_overrides": _sensor_overrides_text(launch.get("sensor_overrides", "")),
        "moveit": moveit_config,
        "task_type": task_type,
        "start_region": start_region,
        "goal_region": goal_region,
        "planning_scene": {
            "apply": bool(planning_scene.get("apply", True)),
            "exclude_tags": [
                str(tag)
                for tag in planning_scene.get("exclude_tags", sorted(DEFAULT_EXCLUDE_TAGS))
            ],
            "include_tags": [
                str(tag)
                for tag in planning_scene.get("include_tags", [])
            ],
        },
        "pass_criteria": {
            "max_goal_position_error_m": float(
                expect.get("max_goal_position_error_m", 0.20)
            ),
            "min_tcp_clearance_m": float(expect.get("min_tcp_clearance_m", 0.02)),
            "max_controller_error_rad": float(
                expect.get("max_controller_error_rad", 0.50)
            ),
            "position_tolerance_m": float(expect.get("position_tolerance_m", 0.10)),
            "orientation_tolerance_rad": float(
                expect.get("orientation_tolerance_rad", math.pi)
            ),
            "required_sensor_min_hz": float(
                expect.get("required_sensor_min_hz", 1.0)
            ),
            "require_tf_ok": bool(expect.get("require_tf_ok", True)),
        },
        "expected_topics": [
            {
                "name": str(topic["name"]),
                "min_hz": float(topic.get("min_hz", expect.get("required_sensor_min_hz", 1.0))),
            }
            for topic in expect.get("topics", [])
        ],
        "artifacts": {
            "rosbag": {
                "enabled": bool(rosbag.get("enabled", True)),
                "topic_group": str(rosbag.get("topic_group", "all")),
                "compression": bool(rosbag.get("compression", False)),
                "extra_topics": [str(topic) for topic in rosbag.get("extra_topics", [])],
            },
            "reports": [str(report) for report in artifacts.get("reports", ["md", "html"])],
        },
        "raw": raw,
    }
    case["launch"] = {
        "profile": profile,
        "profile_file": profile_file,
        "mode": mode,
        "layout": layout,
        "timeout_sec": case["timeout_sec"],
        "sensor_overrides": case["sensor_overrides"],
    }
    case["task"] = {
        "type": task_type,
        "seed": case["seed"],
        "start_region": start_region,
        "goal_region": goal_region,
        "moveit": moveit_config,
    }
    case["expect"] = {
        **case["pass_criteria"],
        "topics": case["expected_topics"],
    }
    return case


def resolve_validation_case_path(name_or_path: str | Path) -> Path:
    candidate = Path(name_or_path).expanduser()
    if candidate.exists():
        return candidate.resolve()
    if candidate.suffix in (".yaml", ".yml") or candidate.parent != Path("."):
        raise RuntimeError(f"validation case file does not exist: {candidate}")

    name = str(name_or_path)
    package_path = _bringup_share_dir() / "config" / "validation_cases" / f"{name}.yaml"
    if not package_path.exists():
        raise RuntimeError(f"unknown validation case '{name}': {package_path}")
    return package_path.resolve()


def collision_primitives_from_scene(
    scene,
    planning_scene: Mapping[str, Any] | None = None,
) -> list[dict[str, Any]]:
    config = planning_scene or {}
    exclude_tags = set(str(tag) for tag in config.get("exclude_tags", DEFAULT_EXCLUDE_TAGS))
    include_tags = set(str(tag) for tag in config.get("include_tags", []))
    primitives = []

    for scene_object in scene.objects:
        raw = scene_object.raw
        tags = set(scene_object.tags)
        if scene_object.object_type != "model":
            continue
        if not bool(raw.get("static", False)):
            continue
        if tags & exclude_tags:
            continue
        if include_tags and not (tags & include_tags):
            continue

        model_pose = tuple(float(value) for value in raw["pose"])
        if "links" in raw:
            for link in raw["links"]:
                link_pose = _pose_value(link.get("pose"), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                base_pose = compose_pose(model_pose, link_pose)
                if (
                    "geometry" in link
                    and bool(link.get("collision", False))
                    and not bool(link.get("visual_only", False))
                ):
                    _append_collision_primitive(
                        primitives,
                        scene_object.name,
                        link["name"],
                        "collision",
                        tags,
                        base_pose,
                        link["geometry"],
                    )
                for collision in link.get("collisions", []):
                    collision_pose = _pose_value(
                        collision.get("pose"),
                        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    )
                    _append_collision_primitive(
                        primitives,
                        scene_object.name,
                        link["name"],
                        collision["name"],
                        tags,
                        compose_pose(base_pose, collision_pose),
                        collision["geometry"],
                    )
            continue

        if bool(raw.get("collision", False)) and not bool(raw.get("visual_only", False)):
            _append_collision_primitive(
                primitives,
                scene_object.name,
                str(raw.get("link_name", "link")),
                "collision",
                tags,
                model_pose,
                raw["geometry"],
            )

    return primitives


def min_clearance_to_primitives(point: tuple[float, float, float], primitives) -> float | None:
    distances = [
        _distance_to_primitive(point, primitive)
        for primitive in primitives
        if primitive["geometry"]["type"] in ("box", "cylinder")
    ]
    if not distances:
        return None
    return min(distances)


def compose_pose(parent, child):
    parent_xyz = tuple(float(value) for value in parent[:3])
    child_xyz = tuple(float(value) for value in child[:3])
    parent_q = quaternion_from_rpy(*parent[3:])
    child_q = quaternion_from_rpy(*child[3:])
    rotated_child = rotate_vector(parent_q, child_xyz)
    xyz = tuple(parent_xyz[index] + rotated_child[index] for index in range(3))
    q = quaternion_multiply(parent_q, child_q)
    rpy = rpy_from_quaternion(q)
    return (*xyz, *rpy)


def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(float(roll) * 0.5)
    sr = math.sin(float(roll) * 0.5)
    cp = math.cos(float(pitch) * 0.5)
    sp = math.sin(float(pitch) * 0.5)
    cy = math.cos(float(yaw) * 0.5)
    sy = math.sin(float(yaw) * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_quaternion(quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def normalize_quaternion(quaternion):
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def rotate_vector(quaternion, vector):
    q = normalize_quaternion(quaternion)
    p = (float(vector[0]), float(vector[1]), float(vector[2]), 0.0)
    q_inv = (-q[0], -q[1], -q[2], q[3])
    rotated = quaternion_multiply_raw(quaternion_multiply_raw(q, p), q_inv)
    return rotated[:3]


def quaternion_multiply_raw(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def inverse_rotate_vector(quaternion, vector):
    q = normalize_quaternion(quaternion)
    return rotate_vector((-q[0], -q[1], -q[2], q[3]), vector)


def _append_collision_primitive(
    primitives,
    model_name,
    link_name,
    collision_name,
    tags,
    pose,
    geometry,
):
    geometry_type = str(geometry["type"])
    if geometry_type not in ("box", "cylinder"):
        return
    primitives.append({
        "id": _safe_id(f"{model_name}_{link_name}_{collision_name}"),
        "model": str(model_name),
        "link": str(link_name),
        "collision": str(collision_name),
        "tags": sorted(tags),
        "pose": tuple(float(value) for value in pose),
        "geometry": _normalized_geometry(geometry),
    })


def _distance_to_primitive(point, primitive):
    pose = primitive["pose"]
    q = quaternion_from_rpy(*pose[3:])
    local = inverse_rotate_vector(q, (
        float(point[0]) - pose[0],
        float(point[1]) - pose[1],
        float(point[2]) - pose[2],
    ))
    geometry = primitive["geometry"]
    if geometry["type"] == "box":
        half = [dimension * 0.5 for dimension in geometry["size"]]
        delta = [abs(local[index]) - half[index] for index in range(3)]
        outside = [max(value, 0.0) for value in delta]
        outside_distance = math.sqrt(sum(value * value for value in outside))
        if outside_distance > 0.0:
            return outside_distance
        return max(delta)

    radius = geometry["radius"]
    half_length = geometry["length"] * 0.5
    radial = math.sqrt(local[0] * local[0] + local[1] * local[1]) - radius
    axial = abs(local[2]) - half_length
    outside_distance = math.sqrt(max(radial, 0.0) ** 2 + max(axial, 0.0) ** 2)
    if outside_distance > 0.0:
        return outside_distance
    return max(radial, axial)


def _normalized_geometry(geometry):
    geometry_type = str(geometry["type"])
    if geometry_type == "box":
        return {
            "type": "box",
            "size": [float(value) for value in geometry["size"]],
        }
    if geometry_type == "cylinder":
        return {
            "type": "cylinder",
            "radius": float(geometry["radius"]),
            "length": float(geometry["length"]),
        }
    raise RuntimeError(f"Unsupported validation collision geometry: {geometry_type}")


def _pose_value(value, default):
    if value is None:
        return default
    if len(value) != 6:
        raise RuntimeError(f"pose must contain 6 values: {value}")
    return tuple(float(item) for item in value)


def _sensor_overrides_text(value):
    if value is None:
        return ""
    if isinstance(value, str):
        return value
    if isinstance(value, dict):
        return ",".join(
            f"{name}={'true' if bool(enabled) else 'false'}"
            for name, enabled in sorted(value.items())
        )
    raise RuntimeError("sensor_overrides must be a string or mapping")


def _resolve_scene_ref(spec, field_name):
    if isinstance(spec, dict):
        package_name = spec.get("package")
        if not package_name:
            raise RuntimeError(f"{field_name}.package is required")
        return os.path.join(
            _package_share_directory(package_name),
            spec.get("path", ""),
        )
    if isinstance(spec, str) and spec:
        return spec
    raise RuntimeError(f"{field_name} must be a non-empty string or mapping")


def _bringup_share_dir():
    source_root = Path(__file__).resolve().parents[1]
    if (source_root / "config" / "validation_cases").exists():
        return source_root
    return Path(_package_share_directory("robot_sim_bringup"))


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


def _required_string(raw: Mapping[str, Any], key: str, path: Path) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value:
        raise RuntimeError(f"{key} must be a non-empty string: {path}")
    return value


def _required_mapping(raw: Mapping[str, Any], key: str, path: Path) -> Mapping[str, Any]:
    value = raw.get(key)
    if not isinstance(value, dict):
        raise RuntimeError(f"{key} must be a mapping: {path}")
    return value


def _safe_id(value):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_")
