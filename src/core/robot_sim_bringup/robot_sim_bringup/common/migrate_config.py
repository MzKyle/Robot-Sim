from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml


TASK_TYPE_BY_CASE_NAME = {
    "industrial_fixture_to_pallet": "fixture_to_pallet",
    "industrial_obstacle_clearance": "obstacle_clearance",
    "industrial_planning_goal": "obstacle_clearance",
}


def build_parser():
    parser = argparse.ArgumentParser(description="Migrate robot_sim YAML between supported schema versions.")
    parser.add_argument("--input", required=True, help="Input YAML file.")
    parser.add_argument("--output", required=True, help="Output YAML file. Use the input path with --in-place to overwrite.")
    parser.add_argument("--kind", default="auto", choices=("auto", "sim_profile", "scene", "world_preset", "validation_case"))
    parser.add_argument("--in-place", action="store_true", help="Allow output path to match input path.")
    parser.add_argument("--from", dest="from_version", default="", choices=("", "v2", "v3"), help="Optional source schema version.")
    parser.add_argument("--to", dest="to_version", default="v3", choices=("v3", "v4"), help="Target schema version.")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        migrate_file(Path(args.input), Path(args.output), args.kind, args.in_place, args.to_version)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


def migrate_file(
    input_path: Path,
    output_path: Path,
    kind: str = "auto",
    in_place: bool = False,
    to_version: str = "v3",
) -> None:
    input_path = input_path.expanduser().resolve()
    output_path = output_path.expanduser().resolve()
    if input_path == output_path and not in_place:
        raise RuntimeError("--in-place is required when output equals input")
    with input_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, dict):
        raise RuntimeError(f"YAML must be a mapping: {input_path}")
    if to_version == "v4":
        migrated = migrate_mapping_to_v4(raw, kind, input_path)
    elif raw.get("schema") == 3:
        migrated = raw
    elif raw.get("schema") == 2:
        migrated = migrate_mapping(raw, kind)
    else:
        raise RuntimeError(f"only schema v2 can be migrated to v3; got {raw.get('schema')!r}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(migrated, handle, sort_keys=False)


def migrate_mapping(raw: dict, kind: str = "auto") -> dict:
    result = dict(raw)
    detected_kind = result.get("kind")
    if kind == "auto":
        kind = detected_kind
    if kind != detected_kind:
        raise RuntimeError(f"kind mismatch: requested {kind}, YAML has {detected_kind}")
    result["schema"] = 3
    if kind == "sim_profile":
        _migrate_sim_profile(result)
    elif kind == "scene":
        _migrate_scene(result)
    elif kind == "world_preset":
        _migrate_world_preset(result)
    elif kind == "validation_case":
        _migrate_validation_case(result)
    else:
        raise RuntimeError(f"unsupported kind: {kind}")
    return result


def migrate_mapping_to_v4(raw: dict, kind: str = "auto", source_path: Path | None = None) -> dict:
    detected_kind = raw.get("kind")
    if kind == "auto":
        kind = detected_kind
    if kind != detected_kind:
        raise RuntimeError(f"kind mismatch: requested {kind}, YAML has {detected_kind}")
    if kind != "validation_case":
        raise RuntimeError("v4 migration currently supports validation_case only")
    if raw.get("schema") == 2:
        raw = migrate_mapping(raw, kind)
    if raw.get("schema") != 3:
        raise RuntimeError(f"only schema v3 validation_case can be migrated to v4; got {raw.get('schema')!r}")
    launch = raw.get("launch", {}) or {}
    artifacts = raw.get("artifacts", {}) or {}
    rosbag = artifacts.get("rosbag", {}) or {}
    return {
        "schema": 4,
        "kind": "validation_case",
        "name": str(raw.get("name", source_path.stem if source_path else "validation_case")),
        "description": str(raw.get("description", "")),
        "system": {
            "type": "robot_simulation",
            "legacy_schema": 3,
            "legacy_case_path": str(source_path or ""),
            "profile": str(launch.get("profile", "")),
            "profile_package": str(launch.get("profile_package", "")),
            "profile_file": str(launch.get("profile_file", "")),
            "mode": str(launch.get("mode", "")),
            "layout": str(launch.get("layout", "")),
            "timeout_sec": float(launch.get("timeout_sec", 120.0)),
            "legacy_adapters": [dict(adapter) for adapter in raw.get("adapters", []) or [] if isinstance(adapter, dict)],
        },
        "inputs": [],
        "adapters": [],
        "actions": [],
        "assertions": [],
        "artifacts": {
            "rosbag": {
                "enabled": bool(rosbag.get("enabled", True)),
                "topics": [str(topic) for topic in rosbag.get("extra_topics", []) or []],
                "duration_sec": 8.0,
                "compression": bool(rosbag.get("compression", False)),
            },
            "reports": [str(report) for report in artifacts.get("reports", ["md", "html"])],
        },
    }


def _migrate_sim_profile(raw: dict) -> None:
    name = str(raw.get("name", "robot"))
    sensors = list((raw.get("sensors") or {}).keys())
    raw.setdefault("metadata", {
        "package": "robot_sim_bringup",
        "robot_name": name,
    })
    raw.setdefault("capabilities", {
        "task_families": ["empty_motion", "obstacle_clearance"],
        "sensors": sensors,
    })
    moveit = raw.get("moveit", {})
    planning_group = "manipulator"
    tool_link = "tool0"
    if name == "panda":
        planning_group = "panda_arm"
        tool_link = "panda_link8"
    raw.setdefault("end_effector", {
        "planning_group": planning_group,
        "tool_link": tool_link,
        "base_frame": "world",
    })
    if name == "panda":
        raw["end_effector"].setdefault("gripper", {
            "controller": "gripper_controller",
            "open_positions": [0.04, 0.04],
            "closed_positions": [0.0, 0.0],
        })
    if moveit and "capabilities" in raw:
        raw["capabilities"].setdefault("task_families", ["empty_motion", "obstacle_clearance"])


def _migrate_scene(raw: dict) -> None:
    raw.setdefault("parameters", {})
    raw.setdefault("variants", {})
    raw.setdefault("generators", [])


def _migrate_world_preset(raw: dict) -> None:
    raw.setdefault("scenario", {
        "type": "generic",
        "task_family": "simulation",
    })


def _migrate_validation_case(raw: dict) -> None:
    task = raw.setdefault("task", {})
    if task.get("type") == "moveit_region_to_region":
        task["type"] = TASK_TYPE_BY_CASE_NAME.get(str(raw.get("name", "")), "obstacle_clearance")
    if task.get("type") == "fixture_to_pallet":
        task.setdefault("object", {
            "model": "workpiece",
            "attach_link": task.get("moveit", {}).get("target_link", "tool0"),
        })


if __name__ == "__main__":
    sys.exit(main())
