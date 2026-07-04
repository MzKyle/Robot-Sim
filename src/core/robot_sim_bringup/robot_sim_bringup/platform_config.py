from __future__ import annotations

from itertools import product
from pathlib import Path
from typing import Any, Mapping

import yaml

from robot_sim_bringup.registry import (
    resolve_data_source_path,
    resolve_system_profile_path,
    resolve_validation_case_path,
    resolve_validation_suite_path,
)
from robot_sim_bringup.schema_validation import validate_config_schema


def load_platform_validation_case(
    name_or_path: str | Path,
    case_package: str = "",
    parameter_overrides: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    path = resolve_validation_case_path(name_or_path, case_package=case_package)
    raw = _load_yaml_mapping(path, "validation_case")
    validate_config_schema(raw, "validation_case_v4.schema.json", "validation_case", path)
    parameters = dict(parameter_overrides or {})
    raw = _apply_parameters(raw, parameters)

    system_spec = dict(raw.get("system", {}))
    profile_raw: dict[str, Any] = {}
    profile_path = ""
    profile_name = str(system_spec.get("profile", ""))
    if profile_name or system_spec.get("profile_file"):
        resolved = resolve_system_profile_path(
            profile_name,
            str(system_spec.get("profile_package", "")),
            str(system_spec.get("profile_file", "")),
        )
        profile_path = str(resolved)
        profile_raw = load_system_profile(resolved)["raw"]

    merged_system = _merge_system(profile_raw.get("system", {}), system_spec)
    inputs, data_sources = _normalize_inputs(raw.get("inputs", []) or [], path.parent, parameters)
    return {
        "schema": 4,
        "kind": "validation_case",
        "name": str(raw.get("name") or path.stem),
        "description": str(raw.get("description", "")),
        "path": str(path),
        "system_profile": profile_name,
        "system_profile_path": profile_path,
        "system": merged_system,
        "inputs": inputs,
        "data_sources": data_sources,
        "adapters": [_normalize_adapter(item) for item in raw.get("adapters", []) or []],
        "actions": [dict(item) for item in raw.get("actions", []) or []],
        "assertions": [dict(item) for item in raw.get("assertions", []) or []],
        "artifacts": _normalize_artifacts(raw.get("artifacts", {})),
        "parameters": parameters,
        "raw": raw,
    }


def load_system_profile(name_or_path: str | Path, profile_package: str = "") -> dict[str, Any]:
    path = resolve_system_profile_path(name_or_path, profile_package)
    raw = _load_yaml_mapping(path, "system_profile")
    validate_config_schema(raw, "system_profile.schema.json", "system_profile", path)
    return {
        "schema": 4,
        "kind": "system_profile",
        "name": str(raw.get("name") or path.stem),
        "description": str(raw.get("description", "")),
        "path": str(path),
        "system": _merge_system({}, raw.get("system", {})),
        "raw": raw,
    }


def load_data_source(
    name_or_path: str | Path | Mapping[str, Any],
    data_source_package: str = "",
    parameters: Mapping[str, Any] | None = None,
    base_dir: Path | None = None,
) -> dict[str, Any]:
    parameters = dict(parameters or {})
    if isinstance(name_or_path, Mapping) and name_or_path.get("schema") == 4:
        path = base_dir or Path.cwd()
        raw = dict(name_or_path)
        source_path = ""
    else:
        source_name = _data_source_name(name_or_path)
        source_package = data_source_package or _data_source_package(name_or_path)
        resolved = resolve_data_source_path(source_name, source_package)
        raw = _load_yaml_mapping(resolved, "data_source")
        source_path = str(resolved)
        path = resolved.parent
    validate_config_schema(raw, "data_source.schema.json", "data_source", source_path or path)
    raw = _apply_parameters(raw, parameters)
    normalized = _normalize_data_source(raw, Path(path), source_path)
    return normalized


def load_validation_suite(name_or_path: str | Path, suite_package: str = "") -> dict[str, Any]:
    path = resolve_validation_suite_path(name_or_path, suite_package)
    raw = _load_yaml_mapping(path, "validation_suite")
    validate_config_schema(raw, "validation_suite.schema.json", "validation_suite", path)
    return {
        "schema": 4,
        "kind": "validation_suite",
        "name": str(raw.get("name") or path.stem),
        "description": str(raw.get("description", "")),
        "path": str(path),
        "cases": list(raw.get("cases", []) or []),
        "matrix": dict(raw.get("matrix", {}) or {}),
        "execution": {
            "continue_on_failure": bool(raw.get("execution", {}).get("continue_on_failure", True)),
            "timeout_sec": float(raw.get("execution", {}).get("timeout_sec", 0.0) or 0.0),
        },
        "artifacts": dict(raw.get("artifacts", {}) or {}),
        "raw": raw,
    }


def expand_suite_cases(suite: Mapping[str, Any]) -> list[dict[str, Any]]:
    parameters = _matrix_parameters(suite.get("matrix", {}).get("parameters", {}))
    cases: list[dict[str, Any]] = []
    for case_spec in suite.get("cases", []) or []:
        if isinstance(case_spec, str):
            base = {"case": case_spec, "case_package": "", "parameters": {}}
        else:
            base = {
                "case": str(case_spec["case"]),
                "case_package": str(case_spec.get("case_package", "")),
                "parameters": dict(case_spec.get("parameters", {}) or {}),
            }
        for parameter_set in parameters:
            merged = dict(parameter_set)
            merged.update(base["parameters"])
            cases.append({
                "case": base["case"],
                "case_package": base["case_package"],
                "parameters": merged,
                "id_suffix": _parameter_suffix(merged),
            })
    return cases


def _normalize_inputs(
    raw_inputs: list[Any],
    case_dir: Path,
    parameters: Mapping[str, Any],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    inputs: list[dict[str, Any]] = []
    data_sources: list[dict[str, Any]] = []
    for item in raw_inputs:
        if isinstance(item, Mapping) and str(item.get("type", "")) == "data_source":
            source_spec = item.get("source", item.get("name", ""))
            package = str(item.get("package", ""))
            source = load_data_source(source_spec, package, parameters, case_dir)
            adapter = _data_source_to_adapter(source, item)
            inputs.append(_normalize_adapter(adapter))
            data_sources.append(_data_source_summary(source, adapter))
        else:
            inputs.append(_normalize_adapter(item))
    return inputs, data_sources


def _matrix_parameters(raw: Mapping[str, Any]) -> list[dict[str, Any]]:
    if not raw:
        return [{}]
    names = sorted(str(name) for name in raw)
    values = []
    for name in names:
        items = raw.get(name, [])
        values.append(items if isinstance(items, list) else [items])
    return [dict(zip(names, combination)) for combination in product(*values)]


def _parameter_suffix(parameters: Mapping[str, Any]) -> str:
    if not parameters:
        return ""
    parts = [f"{_safe_id(name)}-{_safe_id(value)}" for name, value in sorted(parameters.items())]
    return "_".join(parts)


def _merge_system(profile_system: Mapping[str, Any], case_system: Mapping[str, Any]) -> dict[str, Any]:
    profile_system = dict(profile_system or {})
    case_system = dict(case_system or {})
    env = dict(profile_system.get("env", {}) or {})
    env.update(dict(case_system.get("env", {}) or {}))
    processes = [
        dict(item)
        for item in profile_system.get("processes", []) or []
    ] + [
        dict(item)
        for item in case_system.get("processes", []) or []
    ]
    return {
        "type": str(case_system.get("type") or profile_system.get("type") or "ros2_pipeline"),
        "env": env,
        "startup_delay_sec": float(case_system.get("startup_delay_sec", profile_system.get("startup_delay_sec", 0.0)) or 0.0),
        "processes": [_normalize_process(item) for item in processes],
    }


def _normalize_process(item: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "name": str(item.get("name") or "process"),
        "command": [str(part) for part in item.get("command", [])],
        "background": bool(item.get("background", True)),
        "required": bool(item.get("required", True)),
        "timeout_sec": float(item.get("timeout_sec", 0.0) or 0.0),
        "env": {str(key): str(value) for key, value in dict(item.get("env", {}) or {}).items()},
    }


def _normalize_adapter(item: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(item)
    result["name"] = str(result.get("name") or result.get("type", "adapter"))
    result["type"] = str(result.get("type", ""))
    return result


def _normalize_data_source(raw: Mapping[str, Any], base_dir: Path, source_path: str) -> dict[str, Any]:
    result = dict(raw)
    result["name"] = str(result.get("name", "data_source"))
    result["type"] = str(result.get("type", ""))
    result["source_path"] = source_path
    for key in ("path", "bag", "video", "image"):
        if result.get(key):
            result[key] = str(_resolve_relative_path(str(result[key]), base_dir))
    if result.get("images"):
        result["images"] = [
            str(_resolve_relative_path(str(path), base_dir))
            for path in result.get("images", []) or []
        ]
    if result.get("glob") and result.get("path"):
        root = Path(str(result["path"]))
        result["images"] = [
            str(path)
            for path in sorted(root.glob(str(result["glob"])))
            if path.is_file()
        ]
    return result


def _data_source_to_adapter(source: Mapping[str, Any], input_spec: Mapping[str, Any]) -> dict[str, Any]:
    source_type = str(source.get("type", ""))
    adapter = {
        "name": str(input_spec.get("name") or source.get("name")),
        "required": bool(input_spec.get("required", True)),
    }
    if source_type in ("message_sequence", "json", "csv"):
        message_type = str(input_spec.get("message_type") or source.get("message_type", ""))
        topic = str(input_spec.get("topic") or source.get("topic", ""))
        adapter.update({
            "type": "joint_state_replay" if message_type == "sensor_msgs/msg/JointState" or topic == "/joint_states" else "topic_replay",
            "topic": topic,
            "message_type": message_type,
            "rate_hz": float(input_spec.get("rate_hz", source.get("rate_hz", 10.0))),
            "repeat": bool(input_spec.get("repeat", source.get("loop", source.get("repeat", False)))),
            "messages": [dict(item) for item in source.get("messages", []) or []],
            "path": source.get("path", ""),
            "records_key": source.get("records_key", ""),
            "field_map": dict(source.get("field_map", {}) or {}),
        })
        return {key: value for key, value in adapter.items() if value not in ("", [], {})}
    if source_type == "image_sequence":
        adapter.update({
            "type": "image_camera_replay",
            "image_topic": str(input_spec.get("image_topic") or source.get("image_topic", source.get("topic", "/image_raw"))),
            "camera_info_topic": str(input_spec.get("camera_info_topic") or source.get("camera_info_topic", "/camera_info")),
            "frame_id": str(input_spec.get("frame_id") or source.get("frame_id", "camera_optical_frame")),
            "rate_hz": float(input_spec.get("rate_hz", source.get("rate_hz", 10.0))),
            "repeat": bool(input_spec.get("repeat", source.get("loop", source.get("repeat", True)))),
            "images": [str(path) for path in source.get("images", []) or []],
            "camera_info": dict(source.get("camera_info", {}) or {}),
            "encoding": str(source.get("encoding", "bgr8")),
        })
        return adapter
    if source_type == "video":
        adapter.update({
            "type": "image_camera_replay",
            "image_topic": str(input_spec.get("image_topic") or source.get("image_topic", source.get("topic", "/image_raw"))),
            "camera_info_topic": str(input_spec.get("camera_info_topic") or source.get("camera_info_topic", "/camera_info")),
            "frame_id": str(input_spec.get("frame_id") or source.get("frame_id", "camera_optical_frame")),
            "rate_hz": float(input_spec.get("rate_hz", source.get("fps", source.get("rate_hz", 10.0)))),
            "repeat": bool(input_spec.get("repeat", source.get("loop", source.get("repeat", True)))),
            "video": str(source.get("path", "")),
            "camera_info": dict(source.get("camera_info", {}) or {}),
            "start_sec": float(source.get("start_sec", 0.0) or 0.0),
            "duration_sec": float(source.get("duration_sec", 0.0) or 0.0),
        })
        return adapter
    if source_type == "rosbag":
        adapter.update({
            "type": "rosbag_replay",
            "bag": str(source.get("path", "")),
            "topics": [str(topic) for topic in source.get("topics", []) or []],
            "clock": bool(source.get("clock", False)),
            "remap": dict(source.get("remap", {}) or {}),
        })
        return adapter
    raise RuntimeError(f"unsupported data_source type: {source_type}")


def _data_source_summary(source: Mapping[str, Any], adapter: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "name": str(source.get("name", "")),
        "type": str(source.get("type", "")),
        "source_path": str(source.get("source_path", "")),
        "path": str(source.get("path", "")),
        "topic": str(adapter.get("topic") or adapter.get("image_topic", "")),
        "message_type": str(adapter.get("message_type", "")),
        "rate_hz": adapter.get("rate_hz"),
        "records": len(source.get("messages", []) or source.get("images", []) or []),
        "adapter": str(adapter.get("type", "")),
    }


def _data_source_name(spec: str | Path | Mapping[str, Any]) -> str:
    if isinstance(spec, Mapping):
        return str(spec.get("name") or spec.get("path") or "")
    return str(spec)


def _data_source_package(spec: str | Path | Mapping[str, Any]) -> str:
    return str(spec.get("package", "")) if isinstance(spec, Mapping) else ""


def _resolve_relative_path(path_text: str, base_dir: Path) -> Path:
    path = Path(path_text).expanduser()
    if not path.is_absolute():
        path = base_dir / path
    return path.resolve()


def _normalize_artifacts(raw: Mapping[str, Any]) -> dict[str, Any]:
    raw = dict(raw or {})
    rosbag = dict(raw.get("rosbag", {}) or {})
    return {
        "rosbag": {
            "enabled": bool(rosbag.get("enabled", False)),
            "topics": [str(topic) for topic in rosbag.get("topics", []) or []],
            "duration_sec": float(rosbag.get("duration_sec", 5.0)),
            "compression": bool(rosbag.get("compression", False)),
        },
        "reports": [str(item) for item in raw.get("reports", ["md", "html", "json"])],
    }


def _apply_parameters(value: Any, parameters: Mapping[str, Any]) -> Any:
    if isinstance(value, str):
        result = value
        for key, parameter in parameters.items():
            result = result.replace("${" + str(key) + "}", str(parameter))
        return result
    if isinstance(value, list):
        return [_apply_parameters(item, parameters) for item in value]
    if isinstance(value, dict):
        return {key: _apply_parameters(item, parameters) for key, item in value.items()}
    return value


def _load_yaml_mapping(path: Path, label: str) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, dict):
        raise RuntimeError(f"{label} YAML must be a mapping: {path}")
    return raw


def _safe_id(value: Any) -> str:
    import re

    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_")
