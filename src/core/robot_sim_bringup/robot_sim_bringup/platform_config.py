from __future__ import annotations

from itertools import product
from pathlib import Path
from typing import Any, Mapping

import yaml

from robot_sim_bringup.registry import (
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
    return {
        "schema": 4,
        "kind": "validation_case",
        "name": str(raw.get("name") or path.stem),
        "description": str(raw.get("description", "")),
        "path": str(path),
        "system_profile": profile_name,
        "system_profile_path": profile_path,
        "system": merged_system,
        "inputs": [_normalize_adapter(item) for item in raw.get("inputs", []) or []],
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
