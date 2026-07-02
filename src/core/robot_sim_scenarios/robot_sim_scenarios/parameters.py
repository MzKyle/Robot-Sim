from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import random
import re
from typing import Any, Mapping


PARAM_PATTERN = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}")


def materialize_scene_config(
    raw: Mapping[str, Any],
    path: Path,
    variant: str = "",
    parameters: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    config = deepcopy(dict(raw))
    values = _parameter_defaults(config.get("parameters", {}), path)
    if variant:
        variants = config.get("variants", {})
        if variant not in variants:
            valid = ", ".join(sorted(str(name) for name in variants))
            raise RuntimeError(f"unknown scene variant '{variant}' in {path}. Valid variants: {valid}")
        values.update(_variant_parameters(variants[variant], variant, path))
    for name, value in (parameters or {}).items():
        if name not in values:
            valid = ", ".join(sorted(values))
            raise RuntimeError(f"unknown scene parameter '{name}' in {path}. Valid parameters: {valid}")
        values[str(name)] = value
    _validate_parameter_types(config.get("parameters", {}), values, path)

    config = _resolve_templates(config, values, path)
    generated = _generate_objects(config, path)
    if generated:
        config.setdefault("objects", [])
        config["objects"].extend(generated)
    config["_resolved_parameters"] = values
    config["_variant"] = variant
    return config


def _parameter_defaults(raw: Any, path: Path) -> dict[str, Any]:
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise RuntimeError(f"scene.parameters must be a mapping: {path}")
    result = {}
    for name, spec in raw.items():
        if isinstance(spec, dict):
            if "default" not in spec:
                raise RuntimeError(f"scene.parameters.{name}.default is required: {path}")
            result[str(name)] = spec["default"]
        elif isinstance(spec, (str, int, float, bool)):
            result[str(name)] = spec
        else:
            raise RuntimeError(f"scene.parameters.{name} must be scalar or mapping: {path}")
    return result


def _variant_parameters(raw: Any, variant: str, path: Path) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise RuntimeError(f"scene.variants.{variant} must be a mapping: {path}")
    params = raw.get("parameters", {})
    if params is None:
        return {}
    if not isinstance(params, dict):
        raise RuntimeError(f"scene.variants.{variant}.parameters must be a mapping: {path}")
    return {str(name): value for name, value in params.items()}


def _validate_parameter_types(raw: Any, values: Mapping[str, Any], path: Path) -> None:
    if not isinstance(raw, dict):
        return
    for name, spec in raw.items():
        if not isinstance(spec, dict) or "type" not in spec:
            continue
        expected = str(spec["type"])
        value = values.get(str(name))
        if expected == "string" and not isinstance(value, str):
            raise RuntimeError(f"scene parameter '{name}' must be a string: {path}")
        if expected == "boolean" and not isinstance(value, bool):
            raise RuntimeError(f"scene parameter '{name}' must be a boolean: {path}")
        if expected == "integer" and (not isinstance(value, int) or isinstance(value, bool)):
            raise RuntimeError(f"scene parameter '{name}' must be an integer: {path}")
        if expected == "number" and (not isinstance(value, (int, float)) or isinstance(value, bool)):
            raise RuntimeError(f"scene parameter '{name}' must be numeric: {path}")


def _resolve_templates(value: Any, parameters: Mapping[str, Any], path: Path) -> Any:
    if isinstance(value, dict):
        return {
            key: _resolve_templates(child, parameters, path)
            for key, child in value.items()
        }
    if isinstance(value, list):
        return [_resolve_templates(child, parameters, path) for child in value]
    if isinstance(value, str):
        full_match = PARAM_PATTERN.fullmatch(value)
        if full_match:
            return _parameter_value(parameters, full_match.group(1), path)

        def replace(match):
            return str(_parameter_value(parameters, match.group(1), path))

        return PARAM_PATTERN.sub(replace, value)
    return value


def _parameter_value(parameters: Mapping[str, Any], name: str, path: Path) -> Any:
    if name not in parameters:
        valid = ", ".join(sorted(parameters))
        raise RuntimeError(f"unknown scene parameter '{name}' in {path}. Valid parameters: {valid}")
    return parameters[name]


def _generate_objects(config: Mapping[str, Any], path: Path) -> list[dict[str, Any]]:
    generated: list[dict[str, Any]] = []
    for index, spec in enumerate(config.get("generators", []) or []):
        if not isinstance(spec, dict):
            raise RuntimeError(f"scene.generators[{index}] must be a mapping: {path}")
        if spec.get("enabled", True) is False:
            continue
        generator_type = spec.get("type")
        if generator_type == "random_boxes":
            generated.extend(_generate_random_boxes(spec, config, index, path))
            continue
        raise RuntimeError(f"unsupported scene generator type '{generator_type}': {path}")
    return generated


def _generate_random_boxes(
    spec: Mapping[str, Any],
    config: Mapping[str, Any],
    index: int,
    path: Path,
) -> list[dict[str, Any]]:
    count = int(spec.get("count", 0))
    if count < 0:
        raise RuntimeError(f"scene.generators[{index}].count must be non-negative: {path}")
    region_name = str(spec.get("region", ""))
    regions = config.get("regions", {})
    if region_name not in regions:
        valid = ", ".join(sorted(str(name) for name in regions))
        raise RuntimeError(
            f"scene.generators[{index}].region '{region_name}' is unknown. "
            f"Valid regions: {valid}: {path}"
        )
    region = regions[region_name]
    bounds = region["bounds"]
    min_bounds = [float(value) for value in bounds["min"]]
    max_bounds = [float(value) for value in bounds["max"]]
    orientation = [float(value) for value in region.get("orientation_rpy", [0.0, 0.0, 0.0])]
    rng = random.Random(int(spec.get("seed", 1)))
    z_value = spec.get("z")
    geometry = dict(spec.get("geometry", {}))
    material = dict(spec.get("material", {}))
    if "size" in spec:
        geometry["size"] = spec["size"]
    if geometry.get("type") != "box":
        raise RuntimeError(f"scene.generators[{index}] random_boxes geometry.type must be box: {path}")
    tags = [str(tag) for tag in spec.get("tags", [])]
    prefix = str(spec.get("name_prefix", f"generated_{index}"))

    objects = []
    for object_index in range(count):
        xyz = [
            rng.uniform(min_bounds[axis], max_bounds[axis])
            for axis in range(3)
        ]
        if z_value is not None:
            xyz[2] = float(z_value)
        scene_object = {
            "name": f"{prefix}_{object_index + 1:02d}",
            "type": "model",
            "static": bool(spec.get("static", True)),
            "collision": bool(spec.get("collision", True)),
            "visual_only": bool(spec.get("visual_only", False)),
            "pose": [*xyz, *orientation],
            "geometry": geometry,
            "material": material,
            "tags": tags,
        }
        if not scene_object["static"]:
            scene_object["inertial"] = _box_inertial(geometry)
        objects.append(scene_object)
    return objects


def _box_inertial(geometry: Mapping[str, Any]) -> dict[str, Any]:
    size = [float(value) for value in geometry.get("size", [0.05, 0.05, 0.05])]
    mass = 0.05
    x, y, z = size
    return {
        "mass": mass,
        "inertia": {
            "ixx": mass * (y * y + z * z) / 12.0,
            "ixy": 0.0,
            "ixz": 0.0,
            "iyy": mass * (x * x + z * z) / 12.0,
            "iyz": 0.0,
            "izz": mass * (x * x + y * y) / 12.0,
        },
    }
