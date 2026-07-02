import json
from pathlib import Path
from typing import Any, Mapping


SCHEMA_VERSION = 3


def validate_config_schema(raw: Mapping[str, Any], schema_name: str, kind: str, path: Path | str) -> None:
    path = Path(path)
    if raw.get("schema") != SCHEMA_VERSION:
        detected = raw.get("schema")
        migrate_hint = (
            "Run: ros2 run robot_sim_bringup migrate_config --input "
            f"{path} --output <schema3.yaml>"
        )
        raise RuntimeError(
            f"{kind} schema must be {SCHEMA_VERSION}: {path}. "
            f"Detected schema: {detected!r}. schema v1/v2 is no longer supported. "
            + migrate_hint
        )
    if raw.get("kind") != kind:
        raise RuntimeError(f"{kind} YAML must define kind: {kind}: {path}")

    schema = _load_schema(schema_name)
    try:
        from jsonschema import Draft202012Validator
    except ImportError:
        _fallback_validate(raw, schema, path)
        return

    validator = Draft202012Validator(schema)
    errors = sorted(validator.iter_errors(raw), key=lambda error: list(error.path))
    if errors:
        error = errors[0]
        location = ".".join(str(item) for item in error.path) or "<root>"
        raise RuntimeError(f"{schema_name} validation failed at {location}: {error.message}: {path}")


def _load_schema(schema_name: str) -> Mapping[str, Any]:
    schema_path = _schema_dir() / schema_name
    with schema_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _schema_dir() -> Path:
    source_root = Path(__file__).resolve().parents[1]
    if (source_root / "schemas").exists():
        return source_root / "schemas"
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("robot_sim_bringup")) / "schemas"


def _fallback_validate(value: Any, schema: Mapping[str, Any], path: Path, location: str = "<root>") -> None:
    if "oneOf" in schema and not _fallback_one_of_matches(value, schema["oneOf"], path, location):
        raise RuntimeError(f"schema validation failed at {location}: value does not match any allowed shape: {path}")

    expected_type = schema.get("type")
    if expected_type and not _type_matches(value, expected_type):
        raise RuntimeError(f"schema validation failed at {location}: expected {expected_type}: {path}")

    if "const" in schema and value != schema["const"]:
        raise RuntimeError(f"schema validation failed at {location}: expected {schema['const']!r}: {path}")
    if "enum" in schema and value not in schema["enum"]:
        raise RuntimeError(f"schema validation failed at {location}: expected one of {schema['enum']}: {path}")

    if isinstance(value, Mapping):
        for key in schema.get("required", []):
            if key not in value:
                raise RuntimeError(f"schema validation failed at {location}: missing required key '{key}': {path}")
        properties = schema.get("properties", {})
        for key, child_schema in properties.items():
            if key in value:
                _fallback_validate(value[key], child_schema, path, f"{location}.{key}")
        additional = schema.get("additionalProperties")
        if isinstance(additional, Mapping):
            for key, child in value.items():
                if key not in properties:
                    _fallback_validate(child, additional, path, f"{location}.{key}")

    if isinstance(value, list) and isinstance(schema.get("items"), Mapping):
        for index, item in enumerate(value):
            _fallback_validate(item, schema["items"], path, f"{location}[{index}]")


def _fallback_one_of_matches(value: Any, options: list[Mapping[str, Any]], path: Path, location: str) -> bool:
    matches = 0
    for option in options:
        try:
            _fallback_validate(value, option, path, location)
            matches += 1
        except RuntimeError:
            pass
    return matches == 1


def _type_matches(value: Any, expected_type: str | list[str]) -> bool:
    if isinstance(expected_type, list):
        return any(_type_matches(value, item) for item in expected_type)
    if expected_type == "object":
        return isinstance(value, Mapping)
    if expected_type == "array":
        return isinstance(value, list)
    if expected_type == "string":
        return isinstance(value, str)
    if expected_type == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if expected_type == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if expected_type == "boolean":
        return isinstance(value, bool)
    return True
