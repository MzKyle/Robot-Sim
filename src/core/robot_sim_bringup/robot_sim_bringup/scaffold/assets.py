from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys
from typing import Any

import yaml


STANDARD_DIRS = (
    "robot_sim/profiles",
    "robot_sim/validation_cases",
    "robot_sim/suites",
    "robot_sim/data_sources",
    "robot_sim/adapters",
)


def build_common_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--package", required=True, help="ROS package name to create or update.")
    parser.add_argument("--name", required=True, help="Asset name.")
    parser.add_argument("--output", required=True, help="Directory where the package directory will be created.")
    return parser


def main_system(argv=None) -> int:
    parser = build_common_parser("Scaffold a robot_sim v4 system profile.")
    args = parser.parse_args(argv)
    return _run(lambda: scaffold_system(args))


def main_case(argv=None) -> int:
    parser = build_common_parser("Scaffold a robot_sim v4 validation case.")
    parser.add_argument("--system", default="minimal_system", help="Referenced system profile name.")
    args = parser.parse_args(argv)
    return _run(lambda: scaffold_case(args))


def main_suite(argv=None) -> int:
    parser = build_common_parser("Scaffold a robot_sim v4 validation suite.")
    parser.add_argument("--case", default="smoke_case", help="Validation case name to include.")
    args = parser.parse_args(argv)
    return _run(lambda: scaffold_suite(args))


def main_adapter(argv=None) -> int:
    parser = build_common_parser("Scaffold a robot_sim v4 adapter template.")
    parser.add_argument(
        "--adapter-type",
        default="process_supervisor",
        help="Adapter type to scaffold, for example process_supervisor or topic_replay.",
    )
    args = parser.parse_args(argv)
    return _run(lambda: scaffold_adapter(args))


def scaffold_system(args: Any) -> Path:
    package_dir = _ensure_package(args.package, args.output)
    path = package_dir / "robot_sim" / "profiles" / f"{_safe_id(args.name)}.yaml"
    _write_new_yaml(path, {
        "schema": 4,
        "kind": "system_profile",
        "name": _safe_id(args.name),
        "description": "Generic external system profile.",
        "metadata": {"domain": "external"},
        "system": {
            "type": "ros2_pipeline",
            "startup_delay_sec": 0.0,
            "processes": [],
        },
    })
    return path


def scaffold_case(args: Any) -> Path:
    package_dir = _ensure_package(args.package, args.output)
    system_name = _safe_id(args.system)
    system_path = package_dir / "robot_sim" / "profiles" / f"{system_name}.yaml"
    if not system_path.exists():
        _write_new_yaml(system_path, {
            "schema": 4,
            "kind": "system_profile",
            "name": system_name,
            "description": "Generic external system profile.",
            "metadata": {"domain": "external"},
            "system": {"type": "ros2_pipeline", "startup_delay_sec": 0.0, "processes": []},
        })
    case_name = _safe_id(args.name)
    path = package_dir / "robot_sim" / "validation_cases" / f"{case_name}.yaml"
    _write_new_yaml(path, {
        "schema": 4,
        "kind": "validation_case",
        "name": case_name,
        "description": "Generic external validation case.",
        "system": {"profile": system_name, "profile_package": _safe_id(args.package)},
        "actions": [{"name": "settle", "type": "sleep", "duration_sec": 0.1}],
        "assertions": [],
        "evaluators": [],
        "artifacts": {"rosbag": {"enabled": False}, "reports": ["md", "html", "json"]},
    })
    return path


def scaffold_suite(args: Any) -> Path:
    package_dir = _ensure_package(args.package, args.output)
    suite_name = _safe_id(args.name)
    path = package_dir / "robot_sim" / "suites" / f"{suite_name}.yaml"
    _write_new_yaml(path, {
        "schema": 4,
        "kind": "validation_suite",
        "name": suite_name,
        "description": "Generic external validation suite.",
        "cases": [{"case": _safe_id(args.case), "case_package": _safe_id(args.package)}],
        "execution": {"continue_on_failure": True},
    })
    return path


def scaffold_adapter(args: Any) -> Path:
    package_dir = _ensure_package(args.package, args.output)
    adapter_name = _safe_id(args.name)
    adapter_type = _safe_id(args.adapter_type)
    path = package_dir / "robot_sim" / "adapters" / f"{adapter_name}.yaml"
    adapter: dict[str, Any] = {
        "schema": 4,
        "kind": "adapter",
        "name": adapter_name,
        "description": "Generic external adapter template.",
        "type": adapter_type,
    }
    if adapter_type == "process_supervisor":
        adapter.update({
            "command": ["python3", "-c", "print('robot_sim adapter scaffold')"],
            "background": False,
            "required": True,
        })
    _write_new_yaml(path, adapter)
    return path


def _run(factory) -> int:
    try:
        path = factory()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(path)
    return 0


def _ensure_package(package_name: str, output_dir: str) -> Path:
    package = _safe_id(package_name)
    package_dir = Path(output_dir).expanduser().resolve() / package
    package_dir.mkdir(parents=True, exist_ok=True)
    for relative in STANDARD_DIRS:
        (package_dir / relative).mkdir(parents=True, exist_ok=True)
    _write_if_missing(package_dir / "package.xml", _package_xml(package))
    _write_if_missing(package_dir / "CMakeLists.txt", _cmake(package))
    return package_dir


def _write_new_yaml(path: Path, value: dict[str, Any]) -> None:
    if path.exists():
        raise RuntimeError(f"asset already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")


def _write_if_missing(path: Path, text: str) -> None:
    if not path.exists():
        path.write_text(text, encoding="utf-8")


def _package_xml(package: str) -> str:
    return f"""<?xml version=\"1.0\"?>
<package format=\"3\">
  <name>{package}</name>
  <version>0.0.0</version>
  <description>External robot_sim validation assets.</description>
  <maintainer email=\"user@example.com\">user</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""


def _cmake(package: str) -> str:
    return f"""cmake_minimum_required(VERSION 3.8)
project({package})

find_package(ament_cmake REQUIRED)

install(DIRECTORY robot_sim DESTINATION share/${{PROJECT_NAME}})

ament_package()
"""


def _safe_id(value: Any) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_")
    if not text:
        raise RuntimeError("name must contain at least one safe character")
    return text
