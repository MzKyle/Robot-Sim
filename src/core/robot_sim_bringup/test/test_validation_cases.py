from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PACKAGE_ROOT.parents[0]
REPO_ROOT = PACKAGE_ROOT.parents[2]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(SRC_ROOT / "robot_sim_scenarios"))

from robot_sim_bringup.migrate_config import migrate_mapping
from robot_sim_bringup.module_adapter import adapter_dependencies
from robot_sim_bringup.module_runner import _matches_expectation
from robot_sim_bringup.registry import resolve_profile_path, resolve_validation_case_path
from robot_sim_bringup.run_case import CommandRunner, FAILURE, run_case
from robot_sim_bringup.scaffold_robot import scaffold_robot
from robot_sim_bringup.schema_validation import validate_config_schema
from robot_sim_bringup.sim_config_loader import load_sim_profile
from robot_sim_bringup.sim_launch_builder import _spawner_arguments, _spawner_nodes
from robot_sim_bringup.sim_smoke_helper import (
    _frequency_from_stamps,
    _message_stamp_seconds,
)
from robot_sim_bringup.validation_cases import load_validation_case


def test_builtin_v3_profiles_and_cases_validate():
    profile_dirs = [
        REPO_ROOT / "examples" / "robot_arm" / "robot_sim" / "profiles",
    ]
    for path in sorted(path for directory in profile_dirs for path in directory.glob("*.yaml")):
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        validate_config_schema(raw, "sim_profile.schema.json", "sim_profile", path)

    case_dirs = [
        REPO_ROOT / "examples" / "robot_arm" / "robot_sim" / "validation_cases",
        REPO_ROOT / "integrations" / "welding" / "robot_sim" / "validation_cases",
    ]
    for path in sorted(path for directory in case_dirs for path in directory.glob("*.yaml")):
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        validate_config_schema(raw, "validation_case.schema.json", "validation_case", path)


def test_robot_domain_loads_profile_and_case():
    profile = load_sim_profile(profile_name="panda")
    case = load_validation_case("empty_motion")

    assert profile["name"] == "panda"
    assert case["name"] == "empty_motion"
    assert case["task_type"] == "empty_motion"


def test_builtin_registry_resolves_robot_assets():
    assert "examples/robot_arm" in str(resolve_profile_path("panda"))
    assert "examples/robot_arm" in str(resolve_validation_case_path("industrial_planning_goal"))
    assert "integrations/welding" in str(resolve_validation_case_path("weld_pre_positioning_scan_and_move"))


def test_run_case_rejects_schema4_case_with_migration_hint(tmp_path, capsys):
    case_path = tmp_path / "generic_v4.yaml"
    case_path.write_text(yaml.safe_dump({
        "schema": 4,
        "kind": "validation_case",
        "name": "generic_v4",
        "description": "Moved generic validation case.",
        "system": {"type": "ros2_pipeline"},
        "artifacts": {"rosbag": {"enabled": False}, "reports": ["md"]},
    }), encoding="utf-8")
    args = type("Args", (), {
        "case": str(case_path),
        "case_package": "",
        "output_dir": str(tmp_path),
        "profile": "",
        "profile_file": "",
        "profile_package": "",
        "scene": "",
        "scene_package": "",
        "scene_variant": "",
        "scene_param": [],
        "mode": None,
        "sensor_overrides": None,
        "timeout": None,
        "rosbag_duration": 0.0,
        "no_rosbag": True,
        "keep_sim": False,
    })()

    assert run_case(args, CommandRunner()) == FAILURE
    assert "robot_validation" in capsys.readouterr().err


def test_scaffold_robot_outputs_schema_v3_package(tmp_path):
    args = type("Args", (), {
        "package": "demo_robot_pkg",
        "robot_name": "demo_bot",
        "output": str(tmp_path),
        "planning_group": "manipulator",
        "tool_link": "tool0",
        "joint_names": ["joint_1", "joint_2", "joint_3"],
        "sensor_set": "camera,depth,lidar,imu",
        "with_gripper": "true",
    })()

    package_dir = scaffold_robot(args)
    profile_path = package_dir / "robot_sim" / "profiles" / "demo_bot.yaml"
    case_path = package_dir / "robot_sim" / "validation_cases" / "smoke_empty_motion.yaml"

    profile_raw = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    case_raw = yaml.safe_load(case_path.read_text(encoding="utf-8"))
    validate_config_schema(profile_raw, "sim_profile.schema.json", "sim_profile", profile_path)
    validate_config_schema(case_raw, "validation_case.schema.json", "validation_case", case_path)
    assert profile_raw["capabilities"]["task_families"][:2] == ["empty_motion", "obstacle_clearance"]
    assert not (package_dir / "robot_sim" / "data_sources").exists()
    assert not (package_dir / "robot_sim" / "adapters").exists()


def test_migrate_validation_case_v2_to_v3_schema():
    source = resolve_validation_case_path("empty_motion")
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["schema"] = 2

    migrated = migrate_mapping(raw, "validation_case")

    assert migrated["schema"] == 3
    validate_config_schema(migrated, "validation_case.schema.json", "validation_case", source)


def test_schema_v2_reports_migration_hint(tmp_path):
    source = resolve_validation_case_path("industrial_planning_goal")
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["schema"] = 2
    path = tmp_path / "legacy_case.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")

    with pytest.raises(RuntimeError, match="migrate_config"):
        validate_config_schema(raw, "validation_case.schema.json", "validation_case", path)


def test_adapter_registry_declares_dynamic_ros_dependencies():
    assert adapter_dependencies({"type": "tf_to_tcp_pos"}) == ["weld_interface/msg/TcpPos"]
    assert "weld_interface/srv/Scan3d" in adapter_dependencies({"type": "scan3d_service"})
    assert "std_srvs/srv/SetBool" in adapter_dependencies({"type": "loop_motion_services"})


def test_module_topic_expectation_predicates():
    assert _matches_expectation("RUNNING", {"contains": "RUN"})[0] is True
    assert _matches_expectation(0.012, {"abs_max": 0.02})[0] is True
    assert _matches_expectation(None, {"exists": True})[0] is False


def test_controller_spawners_share_one_group_activation_process():
    profile = load_sim_profile(profile_name="panda")

    nodes = _spawner_nodes(
        profile,
        "/controller_manager",
        profile["control"]["controllers_file"],
        {"use_gripper": False},
    )

    assert len(nodes) == 1
    arguments = _spawner_arguments(
        profile["control"]["spawners"][:2],
        "/controller_manager",
        profile["control"]["controllers_file"],
    )
    assert arguments[:2] == ["joint_state_broadcaster", "arm_controller"]
    assert "--activate-as-group" in arguments
    assert arguments[arguments.index("--switch-timeout") + 1] == "90.0"
    assert "-t" not in arguments


def test_sensor_frequency_prefers_simulation_message_stamps():
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=250_000_000))
    )

    assert _message_stamp_seconds(msg) == pytest.approx(12.25)
    assert _frequency_from_stamps([1.0, 1.1, 1.2]) == pytest.approx(10.0)
    assert _frequency_from_stamps([1.0, 1.0]) is None
