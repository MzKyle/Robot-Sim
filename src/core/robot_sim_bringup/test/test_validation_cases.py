from pathlib import Path
import sys

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PACKAGE_ROOT.parents[0]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(SRC_ROOT / "robot_sim_scenarios"))

from robot_sim_bringup.sim_config_loader import load_sim_profile
from robot_sim_bringup.schema_validation import validate_config_schema
from robot_sim_bringup.run_case import FAILURE, run_case
from robot_sim_bringup.scaffold_robot import scaffold_robot
from robot_sim_bringup.validation_cases import (
    collision_primitives_from_scene,
    load_validation_case,
)


def test_industrial_profile_resolves_scene_world(tmp_path, monkeypatch):
    fake_ros_gz = tmp_path / "ros_gz_sim"
    (fake_ros_gz / "launch").mkdir(parents=True)
    (fake_ros_gz / "launch" / "gz_sim.launch.py").write_text("", encoding="utf-8")

    import robot_sim_bringup.sim_config_loader as loader

    original_package_share = loader._package_share_directory
    monkeypatch.setattr(
        loader,
        "_package_share_directory",
        lambda name: str(fake_ros_gz) if name == "ros_gz_sim" else original_package_share(name),
    )

    profile_path = PACKAGE_ROOT / "config" / "sim_profiles" / "fanuc_m20id12l_industrial_cell.yaml"
    profile = load_sim_profile(
        profile_name="unused",
        profile_file=str(profile_path),
        require_moveit=True,
    )

    world = profile["worlds"]["single"]
    assert profile["name"] == "fanuc_m20id12l_industrial_cell"
    assert profile["robot"]["spawn_pose"]["z"] == 0.18
    assert world["source"] == "scene"
    assert world["name"] == "industrial_cell"
    assert Path(world["path"]).exists()
    assert world["scene"]["regions"]["planning_goal"]["frame"] == "world"


def test_validation_case_loads_scene_and_defaults():
    case = load_validation_case("industrial_planning_goal")

    assert case["profile"] == "fanuc_m20id12l_industrial_cell"
    assert case["task_type"] == "obstacle_clearance"
    assert case["mode"] == "full"
    assert case["layout"] == "single"
    assert case["timeout_sec"] == 120.0
    assert case["scene"].name == "industrial_cell"
    assert case["start_region"] == "planning_start"
    assert case["goal_region"] == "planning_goal"
    assert case["task_regions"] == ["planning_start", "planning_goal"]
    assert case["pass_criteria"]["required_sensor_min_hz"] == 1.0
    assert case["expected_topics"][0]["name"] == "/camera/color/image_raw"
    assert case["artifacts"]["rosbag"]["topic_group"] == "all"


def test_all_built_in_schema_files_are_v3():
    for path in sorted((PACKAGE_ROOT / "config" / "sim_profiles").glob("*.yaml")):
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        validate_config_schema(raw, "sim_profile.schema.json", "sim_profile", path)
    for path in sorted((PACKAGE_ROOT / "config" / "validation_cases").glob("*.yaml")):
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        validate_config_schema(raw, "validation_case.schema.json", "validation_case", path)


def test_sim_profile_schema_rejects_legacy_world_source(tmp_path):
    source = PACKAGE_ROOT / "config" / "sim_profiles" / "panda.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["worlds"]["single"] = {
        "scenario": {
            "package": "robot_sim_scenarios",
            "path": "world_presets/lab_demo.yaml",
        }
    }
    path = tmp_path / "bad_profile.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")

    with pytest.raises(RuntimeError, match="validation failed|allowed shape"):
        validate_config_schema(raw, "sim_profile.schema.json", "sim_profile", path)


def test_schema_v2_reports_migration_hint(tmp_path):
    raw = yaml.safe_load((PACKAGE_ROOT / "config" / "validation_cases" / "industrial_planning_goal.yaml").read_text(encoding="utf-8"))
    raw["schema"] = 2
    path = tmp_path / "legacy_case.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")

    with pytest.raises(RuntimeError, match="migrate_config"):
        validate_config_schema(raw, "validation_case.schema.json", "validation_case", path)


def test_validation_case_rejects_unknown_region(tmp_path):
    source = PACKAGE_ROOT / "config" / "validation_cases" / "industrial_planning_goal.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["task"]["goal_region"] = "missing_region"
    path = tmp_path / "bad_case.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")

    with pytest.raises(RuntimeError, match="missing_region"):
        load_validation_case(path)


def test_run_case_writes_failure_artifacts(tmp_path, monkeypatch):
    monkeypatch.setattr("robot_sim_bringup.run_case.shutil.which", lambda _name: None)
    args = type("Args", (), {
        "case": "industrial_fixture_to_pallet",
        "output_dir": str(tmp_path),
        "profile": "",
        "profile_file": "",
        "profile_package": "",
        "case_package": "",
        "scene": "",
        "scene_package": "",
        "scene_variant": "",
        "scene_param": [],
        "mode": "",
        "sensor_overrides": None,
        "timeout": None,
        "rosbag_duration": 0.0,
        "no_rosbag": False,
        "keep_sim": False,
    })()

    assert run_case(args, runner=None) == FAILURE
    run_dirs = list(tmp_path.iterdir())
    assert len(run_dirs) == 1
    run_dir = run_dirs[0]
    assert (run_dir / "manifest.json").exists()
    assert (run_dir / "effective_case.yaml").exists()
    assert (run_dir / "effective_profile.yaml").exists()
    assert (run_dir / "metrics.json").exists()
    assert (run_dir / "report.md").exists()
    assert (run_dir / "report.html").exists()


def test_scene_collision_primitives_compose_model_link_and_collision_poses():
    case = load_validation_case("industrial_planning_goal")
    primitives = collision_primitives_from_scene(case["scene"], case["planning_scene"])
    by_id = {primitive["id"]: primitive for primitive in primitives}

    assert "robot_mount_pedestal_link_collision" not in by_id
    assert "planning_goal_marker_link_collision" not in by_id
    assert by_id["safety_fence_west_panel_collision"]["pose"][:3] == pytest.approx(
        (-1.7, 0.0, 0.82)
    )
    assert by_id["fixture_station_tabletop_collision"]["geometry"] == {
        "type": "box",
        "size": [1.25, 0.75, 0.08],
    }
    assert by_id["planning_columns_near_column_collision"]["geometry"] == {
        "type": "cylinder",
        "radius": 0.12,
        "length": 0.72,
    }


def test_external_case_package_discovery(tmp_path, monkeypatch):
    package = tmp_path / "mock_robot_pkg"
    case_dir = package / "robot_sim" / "validation_cases"
    case_dir.mkdir(parents=True)
    source = PACKAGE_ROOT / "config" / "validation_cases" / "empty_motion.yaml"
    (case_dir / "external_smoke.yaml").write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
    (package / "package.xml").write_text("<package format='3'><name>mock_robot_pkg</name></package>", encoding="utf-8")
    scenario_root = SRC_ROOT / "robot_sim_scenarios"
    monkeypatch.setattr(
        "robot_sim_bringup.registry.package_share_directory",
        lambda name: package if name == "mock_robot_pkg" else scenario_root,
    )

    case = load_validation_case("external_smoke", case_package="mock_robot_pkg")

    assert case["name"] == "empty_motion"
    assert case["task_type"] == "empty_motion"


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
