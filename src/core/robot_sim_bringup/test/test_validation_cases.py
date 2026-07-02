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
from robot_sim_bringup.validation_cases import (
    collision_primitives_from_scene,
    load_validation_case,
)


def test_industrial_profile_resolves_scene_world():
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
    assert case["mode"] == "full"
    assert case["layout"] == "single"
    assert case["timeout_sec"] == 120.0
    assert case["scene"].name == "industrial_cell"
    assert case["start_region"] == "planning_start"
    assert case["goal_region"] == "planning_goal"
    assert case["pass_criteria"]["required_sensor_min_hz"] == 1.0
    assert case["expected_topics"][0]["name"] == "/camera/color/image_raw"
    assert case["artifacts"]["rosbag"]["topic_group"] == "all"


def test_all_built_in_schema_files_are_v2():
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
