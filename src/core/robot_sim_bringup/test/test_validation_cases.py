import json
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
from robot_sim_bringup.module_adapter import (
    _image_message,
    _load_scan3d_frames,
    _point_cloud2_message,
    _select_scan3d_frame,
    adapter_dependencies,
)
from robot_sim_bringup.module_runner import _matches_expectation
from robot_sim_bringup.run_case import FAILURE, _record_rosbag, run_case
from robot_sim_bringup.scaffold_robot import scaffold_robot
from robot_sim_bringup.task_runners import get_task_runner
from robot_sim_bringup.validation_cases import (
    collision_primitives_from_scene,
    load_validation_case,
)
from robot_sim_bringup.registry import _candidate_roots, source_package_directory


def _write_gray_image(path: Path, value: int) -> None:
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    image = np.full((3, 4), value, dtype=np.uint8)
    assert cv2.imwrite(str(path), image)


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
    assert case["moveit"]["execute"] is True


def test_validation_case_parses_plan_only_moveit():
    case = load_validation_case("panda_pick_place")

    assert case["task_type"] == "pick_place"
    assert case["task_regions"] == ["pick_approach", "place_approach"]
    assert case["moveit"]["execute"] is False


def test_module_validation_case_loads_external_contract():
    case = load_validation_case("weld_pre_positioning_scan_and_move")

    assert case["task_type"] == "module_validation"
    assert case["module"]["launch"]["package"] == "coarse_localization_offline"
    assert [action["name"] for action in case["module"]["actions"]] == [
        "scan_and_detect",
        "move_to_detected",
    ]
    assert [adapter["type"] for adapter in case["adapters"]] == [
        "tf_to_tcp_pos",
        "scan3d_service",
        "moveit_pose_service",
        "loop_motion_services",
    ]
    scan_adapter = next(adapter for adapter in case["adapters"] if adapter["type"] == "scan3d_service")
    assert scan_adapter["name"] == "dataset_scan3d"
    assert scan_adapter["source"]["type"] == "dataset"
    assert scan_adapter["source"]["dataset_dir"] == "/home/kyle/sany/data/3dcamera_2d_img"
    assert scan_adapter["source"]["frame_policy"] == "first"
    assert scan_adapter["source"]["fallback_record_path"].endswith("scan3d_20260626_143237.json")
    assert case["expect"]["module"]["required_actions"] == [
        "scan_and_detect",
        "move_to_detected",
    ]


def test_scan3d_dataset_uses_real_image_and_real_cloud(tmp_path):
    np = pytest.importorskip("numpy")
    _write_gray_image(tmp_path / "1.png", 7)
    points = np.arange(3 * 4 * 3, dtype=np.float32).reshape((3, 4, 3))
    np.savez_compressed(tmp_path / "1.npz", points=points)

    frames, policy = _load_scan3d_frames({
        "type": "dataset",
        "dataset_dir": str(tmp_path),
        "image_glob": "*.png",
    })

    assert policy == "first"
    assert len(frames) == 1
    assert frames[0]["data_source"] == "real_image_real_cloud"
    assert frames[0]["image"].shape == (3, 4, 3)
    assert frames[0]["frame_info"]["point_cloud_source"] == "real"
    assert frames[0]["frame_info"]["image_path"].endswith("1.png")
    assert frames[0]["frame_info"]["point_cloud_path"].endswith("1.npz")
    assert np.allclose(frames[0]["points"], points)


def test_scan3d_dataset_synthesizes_cloud_when_only_image_exists(tmp_path):
    np = pytest.importorskip("numpy")
    _write_gray_image(tmp_path / "1.png", 17)

    frames, _policy = _load_scan3d_frames({
        "type": "dataset",
        "dataset_dir": str(tmp_path),
        "image_glob": "*.png",
        "x_span_m": 2.0,
        "y_span_m": 0.5,
        "z_m": 0.75,
    })

    frame = frames[0]
    assert frame["data_source"] == "real_image_synthetic_cloud"
    assert frame["image"].shape == (3, 4, 3)
    assert frame["points"].shape == (3, 4, 3)
    assert frame["frame_info"]["point_cloud_source"] == "synthetic"
    assert np.isclose(frame["points"][0, 0, 0], -1.0)
    assert np.isclose(frame["points"][-1, -1, 0], 1.0)
    assert np.isclose(frame["points"][0, 0, 1], -0.25)
    assert np.isclose(frame["points"][-1, -1, 1], 0.25)
    assert np.allclose(frame["points"][:, :, 2], 0.75)


def test_scan3d_dataset_falls_back_to_replay_when_no_images(tmp_path):
    np = pytest.importorskip("numpy")
    dataset_dir = tmp_path / "dataset"
    fallback_dir = tmp_path / "fallback"
    dataset_dir.mkdir()
    fallback_dir.mkdir()
    _write_gray_image(fallback_dir / "scan.png", 33)
    points = np.ones((3, 4, 3), dtype=np.float32)
    np.savez_compressed(fallback_dir / "scan.npz", points=points)
    record_path = fallback_dir / "scan.json"
    record_path.write_text(
        json.dumps({
            "image_path": "scan.png",
            "point_cloud_path": "scan.npz",
            "image_frame_id": "camera_frame",
            "point_cloud_frame_id": "camera_frame",
        }),
        encoding="utf-8",
    )

    frames, _policy = _load_scan3d_frames({
        "type": "dataset",
        "dataset_dir": str(dataset_dir),
        "fallback_record_path": str(record_path),
    })

    assert len(frames) == 1
    assert frames[0]["data_source"] == "fallback_replay"
    assert frames[0]["frame_info"]["record_path"].endswith("scan.json")
    assert frames[0]["point_cloud_frame_id"] == "camera_frame"


def test_scan3d_dataset_frame_policy_index_and_sequential(tmp_path):
    _write_gray_image(tmp_path / "1.png", 11)
    _write_gray_image(tmp_path / "2.png", 22)

    index_frames, index_policy = _load_scan3d_frames({
        "type": "dataset",
        "dataset_dir": str(tmp_path),
        "image_glob": "*.png",
        "frame_policy": "index",
        "frame_index": 1,
    })
    assert index_policy == "index"
    assert len(index_frames) == 1
    assert index_frames[0]["image_path"].endswith("2.png")

    sequential_frames, sequential_policy = _load_scan3d_frames({
        "type": "dataset",
        "dataset_dir": str(tmp_path),
        "image_glob": "*.png",
        "frame_policy": "sequential",
    })
    state = {"next": 0}
    assert sequential_policy == "sequential"
    assert _select_scan3d_frame(sequential_frames, sequential_policy, state)["image_path"].endswith("1.png")
    assert _select_scan3d_frame(sequential_frames, sequential_policy, state)["image_path"].endswith("2.png")
    assert _select_scan3d_frame(sequential_frames, sequential_policy, state)["image_path"].endswith("1.png")


def test_scan3d_messages_use_bgr8_and_matching_pointcloud_dimensions(tmp_path):
    from builtin_interfaces.msg import Time

    _write_gray_image(tmp_path / "1.png", 44)
    frames, _policy = _load_scan3d_frames({
        "type": "dataset",
        "dataset_dir": str(tmp_path),
        "image_glob": "*.png",
    })

    image_msg = _image_message(frames[0]["image"], Time(), "camera_frame")
    cloud_msg = _point_cloud2_message(frames[0]["points"], Time(), "camera_frame")

    assert image_msg.encoding == "bgr8"
    assert image_msg.height == 3
    assert image_msg.width == 4
    assert image_msg.step == 12
    assert cloud_msg.height == 3
    assert cloud_msg.width == 4
    assert cloud_msg.row_step == 48


def test_module_validation_runner_dispatches_module_runner(tmp_path):
    case = load_validation_case("weld_2d_lateral_correction_dry_run")
    runner = get_task_runner(case["task_type"])
    metrics_path = tmp_path / "validation_metrics.json"

    command = runner.validation_command(
        ["--profile", "ignored"],
        case["path"],
        metrics_path,
        12.0,
    )

    assert "robot_sim_bringup.module_runner" in command
    assert "--validation-case" in command
    assert str(metrics_path) in command
    assert runner.business_actions(case) == [
        {
            "name": "start_dry_run",
            "type": "service_call",
            "service": "/welding_executor/start",
        }
    ]


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


def test_validation_case_schema_rejects_unknown_adapter(tmp_path):
    source = PACKAGE_ROOT / "config" / "validation_cases" / "weld_2d_lateral_correction_dry_run.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["adapters"][0]["type"] = "not_an_adapter"
    path = tmp_path / "bad_module_case.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")

    with pytest.raises(RuntimeError, match="validation failed|not_an_adapter"):
        validate_config_schema(raw, "validation_case.schema.json", "validation_case", path)


def test_adapter_registry_declares_dynamic_ros_dependencies():
    assert adapter_dependencies({"type": "tf_to_tcp_pos"}) == ["weld_interface/msg/TcpPos"]
    assert "weld_interface/srv/Scan3d" in adapter_dependencies({"type": "scan3d_service"})
    assert "std_srvs/srv/SetBool" in adapter_dependencies({"type": "loop_motion_services"})


def test_module_topic_expectation_predicates():
    assert _matches_expectation("RUNNING", {"contains": "RUN"})[0] is True
    assert _matches_expectation(0.012, {"abs_max": 0.02})[0] is True
    assert _matches_expectation(None, {"exists": True})[0] is False
    assert _matches_expectation(0.08, {"max": 0.02})[0] is False


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
    metrics = yaml.safe_load((run_dir / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["steps"][0]["status"] == "FAIL"


def test_rosbag_command_omits_empty_launch_arguments(tmp_path):
    class FakeProcess:
        def poll(self):
            return 0

        def wait(self, timeout=None):
            return 0

    class FakeRunner:
        command = None

        def popen(self, command, log_path, env=None):
            self.command = command
            return FakeProcess()

    case = load_validation_case("empty_motion")
    runner = FakeRunner()
    _record_rosbag(
        runner,
        case,
        {
            "profile": "panda",
            "profile_file": "/tmp/profile.yaml",
            "layout": "single",
            "sensor_overrides": "",
        },
        tmp_path / "rosbag",
        tmp_path / "rosbag.log",
        0.0,
    )

    assert "sensor_overrides:=" not in runner.command
    assert "extra_topics:=" not in runner.command


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


def test_registry_source_scan_is_workspace_scoped():
    roots = list(_candidate_roots(Path(__file__).resolve()))

    assert Path("/") not in roots
    assert source_package_directory("robot_sim_bringup") == PACKAGE_ROOT


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
