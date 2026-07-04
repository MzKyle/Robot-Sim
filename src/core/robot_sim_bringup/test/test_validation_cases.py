import json
import os
from pathlib import Path
import subprocess
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
from robot_sim_bringup.migrate_config import migrate_mapping_to_v4
from robot_sim_bringup.module_runner import _matches_expectation
from robot_sim_bringup.platform_assertions import assign_fields, matches_expectation
from robot_sim_bringup.platform_adapter import (
    _proxy_request_fields,
    _proxy_response_fields,
    _stub_response_fields,
)
from robot_sim_bringup.platform_config import (
    expand_suite_cases,
    load_adapter_template,
    load_data_source,
    load_platform_validation_case,
    load_system_profile,
    load_validation_suite,
)
from robot_sim_bringup.run_case import CommandRunner, FAILURE, _record_rosbag, run_case
from robot_sim_bringup.run_suite import run_suite
from robot_sim_bringup.scaffold_robot import scaffold_robot
from robot_sim_bringup.scaffold_assets import (
    scaffold_adapter,
    scaffold_case,
    scaffold_suite,
    scaffold_system,
)
from robot_sim_bringup.task_runners import get_task_runner
from robot_sim_bringup.validation_cases import (
    collision_primitives_from_scene,
    load_validation_case,
)
from robot_sim_bringup.registry import (
    _candidate_roots,
    resolve_adapter_path,
    resolve_profile_path,
    resolve_validation_case_path,
    resolve_validation_suite_path,
    source_package_directory,
)


REPO_ROOT = PACKAGE_ROOT.parents[2]


def test_internal_package_imports_match_compat_wrappers():
    import importlib

    pairs = [
        ("robot_sim_bringup.registry", "robot_sim_bringup.common.registry", "resolve_validation_case_path"),
        ("robot_sim_bringup.schema_validation", "robot_sim_bringup.common.schema_validation", "validate_config_schema"),
        ("robot_sim_bringup.platform_config", "robot_sim_bringup.platform.config", "load_platform_validation_case"),
        ("robot_sim_bringup.platform_adapter", "robot_sim_bringup.platform.adapter", "adapter_dependencies"),
        ("robot_sim_bringup.platform_assertions", "robot_sim_bringup.platform.assertions", "assign_fields"),
        ("robot_sim_bringup.platform_runner", "robot_sim_bringup.platform.runner", "run_platform_case"),
        ("robot_sim_bringup.run_suite", "robot_sim_bringup.platform.run_suite", "run_suite"),
        ("robot_sim_bringup.sim_config_loader", "robot_sim_bringup.robot_domain.sim_config_loader", "load_sim_profile"),
        ("robot_sim_bringup.validation_cases", "robot_sim_bringup.robot_domain.validation_cases", "load_validation_case"),
        ("robot_sim_bringup.task_runners", "robot_sim_bringup.robot_domain.task_runners", "get_task_runner"),
        ("robot_sim_bringup.run_case", "robot_sim_bringup.robot_domain.run_case", "run_case"),
        ("robot_sim_bringup.module_adapter", "robot_sim_bringup.legacy_integrations.module_adapter", "adapter_dependencies"),
        ("robot_sim_bringup.module_runner", "robot_sim_bringup.legacy_integrations.module_runner", "_matches_expectation"),
        ("robot_sim_bringup.scaffold_assets", "robot_sim_bringup.scaffold.assets", "scaffold_case"),
        ("robot_sim_bringup.scaffold_robot", "robot_sim_bringup.scaffold.robot", "scaffold_robot"),
    ]
    for compat_name, internal_name, symbol in pairs:
        compat = importlib.import_module(compat_name)
        internal = importlib.import_module(internal_name)
        assert getattr(compat, symbol) is getattr(internal, symbol)


def test_cli_wrapper_help_smoke():
    env = dict(os.environ)
    env["PYTHONPATH"] = f"{PACKAGE_ROOT}:{SRC_ROOT / 'robot_sim_scenarios'}:{env.get('PYTHONPATH', '')}"
    commands = [
        [sys.executable, str(PACKAGE_ROOT / "scripts" / "run_case"), "--help"],
        [sys.executable, str(PACKAGE_ROOT / "scripts" / "run_suite"), "--help"],
        [sys.executable, str(PACKAGE_ROOT / "scripts" / "scaffold_system"), "--help"],
        [sys.executable, str(PACKAGE_ROOT / "scripts" / "scaffold_case"), "--help"],
        [sys.executable, str(PACKAGE_ROOT / "scripts" / "scaffold_suite"), "--help"],
        [sys.executable, str(PACKAGE_ROOT / "scripts" / "scaffold_adapter"), "--help"],
    ]
    for command in commands:
        result = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            check=False,
        )
        assert result.returncode == 0, result.stdout


def _write_gray_image(path: Path, value: int) -> None:
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    image = np.full((3, 4), value, dtype=np.uint8)
    assert cv2.imwrite(str(path), image)


def test_industrial_profile_resolves_scene_world(tmp_path, monkeypatch):
    fake_ros_gz = tmp_path / "ros_gz_sim"
    (fake_ros_gz / "launch").mkdir(parents=True)
    (fake_ros_gz / "launch" / "gz_sim.launch.py").write_text("", encoding="utf-8")

    import robot_sim_bringup.robot_domain.sim_config_loader as loader

    original_package_share = loader._package_share_directory
    monkeypatch.setattr(
        loader,
        "_package_share_directory",
        lambda name: str(fake_ros_gz) if name == "ros_gz_sim" else original_package_share(name),
    )

    profile_path = resolve_profile_path("fanuc_m20id12l_industrial_cell")
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

    assert "robot_sim_bringup.legacy_integrations.module_runner" in command
    assert "--validation-case" in command
    assert str(metrics_path) in command
    assert runner.business_actions(case) == [
        {
            "name": "start_dry_run",
            "type": "service_call",
            "service": "/welding_executor/start",
        }
    ]


def test_all_built_in_schema_files_validate():
    profile_dirs = [
        PACKAGE_ROOT / "config" / "sim_profiles",
        REPO_ROOT / "examples" / "robot_arm" / "robot_sim" / "profiles",
        REPO_ROOT / "examples" / "rm_vision" / "robot_sim" / "profiles",
    ]
    for path in sorted(path for directory in profile_dirs for path in directory.glob("*.yaml")):
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        schema_name = "system_profile.schema.json" if raw.get("kind") == "system_profile" else "sim_profile.schema.json"
        validate_config_schema(raw, schema_name, raw.get("kind", "profile"), path)
    case_dirs = [
        PACKAGE_ROOT / "config" / "validation_cases",
        REPO_ROOT / "examples" / "robot_arm" / "robot_sim" / "validation_cases",
        REPO_ROOT / "examples" / "rm_vision" / "robot_sim" / "validation_cases",
        REPO_ROOT / "integrations" / "welding" / "robot_sim" / "validation_cases",
    ]
    for path in sorted(path for directory in case_dirs for path in directory.glob("*.yaml")):
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        schema_name = "validation_case_v4.schema.json" if raw.get("schema") == 4 else "validation_case.schema.json"
        validate_config_schema(raw, schema_name, "validation_case", path)
    for path in sorted((PACKAGE_ROOT / "config" / "system_profiles").glob("*.yaml")):
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        validate_config_schema(raw, "system_profile.schema.json", "system_profile", path)
    suite_dirs = [
        PACKAGE_ROOT / "config" / "validation_suites",
        REPO_ROOT / "examples" / "rm_vision" / "robot_sim" / "suites",
    ]
    for path in sorted(path for directory in suite_dirs for path in directory.glob("*.yaml")):
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        validate_config_schema(raw, "validation_suite.schema.json", "validation_suite", path)
    data_source_dirs = [
        PACKAGE_ROOT / "config" / "data_sources",
        REPO_ROOT / "examples" / "rm_vision" / "robot_sim" / "data_sources",
    ]
    for path in sorted(path for directory in data_source_dirs for path in directory.glob("*.yaml")):
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        validate_config_schema(raw, "data_source.schema.json", "data_source", path)


def test_platform_v4_loads_system_profile_and_case():
    profile = load_system_profile("minimal_pipeline")
    case = load_platform_validation_case("generic_command_smoke", parameter_overrides={"run_label": "unit"})

    assert profile["schema"] == 4
    assert profile["system"]["type"] == "ros2_pipeline"
    assert case["schema"] == 4
    assert case["system_profile"] == "minimal_pipeline"
    assert case["actions"][0]["command"][-1].endswith("unit')")


def test_builtin_registry_resolves_migrated_domains():
    assert "examples/robot_arm" in str(resolve_profile_path("panda"))
    assert "examples/robot_arm" in str(resolve_validation_case_path("industrial_planning_goal"))
    assert "integrations/welding" in str(resolve_validation_case_path("weld_pre_positioning_scan_and_move"))
    assert "examples/rm_vision" in str(resolve_validation_suite_path("rm_vision_interface_smoke"))
    assert "examples/rm_vision" in str(resolve_adapter_path("rm_vision_odom_camera_tf"))


def test_adapter_ref_expands_builtin_template():
    adapter = load_adapter_template("rm_vision_odom_camera_tf")
    case = load_platform_validation_case("tracker_sim_target")

    assert adapter["type"] == "tf_static_publisher"
    assert case["inputs"][0]["type"] == "tf_static_publisher"
    assert case["inputs"][0]["transforms"][0]["child_frame_id"] == "camera_optical_frame"


def test_assign_fields_supports_nested_message_arrays():
    from geometry_msgs.msg import PoseArray

    msg = PoseArray()
    assign_fields(msg, {
        "header.frame_id": "odom",
        "poses": [{
            "position": {"x": 1.25, "y": -0.5, "z": 2.0},
            "orientation": {"w": 1.0},
        }],
    })

    assert msg.header.frame_id == "odom"
    assert len(msg.poses) == 1
    assert msg.poses[0].position.x == pytest.approx(1.25)
    assert msg.poses[0].orientation.w == pytest.approx(1.0)


def test_data_source_loader_and_case_input_expansion():
    source = load_data_source("example_message_sequence")
    case = load_platform_validation_case("generic_data_source_replay")

    assert source["name"] == "example_message_sequence"
    assert source["type"] == "message_sequence"
    assert source["messages"][0]["data"] == "READY"
    assert case["inputs"][0]["type"] == "topic_replay"
    assert case["inputs"][0]["topic"] == "/example/status"
    assert case["data_sources"][0]["adapter"] == "topic_replay"
    assert case["data_sources"][0]["records"] == 3


def test_service_source_loader_and_case_input_expansion():
    source = load_data_source("mock_lower_device_enable")
    case = load_platform_validation_case("generic_device_mock_smoke")

    assert source["type"] == "service_source"
    assert source["service"] == "/mock/lower_device/set_enabled"
    assert case["inputs"][0]["type"] == "service_stub"
    assert case["inputs"][0]["service_type"] == "std_srvs/srv/SetBool"
    assert case["inputs"][1]["type"] == "service_proxy"
    assert case["inputs"][1]["target_service"] == "/mock/lower_device/set_enabled"
    assert case["data_sources"][0]["adapter"] == "service_stub"
    assert case["data_sources"][0]["service"] == "/mock/lower_device/set_enabled"
    assert case["data_sources"][1]["adapter"] == "service_proxy"
    assert case["data_sources"][1]["service_type"] == "std_srvs/srv/SetBool"


def test_service_stub_response_selection_and_proxy_mapping_helpers():
    request = type("Request", (), {})()
    request.data = True
    response = type("Response", (), {})()
    response.success = True
    response.message = "lower device enabled"

    assert _stub_response_fields(
        {
            "responses": [
                {
                    "match": {"data": {"equals": True}},
                    "response": {"success": True, "message": "matched"},
                }
            ],
            "default_response": {"success": False, "message": "default"},
        },
        request,
        {"calls": 0},
    ) == {"success": True, "message": "matched"}
    assert _stub_response_fields(
        {
            "response_sequence": [
                {"success": False, "message": "first"},
                {"success": True, "message": "second"},
            ],
            "repeat": False,
        },
        request,
        {"calls": 1},
    ) == {"success": True, "message": "second"}
    assert _proxy_request_fields(request, {"request_map": {"data": "request.data"}}) == {"data": True}
    assert _proxy_response_fields(
        response,
        {"response_map": {"success": "response.success", "message": "response.message"}},
    ) == {"success": True, "message": "lower device enabled"}


def test_external_data_source_package_discovery(tmp_path, monkeypatch):
    package = tmp_path / "mock_data_pkg"
    data_dir = package / "robot_sim" / "data_sources"
    data_dir.mkdir(parents=True)
    (data_dir / "status.yaml").write_text(
        yaml.safe_dump({
            "schema": 4,
            "kind": "data_source",
            "name": "status",
            "type": "message_sequence",
            "topic": "/status",
            "message_type": "std_msgs/msg/String",
            "messages": [{"data": "OK"}],
        }),
        encoding="utf-8",
    )
    (package / "package.xml").write_text("<package format='3'><name>mock_data_pkg</name></package>", encoding="utf-8")
    monkeypatch.setattr(
        "robot_sim_bringup.common.registry.package_share_directory",
        lambda name: package if name == "mock_data_pkg" else PACKAGE_ROOT,
    )

    source = load_data_source("status", data_source_package="mock_data_pkg")

    assert source["topic"] == "/status"
    assert source["messages"][0]["data"] == "OK"


def test_external_suite_package_discovery_prefers_suites_dir(tmp_path, monkeypatch):
    package = tmp_path / "mock_suite_pkg"
    suite_dir = package / "robot_sim" / "suites"
    suite_dir.mkdir(parents=True)
    (suite_dir / "external_suite.yaml").write_text(
        yaml.safe_dump({
            "schema": 4,
            "kind": "validation_suite",
            "name": "external_suite",
            "description": "External suite.",
            "cases": ["generic_command_smoke"],
            "execution": {"continue_on_failure": True},
        }),
        encoding="utf-8",
    )
    (package / "package.xml").write_text("<package format='3'><name>mock_suite_pkg</name></package>", encoding="utf-8")
    monkeypatch.setattr(
        "robot_sim_bringup.common.registry.package_share_directory",
        lambda name: package if name == "mock_suite_pkg" else PACKAGE_ROOT,
    )

    suite = load_validation_suite("external_suite", suite_package="mock_suite_pkg")

    assert suite["path"].endswith("robot_sim/suites/external_suite.yaml")
    assert suite["cases"] == ["generic_command_smoke"]


def test_validation_suite_expands_matrix():
    suite = load_validation_suite("generic_platform_smoke")
    cases = expand_suite_cases(suite)

    assert [case["parameters"]["run_label"] for case in cases] == ["alpha", "beta", "alpha", "beta", "alpha", "beta"]
    assert cases[0]["case"] == "generic_command_smoke"
    assert cases[4]["case"] == "generic_device_mock_smoke"


def test_platform_assertion_predicates():
    assert matches_expectation("RUNNING", {"contains": "RUN"})[0] is True
    assert matches_expectation([1, 2, 3], {"len_min": 2, "len_max": 3})[0] is True
    assert matches_expectation(0.08, {"abs_max": 0.02})[0] is False


def test_run_case_executes_platform_v4_case(tmp_path):
    args = type("Args", (), {
        "case": "generic_command_smoke",
        "output_dir": str(tmp_path),
        "case_package": "",
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

    assert run_case(args, CommandRunner()) == 0
    run_dir = next(tmp_path.iterdir())
    metrics = yaml.safe_load((run_dir / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["schema"] == 4
    assert metrics["passed"] is True
    assert metrics["actions"][0]["name"] == "echo_command"
    assert (run_dir / "report.html").exists()


def test_run_case_executes_data_source_replay(tmp_path):
    args = type("Args", (), {
        "case": "generic_data_source_replay",
        "output_dir": str(tmp_path),
        "case_package": "",
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

    assert run_case(args, CommandRunner()) == 0
    run_dir = next(tmp_path.iterdir())
    metrics = yaml.safe_load((run_dir / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["passed"] is True
    assert metrics["data_sources"][0]["name"] == "example_message_sequence"
    assert metrics["assertions"][0]["ok"] is True


def test_run_case_executes_device_mock_smoke(tmp_path):
    args = type("Args", (), {
        "case": "generic_device_mock_smoke",
        "output_dir": str(tmp_path),
        "case_package": "",
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

    assert run_case(args, CommandRunner()) == 0
    run_dir = next(tmp_path.iterdir())
    metrics = yaml.safe_load((run_dir / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["passed"] is True
    assert [item["adapter"] for item in metrics["data_sources"]] == [
        "service_stub",
        "service_proxy",
        "service_proxy",
    ]
    assert metrics["assertions"][0]["ok"] is True
    assert metrics["assertions"][1]["ok"] is True
    assert "Endpoint" in (run_dir / "report.md").read_text(encoding="utf-8")


def _platform_args(case_path, tmp_path):
    return type("Args", (), {
        "case": str(case_path),
        "output_dir": str(tmp_path),
        "case_package": "",
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


def _write_platform_case(path: Path, evaluators):
    path.write_text(
        yaml.safe_dump({
            "schema": 4,
            "kind": "validation_case",
            "name": path.stem,
            "description": "Evaluator test case.",
            "system": {"type": "ros2_pipeline", "processes": []},
            "actions": [{"name": "settle", "type": "sleep", "duration_sec": 0.0}],
            "assertions": [],
            "evaluators": evaluators,
            "artifacts": {"rosbag": {"enabled": False}, "reports": ["md", "html", "json"]},
        }),
        encoding="utf-8",
    )


def _evaluator_command(payload, exit_code=0):
    script = (
        "import json, pathlib, sys; "
        "path = pathlib.Path(sys.argv[1]); "
        "path.parent.mkdir(parents=True, exist_ok=True); "
        f"path.write_text(json.dumps({payload!r}), encoding='utf-8'); "
        f"raise SystemExit({exit_code})"
    )
    return [sys.executable, "-c", script, "${evaluator_output}"]


def test_platform_evaluator_required_passes_and_reports(tmp_path):
    case_path = tmp_path / "evaluator_pass.yaml"
    _write_platform_case(case_path, [{
        "name": "oracle",
        "type": "command",
        "command": _evaluator_command({
            "passed": True,
            "summary": "within tolerance",
            "metrics": {"error_m": 0.001},
            "failures": [],
            "artifacts": [],
        }),
    }])

    assert run_case(_platform_args(case_path, tmp_path), CommandRunner()) == 0
    run_dir = next(path for path in tmp_path.iterdir() if path.is_dir())
    metrics = yaml.safe_load((run_dir / "metrics.json").read_text(encoding="utf-8"))
    report = (run_dir / "report.md").read_text(encoding="utf-8")

    assert metrics["passed"] is True
    assert metrics["evaluators"][0]["name"] == "oracle"
    assert metrics["evaluators"][0]["ok"] is True
    assert metrics["evaluators"][0]["metrics"]["error_m"] == pytest.approx(0.001)
    assert Path(metrics["evaluators"][0]["output"]).exists()
    assert "## Evaluators" in report
    assert "within tolerance" in report


def test_platform_evaluator_required_failure_fails_case(tmp_path):
    case_path = tmp_path / "evaluator_required_fail.yaml"
    _write_platform_case(case_path, [{
        "name": "oracle",
        "type": "command",
        "command": _evaluator_command({
            "passed": False,
            "summary": "outside tolerance",
            "metrics": {},
            "failures": ["error too large"],
            "artifacts": [],
        }),
        "required": True,
    }])

    assert run_case(_platform_args(case_path, tmp_path), CommandRunner()) == FAILURE
    run_dir = next(path for path in tmp_path.iterdir() if path.is_dir())
    metrics = yaml.safe_load((run_dir / "metrics.json").read_text(encoding="utf-8"))

    assert metrics["passed"] is False
    assert metrics["evaluators"][0]["ok"] is False
    assert "evaluator failed: oracle" in metrics["error"]


def test_platform_evaluator_optional_failure_does_not_fail_case(tmp_path):
    case_path = tmp_path / "evaluator_optional_fail.yaml"
    _write_platform_case(case_path, [{
        "name": "optional_oracle",
        "type": "command",
        "command": _evaluator_command({
            "passed": False,
            "summary": "diagnostic only",
            "metrics": {},
            "failures": ["optional mismatch"],
            "artifacts": [],
        }),
        "required": False,
    }])

    assert run_case(_platform_args(case_path, tmp_path), CommandRunner()) == 0
    run_dir = next(path for path in tmp_path.iterdir() if path.is_dir())
    metrics = yaml.safe_load((run_dir / "metrics.json").read_text(encoding="utf-8"))

    assert metrics["passed"] is True
    assert metrics["evaluators"][0]["required"] is False
    assert metrics["evaluators"][0]["ok"] is False


def test_platform_evaluator_malformed_output_fails(tmp_path):
    case_path = tmp_path / "evaluator_bad_output.yaml"
    script = (
        "import pathlib, sys; "
        "path = pathlib.Path(sys.argv[1]); "
        "path.parent.mkdir(parents=True, exist_ok=True); "
        "path.write_text('{bad json', encoding='utf-8')"
    )
    _write_platform_case(case_path, [{
        "name": "bad_output",
        "type": "command",
        "command": [sys.executable, "-c", script, "${evaluator_output}"],
    }])

    assert run_case(_platform_args(case_path, tmp_path), CommandRunner()) == FAILURE
    run_dir = next(path for path in tmp_path.iterdir() if path.is_dir())
    metrics = yaml.safe_load((run_dir / "metrics.json").read_text(encoding="utf-8"))

    assert metrics["evaluators"][0]["ok"] is False
    assert any("not valid JSON" in failure for failure in metrics["evaluators"][0]["failures"])


def test_platform_evaluator_missing_output_fails(tmp_path):
    case_path = tmp_path / "evaluator_missing_output.yaml"
    _write_platform_case(case_path, [{
        "name": "missing_output",
        "type": "command",
        "command": [sys.executable, "-c", "pass"],
    }])

    assert run_case(_platform_args(case_path, tmp_path), CommandRunner()) == FAILURE
    run_dir = next(path for path in tmp_path.iterdir() if path.is_dir())
    metrics = yaml.safe_load((run_dir / "metrics.json").read_text(encoding="utf-8"))

    assert metrics["evaluators"][0]["ok"] is False
    assert any("output was not created" in failure for failure in metrics["evaluators"][0]["failures"])


def test_platform_evaluator_missing_passed_field_fails(tmp_path):
    case_path = tmp_path / "evaluator_missing_passed.yaml"
    _write_platform_case(case_path, [{
        "name": "missing_passed",
        "type": "command",
        "command": _evaluator_command({
            "summary": "missing passed",
            "metrics": {},
            "failures": [],
            "artifacts": [],
        }),
    }])

    assert run_case(_platform_args(case_path, tmp_path), CommandRunner()) == FAILURE
    run_dir = next(path for path in tmp_path.iterdir() if path.is_dir())
    metrics = yaml.safe_load((run_dir / "metrics.json").read_text(encoding="utf-8"))

    assert metrics["evaluators"][0]["ok"] is False
    assert any("boolean field 'passed'" in failure for failure in metrics["evaluators"][0]["failures"])


def test_platform_evaluator_schema_requires_command(tmp_path):
    raw = {
        "schema": 4,
        "kind": "validation_case",
        "name": "bad_evaluator_schema",
        "description": "Bad evaluator schema.",
        "system": {"type": "ros2_pipeline"},
        "evaluators": [{"name": "missing_command", "type": "command"}],
        "artifacts": {"rosbag": {"enabled": False}, "reports": ["md", "html", "json"]},
    }
    path = tmp_path / "bad_evaluator_schema.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")

    with pytest.raises(RuntimeError, match="validation failed|command"):
        validate_config_schema(raw, "validation_case_v4.schema.json", "validation_case", path)


def test_run_suite_executes_matrix_and_writes_junit(tmp_path):
    args = type("Args", (), {
        "suite": "generic_platform_smoke",
        "suite_package": "",
        "output_dir": str(tmp_path),
        "timeout": None,
        "rosbag_duration": 0.0,
        "no_rosbag": True,
    })()

    assert run_suite(args, CommandRunner()) == 0
    suite_dir = next(tmp_path.iterdir())
    metrics = yaml.safe_load((suite_dir / "suite_metrics.json").read_text(encoding="utf-8"))
    assert metrics["case_count"] == 6
    assert metrics["passed"] is True
    assert (suite_dir / "junit.xml").exists()


def test_sim_profile_schema_rejects_legacy_world_source(tmp_path):
    source = resolve_profile_path("panda")
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
    source = resolve_validation_case_path("industrial_planning_goal")
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["schema"] = 2
    path = tmp_path / "legacy_case.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")

    with pytest.raises(RuntimeError, match="migrate_config"):
        validate_config_schema(raw, "validation_case.schema.json", "validation_case", path)


def test_migrate_validation_case_v3_to_v4_schema():
    source = resolve_validation_case_path("empty_motion")
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))

    migrated = migrate_mapping_to_v4(raw, "validation_case", source)

    assert migrated["schema"] == 4
    assert migrated["system"]["type"] == "robot_simulation"
    validate_config_schema(migrated, "validation_case_v4.schema.json", "validation_case", source)


def test_validation_case_schema_rejects_unknown_adapter(tmp_path):
    source = resolve_validation_case_path("weld_2d_lateral_correction_dry_run")
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
    source = resolve_validation_case_path("industrial_planning_goal")
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
    source = resolve_validation_case_path("empty_motion")
    (case_dir / "external_smoke.yaml").write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
    (package / "package.xml").write_text("<package format='3'><name>mock_robot_pkg</name></package>", encoding="utf-8")
    scenario_root = SRC_ROOT / "robot_sim_scenarios"
    monkeypatch.setattr(
        "robot_sim_bringup.common.registry.package_share_directory",
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


def test_scaffold_generic_assets_outputs_standard_package(tmp_path):
    base = {
        "package": "demo_asset_pkg",
        "output": str(tmp_path),
    }
    system_path = scaffold_system(type("Args", (), {**base, "name": "minimal_system"})())
    case_path = scaffold_case(type("Args", (), {**base, "name": "smoke_case", "system": "minimal_system"})())
    suite_path = scaffold_suite(type("Args", (), {**base, "name": "smoke_suite", "case": "smoke_case"})())
    adapter_path = scaffold_adapter(type("Args", (), {**base, "name": "smoke_adapter", "adapter_type": "process_supervisor"})())

    system_raw = yaml.safe_load(system_path.read_text(encoding="utf-8"))
    case_raw = yaml.safe_load(case_path.read_text(encoding="utf-8"))
    suite_raw = yaml.safe_load(suite_path.read_text(encoding="utf-8"))
    adapter_raw = yaml.safe_load(adapter_path.read_text(encoding="utf-8"))
    validate_config_schema(system_raw, "system_profile.schema.json", "system_profile", system_path)
    validate_config_schema(case_raw, "validation_case_v4.schema.json", "validation_case", case_path)
    validate_config_schema(suite_raw, "validation_suite.schema.json", "validation_suite", suite_path)
    assert adapter_raw["type"] == "process_supervisor"
    assert (tmp_path / "demo_asset_pkg" / "robot_sim" / "data_sources").is_dir()
    assert (tmp_path / "demo_asset_pkg" / "robot_sim" / "adapters").is_dir()


def test_rm_vision_interface_smoke_suite_runs_when_workspace_available(tmp_path):
    setup = Path(os.environ.get("RM_VISION_SETUP", "/home/kyle/RM/vision_dev/install/setup.bash"))
    if not setup.exists():
        pytest.skip(f"RM vision workspace setup not found: {setup}")

    command = (
        "set -e; "
        "source /opt/ros/humble/setup.bash; "
        f"source {setup}; "
        f"PYTHONPATH={PACKAGE_ROOT}:{SRC_ROOT / 'robot_sim_scenarios'}:$PYTHONPATH "
        "python3 -m robot_sim_bringup.run_suite "
        f"--suite rm_vision_interface_smoke --output-dir {tmp_path} --no-rosbag"
    )
    result = subprocess.run(
        ["bash", "-lc", command],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    assert result.returncode == 0, result.stdout
    suite_dir = next(tmp_path.iterdir())
    metrics = yaml.safe_load((suite_dir / "suite_metrics.json").read_text(encoding="utf-8"))
    assert metrics["passed"] is True
    assert metrics["case_count"] == 4
    assert metrics["failed_count"] == 0
