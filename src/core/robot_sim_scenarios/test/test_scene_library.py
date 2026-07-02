from pathlib import Path
import random
import shutil
import subprocess
import sys
from xml.etree import ElementTree as ET

import pytest
import yaml


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_sim_scenarios import build_world, load_scene
from robot_sim_scenarios.schema_validation import validate_config_schema


SCENE_NAMES = [
    "debug_empty",
    "tabletop_pick_place",
    "industrial_cell",
    "conveyor_sorting",
    "shelf_bin_picking",
]


def _parse_world(scene_name, tmp_path):
    scene = load_scene(scene_name)
    world_path = build_world(scene, output_dir=tmp_path)
    return scene, world_path, ET.parse(world_path).getroot().find("world")


def test_debug_empty_loads_core_interfaces():
    scene = load_scene("debug_empty")

    assert scene.name == "debug_empty"
    assert scene.robot_mount_pose == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert scene.workspace.frame == "world"
    assert scene.workspace.min_bounds == (-1.5, -1.5, 0.0)
    assert scene.workspace.max_bounds == (1.5, 1.5, 1.8)
    assert scene.objects == ()
    assert scene.ground["name"] == "ground_plane"
    assert scene.lights[0]["name"] == "sun"
    assert scene.world["gui"]["camera"]["pose"] == [-3.0, -2.2, 2.0, 0.0, 0.42, 0.62]


def test_tabletop_pick_place_loads_objects_and_regions():
    scene = load_scene("tabletop_pick_place")

    object_names = {scene_object.name for scene_object in scene.objects}
    assert "work_table" in object_names
    assert "pick_red_cube" in object_names
    assert "pick_green_cylinder" in object_names
    assert "place_target_area" in object_names
    assert scene.robot_mount_pose == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert scene.workspace.frame == "world"
    assert scene.workspace.min_bounds == (0.35, -0.65, 0.72)
    assert scene.workspace.max_bounds == (1.35, 0.65, 1.35)
    assert sorted(scene.regions) == ["pick_spawn", "place_target", "random_tabletop"]


def test_region_sampling_is_bounded_and_seeded():
    scene = load_scene("tabletop_pick_place")

    first = scene.sample_region("pick_spawn", rng=random.Random(7))
    second = scene.sample_region("pick_spawn", rng=random.Random(7))
    assert first == second

    region = scene.regions["pick_spawn"]
    assert region.min_bounds[0] <= first[0] <= region.max_bounds[0]
    assert region.min_bounds[1] <= first[1] <= region.max_bounds[1]
    assert region.min_bounds[2] <= first[2] <= region.max_bounds[2]
    assert first[3:] == (0.0, 0.0, 0.0)


def test_build_world_creates_parseable_sdf(tmp_path):
    scene = load_scene("tabletop_pick_place")
    world_path = build_world(scene, output_dir=tmp_path)

    assert world_path.exists()
    root = ET.parse(world_path).getroot()
    assert root.tag == "sdf"

    world = root.find("world")
    assert world is not None
    assert world.attrib["name"] == "tabletop_pick_place"

    model_names = {model.attrib["name"] for model in world.findall("model")}
    assert "ground_plane" in model_names
    assert "work_table" in model_names
    assert "pick_red_cube" in model_names
    assert "place_target_area" in model_names

    plugin_filenames = {plugin.attrib["filename"] for plugin in world.findall("plugin")}
    assert "gz-sim-sensors-system" in plugin_filenames

    gui = world.find("gui")
    gui_plugin_filenames = {plugin.attrib["filename"] for plugin in gui.findall("plugin")}
    assert "MinimalScene" in gui_plugin_filenames
    assert "GzSceneManager" in gui_plugin_filenames
    assert "InteractiveViewControl" in gui_plugin_filenames


def test_build_debug_empty_world_has_no_tabletop_objects(tmp_path):
    scene = load_scene("debug_empty")
    world_path = build_world(scene, output_dir=tmp_path)
    root = ET.parse(world_path).getroot()
    world = root.find("world")

    model_names = {model.attrib["name"] for model in world.findall("model")}
    assert model_names == {"ground_plane"}
    assert world.find("light").attrib["name"] == "sun"


def test_all_scene_yamls_load():
    scenes = [load_scene(scene_name) for scene_name in SCENE_NAMES]

    assert [scene.name for scene in scenes] == SCENE_NAMES
    assert load_scene("industrial_cell").robot_mount_pose == (0.0, 0.0, 0.18, 0.0, 0.0, 0.0)
    assert load_scene("conveyor_sorting").robot_mount_pose == (-0.25, -1.15, 0.18, 0.0, 0.0, 0.0)
    assert load_scene("shelf_bin_picking").robot_mount_pose == (-0.55, -1.05, 0.18, 0.0, 0.0, 0.0)


def test_all_scene_and_world_preset_schemas_are_v3():
    package_root = Path(__file__).resolve().parents[1]
    for path in sorted((package_root / "scenes").glob("*.yaml")):
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        validate_config_schema(raw, "scene.schema.json", "scene", path)
    for path in sorted((package_root / "world_presets").glob("*.yaml")):
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        validate_config_schema(raw, "world_preset.schema.json", "world_preset", path)


def test_scene_schema_rejects_v1_files(tmp_path):
    raw = yaml.safe_load((Path(__file__).resolve().parents[1] / "scenes" / "debug_empty.yaml").read_text(encoding="utf-8"))
    raw["schema"] = 1
    path = tmp_path / "legacy_scene.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")

    with pytest.raises(RuntimeError, match="schema v1/v2 is no longer supported|migrate_config"):
        validate_config_schema(raw, "scene.schema.json", "scene", path)


def test_scene_parameters_variants_and_generators_are_deterministic():
    first = load_scene("industrial_cell", variant="dense_obstacles")
    second = load_scene("industrial_cell", variant="dense_obstacles")

    generated_first = [
        scene_object
        for scene_object in first.objects
        if "generated" in scene_object.tags
    ]
    generated_second = [
        scene_object
        for scene_object in second.objects
        if "generated" in scene_object.tags
    ]

    assert len(generated_first) == 4
    assert [item.pose for item in generated_first] == [item.pose for item in generated_second]
    assert first.raw["_resolved_parameters"]["generated_obstacle_count"] == 4


def test_scene_parameter_override_moves_fixture_without_variant():
    scene = load_scene("industrial_cell", parameters={"fixture_x": 1.33, "fixture_y": -0.84})
    fixture = next(scene_object for scene_object in scene.objects if scene_object.name == "fixture_station")

    assert fixture.pose[:2] == pytest.approx((1.33, -0.84))


def test_new_scene_workspaces_and_regions_are_readable():
    industrial = load_scene("industrial_cell")
    conveyor = load_scene("conveyor_sorting")
    shelf = load_scene("shelf_bin_picking")

    assert industrial.workspace.frame == "world"
    assert industrial.workspace.min_bounds == (-1.6, -1.35, 0.35)
    assert industrial.workspace.max_bounds == (1.95, 1.55, 2.1)
    assert {"obstacle_test", "fixture_pick", "planning_goal"} <= set(industrial.regions)

    assert conveyor.workspace.min_bounds == (-0.35, -1.25, 0.45)
    assert conveyor.workspace.max_bounds == (1.8, 0.75, 1.65)
    assert {"conveyor_spawn", "pick_window", "left_sort_bin", "right_sort_bin"} <= set(conveyor.regions)

    assert shelf.workspace.min_bounds == (-0.45, -1.2, 0.45)
    assert shelf.workspace.max_bounds == (1.85, 1.2, 1.95)
    assert {"lower_left_bin_pick", "middle_left_bin_pick", "staging_place"} <= set(shelf.regions)


def test_new_scene_region_sampling_is_bounded_and_seeded():
    scene_regions = [
        ("industrial_cell", "planning_goal"),
        ("conveyor_sorting", "pick_window"),
        ("shelf_bin_picking", "lower_left_bin_pick"),
    ]

    for scene_name, region_name in scene_regions:
        scene = load_scene(scene_name)
        first = scene.sample_region(region_name, rng=random.Random(23))
        second = scene.sample_region(region_name, rng=random.Random(23))
        region = scene.regions[region_name]

        assert first == second
        assert region.min_bounds[0] <= first[0] <= region.max_bounds[0]
        assert region.min_bounds[1] <= first[1] <= region.max_bounds[1]
        assert region.min_bounds[2] <= first[2] <= region.max_bounds[2]
        assert first[3:] == region.orientation_rpy


def test_industrial_cell_contains_fence_obstacles_and_optional_include(tmp_path):
    scene, _, world = _parse_world("industrial_cell", tmp_path)

    object_names = {scene_object.name for scene_object in scene.objects}
    model_names = {model.attrib["name"] for model in world.findall("model")}
    include_names = {include.findtext("name") for include in world.findall("include")}

    assert "safety_fence" in object_names
    assert "planning_columns" in object_names
    assert "fixture_station" in object_names
    assert "optional_fuel_visual_cone" in object_names
    assert "safety_fence" in model_names
    assert "planning_columns" in model_names
    assert "optional_fuel_visual_cone" not in include_names


def test_optional_fuel_includes_can_be_enabled(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBOT_SIM_ENABLE_FUEL_INCLUDES", "1")
    _, _, world = _parse_world("industrial_cell", tmp_path)

    include_names = {include.findtext("name") for include in world.findall("include")}
    assert "optional_fuel_visual_cone" in include_names


def test_conveyor_sorting_contains_dynamic_systems(tmp_path):
    scene, _, world = _parse_world("conveyor_sorting", tmp_path)

    object_names = {scene_object.name for scene_object in scene.objects}
    assert "main_conveyor" in object_names
    assert "moving_parcel_red" in object_names
    assert "moving_parcel_blue" in object_names
    assert scene.raw["startup_commands"][0]["args"][-1] == "data: 0.45"

    model_plugins = {
        plugin.attrib["filename"]
        for model in world.findall("model")
        for plugin in model.findall("plugin")
    }
    assert "gz-sim-track-controller-system" in model_plugins
    assert "gz-sim-trajectory-follower-system" in model_plugins

    conveyor = next(model for model in world.findall("model") if model.attrib["name"] == "main_conveyor")
    assert conveyor.find("plugin/link").text == "base_link"


def test_shelf_bin_picking_contains_bins_clutter_and_depth_camera(tmp_path):
    scene, _, world = _parse_world("shelf_bin_picking", tmp_path)

    object_names = {scene_object.name for scene_object in scene.objects}
    assert "storage_shelf" in object_names
    assert "bin_lower_left" in object_names
    assert "bin_middle_left" in object_names
    assert "clutter_objects" in object_names
    assert "occlusion_panel" in object_names

    sensors = [
        sensor
        for model in world.findall("model")
        for link in model.findall("link")
        for sensor in link.findall("sensor")
    ]
    assert any(sensor.attrib["type"] == "depth_camera" for sensor in sensors)


def test_generated_new_worlds_are_valid_sdf_when_gz_is_available(tmp_path):
    gz = shutil.which("gz")
    if not gz:
        return

    for scene_name in ("industrial_cell", "conveyor_sorting", "shelf_bin_picking"):
        scene = load_scene(scene_name)
        world_path = build_world(scene, output_dir=tmp_path)
        result = subprocess.run([gz, "sdf", "-k", str(world_path)], check=False, text=True, capture_output=True)
        assert result.returncode == 0, result.stderr + result.stdout
