from pathlib import Path
from typing import Any, Mapping

import yaml

from robot_sim_scenarios.models import (
    Region,
    Scene,
    SceneObject,
    Workspace,
    pose_from_sequence,
    vector3_from_sequence,
)
from robot_sim_scenarios.parameters import materialize_scene_config
from robot_sim_scenarios.schema_validation import validate_config_schema


def load_scene(
    name_or_path: str | Path,
    variant: str = "",
    parameters: Mapping[str, Any] | None = None,
) -> Scene:
    path = _resolve_scene_path(name_or_path)
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)

    if not isinstance(raw, dict):
        raise RuntimeError(f"scene YAML must be a mapping: {path}")
    raw = materialize_scene_config(raw, path, variant=variant, parameters=parameters)
    validate_config_schema(raw, "scene.schema.json", "scene", path)
    if "startup_commands" in raw:
        _validate_startup_commands(
            _required_list(raw, "startup_commands", path),
            "startup_commands",
            path,
        )

    name = _required_string(raw, "name", path)
    world = _required_mapping(raw, "world", path)
    ground = _required_mapping(raw, "ground", path)
    lights = _required_list(raw, "lights", path)
    objects = _required_list(raw, "objects", path)
    regions = _required_mapping(raw, "regions", path)

    _validate_world(world, "world", path)
    _validate_renderable(ground, "ground", path)
    for index, light in enumerate(lights):
        _validate_light(light, f"lights[{index}]", path)

    scene_objects = tuple(
        _normalize_object(item, f"objects[{index}]", path)
        for index, item in enumerate(objects)
    )

    workspace = _normalize_workspace(
        _required_mapping(raw, "workspace", path),
        "workspace",
        path,
    )
    normalized_regions = {
        str(region_name): _normalize_region(str(region_name), region, f"regions.{region_name}", path)
        for region_name, region in regions.items()
    }

    return Scene(
        name=name,
        path=path,
        description=str(raw.get("description", "")),
        raw=raw,
        world=world,
        ground=ground,
        lights=tuple(lights),
        robot_mount_pose=pose_from_sequence(
            _required_list(raw, "robot_mount_pose", path),
            "robot_mount_pose",
        ),
        workspace=workspace,
        objects=scene_objects,
        regions=normalized_regions,
    )


def _resolve_scene_path(name_or_path: str | Path) -> Path:
    candidate = Path(name_or_path).expanduser()
    if candidate.exists():
        return candidate.resolve()

    if candidate.suffix in (".yaml", ".yml") or candidate.parent != Path("."):
        raise RuntimeError(f"scene file does not exist: {candidate}")

    scene_name = str(name_or_path)
    scene_path = _package_share_dir() / "scenes" / f"{scene_name}.yaml"
    if not scene_path.exists():
        raise RuntimeError(f"unknown scene '{scene_name}': {scene_path}")
    return scene_path.resolve()


def _package_share_dir() -> Path:
    source_root = Path(__file__).resolve().parents[1]
    if (source_root / "scenes").exists():
        return source_root

    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("robot_sim_scenarios"))
    except Exception:
        return source_root


def _normalize_workspace(raw: Mapping[str, Any], field_name: str, path: Path) -> Workspace:
    frame = _required_string(raw, "frame", path, field_name)
    bounds = _required_mapping(raw, "bounds", path, field_name)
    return Workspace(
        frame=frame,
        min_bounds=vector3_from_sequence(_required_list(bounds, "min", path, f"{field_name}.bounds"), f"{field_name}.bounds.min"),
        max_bounds=vector3_from_sequence(_required_list(bounds, "max", path, f"{field_name}.bounds"), f"{field_name}.bounds.max"),
        raw=raw,
    )


def _normalize_region(name: str, raw: Any, field_name: str, path: Path) -> Region:
    if not isinstance(raw, dict):
        raise RuntimeError(f"{field_name} must be a mapping: {path}")
    bounds = _required_mapping(raw, "bounds", path, field_name)
    return Region(
        name=name,
        frame=_required_string(raw, "frame", path, field_name),
        min_bounds=vector3_from_sequence(_required_list(bounds, "min", path, f"{field_name}.bounds"), f"{field_name}.bounds.min"),
        max_bounds=vector3_from_sequence(_required_list(bounds, "max", path, f"{field_name}.bounds"), f"{field_name}.bounds.max"),
        orientation_rpy=vector3_from_sequence(
            _required_list(raw, "orientation_rpy", path, field_name),
            f"{field_name}.orientation_rpy",
        ),
        raw=raw,
    )


def _normalize_object(raw: Any, field_name: str, path: Path) -> SceneObject:
    if not isinstance(raw, dict):
        raise RuntimeError(f"{field_name} must be a mapping: {path}")
    _validate_object(raw, field_name, path)
    tags = _required_list(raw, "tags", path, field_name)
    return SceneObject(
        name=_required_string(raw, "name", path, field_name),
        object_type=_required_string(raw, "type", path, field_name),
        pose=pose_from_sequence(_required_list(raw, "pose", path, field_name), f"{field_name}.pose"),
        geometry=raw.get("geometry", {}),
        tags=tuple(str(tag) for tag in tags),
        raw=raw,
    )


def _validate_world(raw: Mapping[str, Any], field_name: str, path: Path) -> None:
    for key in ("name", "sdf_version"):
        _required_string(raw, key, path, field_name)
    for key in ("gravity", "magnetic_field"):
        vector3_from_sequence(_required_list(raw, key, path, field_name), f"{field_name}.{key}")
    atmosphere = _required_mapping(raw, "atmosphere", path, field_name)
    _required_string(atmosphere, "type", path, f"{field_name}.atmosphere")

    physics = _required_mapping(raw, "physics", path, field_name)
    for key in ("name", "type"):
        _required_string(physics, key, path, f"{field_name}.physics")
    for key in ("max_step_size", "real_time_factor"):
        _required_number(physics, key, path, f"{field_name}.physics")

    plugins = _required_list(raw, "plugins", path, field_name)
    for index, plugin in enumerate(plugins):
        _validate_plugin(plugin, f"{field_name}.plugins[{index}]", path)

    scene = _required_mapping(raw, "scene", path, field_name)
    for key in ("ambient", "background"):
        _float_list(_required_list(scene, key, path, f"{field_name}.scene"), f"{field_name}.scene.{key}", 3)
    _required_bool(scene, "grid", path, f"{field_name}.scene")

    gui = _required_mapping(raw, "gui", path, field_name)
    _required_number(gui, "fullscreen", path, f"{field_name}.gui")
    _validate_camera(
        _required_mapping(gui, "camera", path, f"{field_name}.gui"),
        f"{field_name}.gui.camera",
        path,
    )
    for index, plugin in enumerate(gui.get("plugins", [])):
        _validate_plugin(plugin, f"{field_name}.gui.plugins[{index}]", path)


def _validate_light(raw: Any, field_name: str, path: Path) -> None:
    if not isinstance(raw, dict):
        raise RuntimeError(f"{field_name} must be a mapping: {path}")
    for key in ("name", "type"):
        _required_string(raw, key, path, field_name)
    _required_bool(raw, "cast_shadows", path, field_name)
    pose_from_sequence(_required_list(raw, "pose", path, field_name), f"{field_name}.pose")
    vector3_from_sequence(_required_list(raw, "direction", path, field_name), f"{field_name}.direction")
    for key in ("diffuse", "specular"):
        _float_list(_required_list(raw, key, path, field_name), f"{field_name}.{key}", 4)
    attenuation = _required_mapping(raw, "attenuation", path, field_name)
    for key in ("range", "constant", "linear", "quadratic"):
        _required_number(attenuation, key, path, f"{field_name}.attenuation")


def _validate_renderable(raw: Mapping[str, Any], field_name: str, path: Path) -> None:
    _required_string(raw, "name", path, field_name)
    _required_bool(raw, "static", path, field_name)
    _required_bool(raw, "collision", path, field_name)
    _required_bool(raw, "visual_only", path, field_name)
    pose_from_sequence(_required_list(raw, "pose", path, field_name), f"{field_name}.pose")
    geometry = _required_mapping(raw, "geometry", path, field_name)
    _validate_geometry(geometry, f"{field_name}.geometry", path)
    _validate_material(_required_mapping(raw, "material", path, field_name), f"{field_name}.material", path)
    if not bool(raw["static"]) and "inertial" not in raw:
        raise RuntimeError(f"{field_name}.inertial is required for non-static objects: {path}")
    if "inertial" in raw:
        _validate_inertial(_required_mapping(raw, "inertial", path, field_name), f"{field_name}.inertial", path)


def _validate_object(raw: Mapping[str, Any], field_name: str, path: Path) -> None:
    object_type = _required_string(raw, "type", path, field_name)
    _required_string(raw, "name", path, field_name)
    _required_list(raw, "tags", path, field_name)
    pose_from_sequence(_required_list(raw, "pose", path, field_name), f"{field_name}.pose")

    if object_type == "include":
        _required_string(raw, "uri", path, field_name)
        if "static" in raw:
            _required_bool(raw, "static", path, field_name)
        if "optional" in raw:
            _required_bool(raw, "optional", path, field_name)
        return

    if "links" in raw:
        _required_bool(raw, "static", path, field_name)
        links = _required_list(raw, "links", path, field_name)
        if not links:
            raise RuntimeError(f"{field_name}.links must not be empty: {path}")
        for index, link in enumerate(links):
            _validate_link(link, f"{field_name}.links[{index}]", path)
        plugins = _required_list(raw, "plugins", path, field_name) if "plugins" in raw else []
        for index, plugin in enumerate(plugins):
            _validate_plugin(plugin, f"{field_name}.plugins[{index}]", path)
        return

    _validate_renderable(raw, field_name, path)
    plugins = _required_list(raw, "plugins", path, field_name) if "plugins" in raw else []
    for index, plugin in enumerate(plugins):
        _validate_plugin(plugin, f"{field_name}.plugins[{index}]", path)


def _validate_link(raw: Any, field_name: str, path: Path) -> None:
    if not isinstance(raw, dict):
        raise RuntimeError(f"{field_name} must be a mapping: {path}")
    _required_string(raw, "name", path, field_name)
    pose_from_sequence(_required_list(raw, "pose", path, field_name), f"{field_name}.pose")

    if "inertial" in raw:
        _validate_inertial(_required_mapping(raw, "inertial", path, field_name), f"{field_name}.inertial", path)
    if "gravity" in raw:
        _required_bool(raw, "gravity", path, field_name)
    if "kinematic" in raw:
        _required_bool(raw, "kinematic", path, field_name)

    has_simple_shape = "geometry" in raw
    if has_simple_shape:
        _required_bool(raw, "collision", path, field_name)
        _required_bool(raw, "visual_only", path, field_name)
        _validate_geometry(_required_mapping(raw, "geometry", path, field_name), f"{field_name}.geometry", path)
        _validate_material(_required_mapping(raw, "material", path, field_name), f"{field_name}.material", path)

    collisions = _required_list(raw, "collisions", path, field_name) if "collisions" in raw else []
    visuals = _required_list(raw, "visuals", path, field_name) if "visuals" in raw else []
    sensors = _required_list(raw, "sensors", path, field_name) if "sensors" in raw else []

    for index, collision in enumerate(collisions):
        _validate_collision(collision, f"{field_name}.collisions[{index}]", path)
    for index, visual in enumerate(visuals):
        _validate_visual(visual, f"{field_name}.visuals[{index}]", path)
    for index, sensor in enumerate(sensors):
        _validate_sensor(sensor, f"{field_name}.sensors[{index}]", path)

    if not has_simple_shape and not any((collisions, visuals, sensors)):
        raise RuntimeError(
            f"{field_name} must define geometry, collisions, visuals, or sensors: {path}"
        )


def _validate_collision(raw: Any, field_name: str, path: Path) -> None:
    if not isinstance(raw, dict):
        raise RuntimeError(f"{field_name} must be a mapping: {path}")
    _required_string(raw, "name", path, field_name)
    pose_from_sequence(_required_list(raw, "pose", path, field_name), f"{field_name}.pose")
    _validate_geometry(_required_mapping(raw, "geometry", path, field_name), f"{field_name}.geometry", path)
    if "surface" in raw:
        _required_mapping(raw, "surface", path, field_name)


def _validate_visual(raw: Any, field_name: str, path: Path) -> None:
    if not isinstance(raw, dict):
        raise RuntimeError(f"{field_name} must be a mapping: {path}")
    _required_string(raw, "name", path, field_name)
    pose_from_sequence(_required_list(raw, "pose", path, field_name), f"{field_name}.pose")
    _validate_geometry(_required_mapping(raw, "geometry", path, field_name), f"{field_name}.geometry", path)
    _validate_material(_required_mapping(raw, "material", path, field_name), f"{field_name}.material", path)


def _validate_sensor(raw: Any, field_name: str, path: Path) -> None:
    if not isinstance(raw, dict):
        raise RuntimeError(f"{field_name} must be a mapping: {path}")
    _required_string(raw, "name", path, field_name)
    sensor_type = _required_string(raw, "type", path, field_name)
    pose_from_sequence(_required_list(raw, "pose", path, field_name), f"{field_name}.pose")
    _required_bool(raw, "always_on", path, field_name)
    _required_bool(raw, "visualize", path, field_name)
    _required_number(raw, "update_rate", path, field_name)
    if "topic" in raw:
        _required_string(raw, "topic", path, field_name)
    if sensor_type in ("camera", "depth_camera"):
        camera = _required_mapping(raw, "camera", path, field_name)
        _required_number(camera, "horizontal_fov", path, f"{field_name}.camera")
        image = _required_mapping(camera, "image", path, f"{field_name}.camera")
        for key in ("width", "height"):
            _required_number(image, key, path, f"{field_name}.camera.image")
        _required_string(image, "format", path, f"{field_name}.camera.image")
        clip = _required_mapping(camera, "clip", path, f"{field_name}.camera")
        for key in ("near", "far"):
            _required_number(clip, key, path, f"{field_name}.camera.clip")


def _validate_geometry(raw: Mapping[str, Any], field_name: str, path: Path) -> None:
    geometry_type = _required_string(raw, "type", path, field_name)
    if geometry_type == "box":
        _float_list(_required_list(raw, "size", path, field_name), f"{field_name}.size", 3)
    elif geometry_type == "cylinder":
        _required_number(raw, "radius", path, field_name)
        _required_number(raw, "length", path, field_name)
    elif geometry_type == "plane":
        _float_list(_required_list(raw, "size", path, field_name), f"{field_name}.size", 2)
        vector3_from_sequence(_required_list(raw, "normal", path, field_name), f"{field_name}.normal")
    else:
        raise RuntimeError(
            f"{field_name}.type must be one of box, cylinder, plane; got '{geometry_type}': {path}"
        )


def _validate_material(raw: Mapping[str, Any], field_name: str, path: Path) -> None:
    for key in ("ambient", "diffuse"):
        _float_list(_required_list(raw, key, path, field_name), f"{field_name}.{key}", 4)
    if "specular" in raw:
        _float_list(_required_list(raw, "specular", path, field_name), f"{field_name}.specular", 4)


def _validate_inertial(raw: Mapping[str, Any], field_name: str, path: Path) -> None:
    _required_number(raw, "mass", path, field_name)
    inertia = _required_mapping(raw, "inertia", path, field_name)
    for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        _required_number(inertia, key, path, f"{field_name}.inertia")


def _validate_camera(raw: Mapping[str, Any], field_name: str, path: Path) -> None:
    for key in ("plugin_filename", "plugin_name", "engine", "scene"):
        _required_string(raw, key, path, field_name)
    _validate_gz_gui(_required_mapping(raw, "gz-gui", path, field_name), f"{field_name}.gz-gui", path)
    for key in ("ambient_light", "background_color"):
        _float_list(_required_list(raw, key, path, field_name), f"{field_name}.{key}", 3)
    pose_from_sequence(_required_list(raw, "pose", path, field_name), f"{field_name}.pose")
    clip = _required_mapping(raw, "clip", path, field_name)
    for key in ("near", "far"):
        _required_number(clip, key, path, f"{field_name}.clip")


def _validate_plugin(raw: Any, field_name: str, path: Path) -> None:
    if not isinstance(raw, dict):
        raise RuntimeError(f"{field_name} must be a mapping: {path}")
    for key in ("filename", "name"):
        _required_string(raw, key, path, field_name)
    if "gz-gui" in raw:
        _validate_gz_gui(_required_mapping(raw, "gz-gui", path, field_name), f"{field_name}.gz-gui", path)
    if "children" in raw:
        _validate_children(raw["children"], f"{field_name}.children", path)


def _validate_gz_gui(raw: Mapping[str, Any], field_name: str, path: Path) -> None:
    if "title" in raw:
        _required_string(raw, "title", path, field_name)
    properties = _required_list(raw, "properties", path, field_name)
    for index, property_spec in enumerate(properties):
        property_field = f"{field_name}.properties[{index}]"
        if not isinstance(property_spec, dict):
            raise RuntimeError(f"{property_field} must be a mapping: {path}")
        _required_string(property_spec, "type", path, property_field)
        _required_string(property_spec, "key", path, property_field)
        if "value" not in property_spec:
            raise RuntimeError(f"{property_field}.value is required: {path}")


def _validate_children(raw: Any, field_name: str, path: Path) -> None:
    if isinstance(raw, dict):
        for key, value in raw.items():
            if not isinstance(key, str) or not key:
                raise RuntimeError(f"{field_name} keys must be non-empty strings: {path}")
            _validate_child_value(value, f"{field_name}.{key}", path)
        return
    if isinstance(raw, list):
        for index, item in enumerate(raw):
            _validate_child_node(item, f"{field_name}[{index}]", path)
        return
    raise RuntimeError(f"{field_name} must be a mapping or list: {path}")


def _validate_child_value(raw: Any, field_name: str, path: Path) -> None:
    if isinstance(raw, dict):
        if any(key in raw for key in ("attributes", "text", "children")):
            if "attributes" in raw:
                attributes = _required_mapping(raw, "attributes", path, field_name)
                for attr_key in attributes:
                    if not isinstance(attr_key, str) or not attr_key:
                        raise RuntimeError(f"{field_name}.attributes keys must be non-empty strings: {path}")
            if "children" in raw:
                _validate_children(raw["children"], f"{field_name}.children", path)
        else:
            _validate_children(raw, field_name, path)
    elif isinstance(raw, list):
        for index, item in enumerate(raw):
            if isinstance(item, dict) and "tag" in item:
                _validate_child_node(item, f"{field_name}[{index}]", path)
    else:
        return


def _validate_child_node(raw: Any, field_name: str, path: Path) -> None:
    if not isinstance(raw, dict):
        raise RuntimeError(f"{field_name} must be a mapping: {path}")
    _required_string(raw, "tag", path, field_name)
    if "attributes" in raw:
        attributes = _required_mapping(raw, "attributes", path, field_name)
        for attr_key in attributes:
            if not isinstance(attr_key, str) or not attr_key:
                raise RuntimeError(f"{field_name}.attributes keys must be non-empty strings: {path}")
    if "children" in raw:
        _validate_children(raw["children"], f"{field_name}.children", path)


def _validate_startup_commands(raw: list[Any], field_name: str, path: Path) -> None:
    for index, command in enumerate(raw):
        command_field = f"{field_name}[{index}]"
        if not isinstance(command, dict):
            raise RuntimeError(f"{command_field} must be a mapping: {path}")
        _required_string(command, "command", path, command_field)
        args = _required_list(command, "args", path, command_field)
        for arg_index, arg in enumerate(args):
            if not isinstance(arg, (str, int, float, bool)):
                raise RuntimeError(f"{command_field}.args[{arg_index}] must be scalar: {path}")
        if "delay_sec" in command:
            _required_number(command, "delay_sec", path, command_field)


def _required_mapping(raw: Mapping[str, Any], key: str, path: Path, prefix: str | None = None) -> Mapping[str, Any]:
    value = raw.get(key)
    if not isinstance(value, dict):
        raise RuntimeError(f"{_field(prefix, key)} must be a mapping: {path}")
    return value


def _required_list(raw: Mapping[str, Any], key: str, path: Path, prefix: str | None = None) -> list[Any]:
    value = raw.get(key)
    if not isinstance(value, list):
        raise RuntimeError(f"{_field(prefix, key)} must be a list: {path}")
    return value


def _required_string(raw: Mapping[str, Any], key: str, path: Path, prefix: str | None = None) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value:
        raise RuntimeError(f"{_field(prefix, key)} must be a non-empty string: {path}")
    return value


def _required_bool(raw: Mapping[str, Any], key: str, path: Path, prefix: str | None = None) -> bool:
    value = raw.get(key)
    if not isinstance(value, bool):
        raise RuntimeError(f"{_field(prefix, key)} must be a boolean: {path}")
    return value


def _required_number(raw: Mapping[str, Any], key: str, path: Path, prefix: str | None = None) -> float:
    value = raw.get(key)
    if not isinstance(value, (int, float)):
        raise RuntimeError(f"{_field(prefix, key)} must be numeric: {path}")
    return float(value)


def _float_list(value: list[Any], field_name: str, expected_len: int) -> tuple[float, ...]:
    if len(value) != expected_len:
        raise RuntimeError(f"{field_name} must contain {expected_len} values")
    try:
        return tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"{field_name} must contain numeric values") from exc


def _field(prefix: str | None, key: str) -> str:
    return f"{prefix}.{key}" if prefix else key
