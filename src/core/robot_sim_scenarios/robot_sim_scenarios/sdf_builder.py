from hashlib import sha256
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Mapping
from xml.etree import ElementTree as ET

import yaml

from robot_sim_scenarios.models import Scene


def build_world(scene: Scene, output_dir: str | Path | None = None) -> Path:
    root = ET.Element("sdf", version=str(scene.world["sdf_version"]))
    world = ET.SubElement(root, "world", name=str(scene.world["name"]))

    _append_world_settings(world, scene.world)
    _append_light_elements(world, scene.lights)
    _append_model(world, scene.ground)
    for scene_object in scene.objects:
        if scene_object.object_type == "include":
            if not _skip_optional_remote_include(scene_object.raw):
                _append_include(world, scene_object.raw)
        else:
            _append_model(world, scene_object.raw)

    tree = ET.ElementTree(root)
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")

    path = _world_output_path(scene, output_dir)
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path


def _append_world_settings(world: ET.Element, spec: Mapping[str, Any]) -> None:
    _append_text(world, "gravity", spec["gravity"])
    _append_text(world, "magnetic_field", spec["magnetic_field"])

    atmosphere = ET.SubElement(world, "atmosphere", type=str(spec["atmosphere"]["type"]))
    for key, value in spec["atmosphere"].items():
        if key != "type":
            _append_text(atmosphere, key, value)

    physics_spec = spec["physics"]
    physics = ET.SubElement(
        world,
        "physics",
        name=str(physics_spec["name"]),
        type=str(physics_spec["type"]),
    )
    for key, value in physics_spec.items():
        if key not in ("name", "type"):
            _append_text(physics, key, value)

    for plugin_spec in spec["plugins"]:
        _append_plugin(world, plugin_spec)

    scene_spec = spec["scene"]
    scene = ET.SubElement(world, "scene")
    for key, value in scene_spec.items():
        _append_text(scene, key, value)

    _append_gui(world, spec["gui"])


def _append_gui(world: ET.Element, spec: Mapping[str, Any]) -> None:
    gui = ET.SubElement(world, "gui", fullscreen=str(spec["fullscreen"]))
    _append_camera_plugin(gui, spec["camera"])
    for plugin_spec in spec.get("plugins", []):
        _append_plugin(gui, plugin_spec)


def _append_camera_plugin(gui: ET.Element, spec: Mapping[str, Any]) -> None:
    plugin = ET.SubElement(
        gui,
        "plugin",
        filename=str(spec["plugin_filename"]),
        name=str(spec["plugin_name"]),
    )
    _append_gz_gui(plugin, spec["gz-gui"])
    for key in ("engine", "scene", "ambient_light", "background_color"):
        _append_text(plugin, key, spec[key])
    _append_text(plugin, "camera_pose", spec["pose"])
    clip = ET.SubElement(plugin, "camera_clip")
    _append_text(clip, "near", spec["clip"]["near"])
    _append_text(clip, "far", spec["clip"]["far"])


def _append_plugin(parent: ET.Element, spec: Mapping[str, Any]) -> None:
    plugin = ET.SubElement(
        parent,
        "plugin",
        filename=str(spec["filename"]),
        name=str(spec["name"]),
    )
    for key, value in spec.items():
        if key in ("filename", "name"):
            continue
        if key == "gz-gui":
            _append_gz_gui(plugin, value)
        elif key == "children":
            _append_children(plugin, value)
        else:
            _append_text(plugin, key, value)


def _append_gz_gui(parent: ET.Element, spec: Mapping[str, Any]) -> None:
    gz_gui = ET.SubElement(parent, "gz-gui")
    if "title" in spec:
        _append_text(gz_gui, "title", spec["title"])
    for property_spec in spec.get("properties", []):
        prop = ET.SubElement(
            gz_gui,
            "property",
            type=str(property_spec["type"]),
            key=str(property_spec["key"]),
        )
        prop.text = _text_value(property_spec["value"])


def _append_light_elements(world: ET.Element, lights: tuple[Mapping[str, Any], ...]) -> None:
    for spec in lights:
        light = ET.SubElement(world, "light", type=str(spec["type"]), name=str(spec["name"]))
        _append_text(light, "cast_shadows", spec["cast_shadows"])
        _append_text(light, "pose", spec["pose"])
        _append_text(light, "diffuse", spec["diffuse"])
        _append_text(light, "specular", spec["specular"])
        attenuation = ET.SubElement(light, "attenuation")
        for key, value in spec["attenuation"].items():
            _append_text(attenuation, key, value)
        _append_text(light, "direction", spec["direction"])


def _append_model(world: ET.Element, spec: Mapping[str, Any]) -> None:
    model = ET.SubElement(world, "model", name=str(spec["name"]))
    _append_text(model, "pose", spec["pose"])
    _append_text(model, "static", spec["static"])

    if "links" in spec:
        for link_spec in spec["links"]:
            _append_link(model, link_spec)
    else:
        link = ET.SubElement(model, "link", name=str(spec.get("link_name", "link")))
        if "inertial" in spec:
            _append_inertial(link, spec["inertial"])

        if spec["collision"] and not spec["visual_only"]:
            collision = ET.SubElement(link, "collision", name="collision")
            _append_geometry(collision, spec["geometry"])

        visual = ET.SubElement(link, "visual", name="visual")
        _append_geometry(visual, spec["geometry"])
        _append_material(visual, spec["material"])

    for plugin_spec in spec.get("plugins", []):
        _append_plugin(model, plugin_spec)


def _append_include(world: ET.Element, spec: Mapping[str, Any]) -> None:
    include = ET.SubElement(world, "include")
    _append_text(include, "uri", spec["uri"])
    _append_text(include, "name", spec["name"])
    _append_text(include, "pose", spec["pose"])
    if "static" in spec:
        _append_text(include, "static", spec["static"])


def _skip_optional_remote_include(spec: Mapping[str, Any]) -> bool:
    if not spec.get("optional", False):
        return False
    uri = str(spec["uri"])
    if not uri.startswith(("http://", "https://")):
        return False
    enabled = os.environ.get("ROBOT_SIM_ENABLE_FUEL_INCLUDES", "").lower()
    return enabled not in ("1", "true", "yes", "on")


def _append_link(model: ET.Element, spec: Mapping[str, Any]) -> None:
    link = ET.SubElement(model, "link", name=str(spec["name"]))
    _append_text(link, "pose", spec["pose"])
    if "inertial" in spec:
        _append_inertial(link, spec["inertial"])

    if "geometry" in spec:
        if spec["collision"] and not spec["visual_only"]:
            collision = ET.SubElement(link, "collision", name="collision")
            _append_geometry(collision, spec["geometry"])
        visual = ET.SubElement(link, "visual", name="visual")
        _append_geometry(visual, spec["geometry"])
        _append_material(visual, spec["material"])

    for collision_spec in spec.get("collisions", []):
        collision = ET.SubElement(link, "collision", name=str(collision_spec["name"]))
        _append_text(collision, "pose", collision_spec["pose"])
        _append_geometry(collision, collision_spec["geometry"])
        if "surface" in collision_spec:
            surface = ET.SubElement(collision, "surface")
            _append_children(surface, collision_spec["surface"])

    for visual_spec in spec.get("visuals", []):
        visual = ET.SubElement(link, "visual", name=str(visual_spec["name"]))
        _append_text(visual, "pose", visual_spec["pose"])
        _append_geometry(visual, visual_spec["geometry"])
        _append_material(visual, visual_spec["material"])

    for sensor_spec in spec.get("sensors", []):
        _append_sensor(link, sensor_spec)

    for key in ("gravity", "kinematic"):
        if key in spec:
            _append_text(link, key, spec[key])


def _append_sensor(link: ET.Element, spec: Mapping[str, Any]) -> None:
    sensor = ET.SubElement(link, "sensor", name=str(spec["name"]), type=str(spec["type"]))
    _append_text(sensor, "pose", spec["pose"])
    _append_text(sensor, "always_on", spec["always_on"])
    _append_text(sensor, "update_rate", spec["update_rate"])
    _append_text(sensor, "visualize", spec["visualize"])
    if "topic" in spec:
        _append_text(sensor, "topic", spec["topic"])

    if "camera" in spec:
        camera = ET.SubElement(sensor, "camera")
        camera_spec = spec["camera"]
        _append_text(camera, "horizontal_fov", camera_spec["horizontal_fov"])
        image = ET.SubElement(camera, "image")
        for key, value in camera_spec["image"].items():
            _append_text(image, key, value)
        clip = ET.SubElement(camera, "clip")
        _append_text(clip, "near", camera_spec["clip"]["near"])
        _append_text(clip, "far", camera_spec["clip"]["far"])


def _append_inertial(link: ET.Element, spec: Mapping[str, Any]) -> None:
    inertial = ET.SubElement(link, "inertial")
    _append_text(inertial, "mass", spec["mass"])
    inertia = ET.SubElement(inertial, "inertia")
    for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        _append_text(inertia, key, spec["inertia"][key])


def _append_geometry(parent: ET.Element, spec: Mapping[str, Any]) -> None:
    geometry = ET.SubElement(parent, "geometry")
    geometry_type = spec["type"]
    if geometry_type == "box":
        box = ET.SubElement(geometry, "box")
        _append_text(box, "size", spec["size"])
    elif geometry_type == "cylinder":
        cylinder = ET.SubElement(geometry, "cylinder")
        _append_text(cylinder, "radius", spec["radius"])
        _append_text(cylinder, "length", spec["length"])
    elif geometry_type == "plane":
        plane = ET.SubElement(geometry, "plane")
        _append_text(plane, "normal", spec["normal"])
        _append_text(plane, "size", spec["size"])
    else:
        raise RuntimeError(f"Unsupported geometry type: {geometry_type}")


def _append_material(parent: ET.Element, spec: Mapping[str, Any]) -> None:
    material = ET.SubElement(parent, "material")
    for key, value in spec.items():
        _append_text(material, key, value)


def _append_children(parent: ET.Element, spec: Any) -> None:
    if isinstance(spec, Mapping):
        for tag, value in spec.items():
            _append_child(parent, tag, value)
        return
    if isinstance(spec, list):
        for item in spec:
            _append_child_node(parent, item)
        return
    raise RuntimeError("Structured SDF children must be a mapping or list")


def _append_child(parent: ET.Element, tag: str, value: Any) -> ET.Element:
    if isinstance(value, Mapping) and any(key in value for key in ("attributes", "text", "children")):
        return _append_structured_element(parent, tag, value)
    if isinstance(value, Mapping):
        element = ET.SubElement(parent, tag)
        _append_children(element, value)
        return element
    return _append_text(parent, tag, value)


def _append_child_node(parent: ET.Element, spec: Mapping[str, Any]) -> ET.Element:
    if not isinstance(spec, Mapping):
        raise RuntimeError("Structured SDF child nodes must be mappings")
    return _append_structured_element(parent, str(spec["tag"]), spec)


def _append_structured_element(parent: ET.Element, tag: str, spec: Mapping[str, Any]) -> ET.Element:
    attributes = {
        str(key): _text_value(value)
        for key, value in spec.get("attributes", {}).items()
    }
    element = ET.SubElement(parent, tag, attributes)
    if "text" in spec:
        element.text = _text_value(spec["text"])
    if "children" in spec:
        _append_children(element, spec["children"])
    return element


def _append_text(parent: ET.Element, tag: str, value: Any) -> ET.Element:
    element = ET.SubElement(parent, tag)
    element.text = _text_value(value)
    return element


def _text_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        return " ".join(_text_value(item) for item in value)
    return str(value)


def _world_output_path(scene: Scene, output_dir: str | Path | None) -> Path:
    digest = sha256(yaml.safe_dump(scene.raw, sort_keys=True).encode("utf-8")).hexdigest()
    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", scene.name)
    base_dir = Path(output_dir) if output_dir else Path(tempfile.gettempdir()) / "robot_sim_scenes"
    return base_dir / f"{safe_name}-{digest[:12]}.world.sdf"
