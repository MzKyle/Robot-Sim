from dataclasses import dataclass
from pathlib import Path
import random
from typing import Any, Mapping, Sequence


Pose = tuple[float, float, float, float, float, float]
Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class Workspace:
    frame: str
    min_bounds: Vector3
    max_bounds: Vector3
    raw: Mapping[str, Any]


@dataclass(frozen=True)
class Region:
    name: str
    frame: str
    min_bounds: Vector3
    max_bounds: Vector3
    orientation_rpy: Vector3
    raw: Mapping[str, Any]

    def sample(self, rng=None) -> Pose:
        generator = rng if rng is not None else random
        xyz = tuple(
            generator.uniform(low, high)
            for low, high in zip(self.min_bounds, self.max_bounds)
        )
        return (*xyz, *self.orientation_rpy)


@dataclass(frozen=True)
class SceneObject:
    name: str
    object_type: str
    pose: Pose
    geometry: Mapping[str, Any]
    tags: tuple[str, ...]
    raw: Mapping[str, Any]


@dataclass(frozen=True)
class Scene:
    name: str
    path: Path
    description: str
    raw: Mapping[str, Any]
    world: Mapping[str, Any]
    ground: Mapping[str, Any]
    lights: tuple[Mapping[str, Any], ...]
    robot_mount_pose: Pose
    workspace: Workspace
    objects: tuple[SceneObject, ...]
    regions: Mapping[str, Region]

    def sample_region(self, region_name: str, rng=None) -> Pose:
        try:
            region = self.regions[region_name]
        except KeyError as exc:
            valid = ", ".join(sorted(self.regions))
            raise KeyError(f"Unknown scene region '{region_name}'. Valid regions: {valid}") from exc
        return region.sample(rng)


def pose_from_sequence(value: Sequence[Any], field_name: str) -> Pose:
    values = _float_sequence(value, field_name, 6)
    return (values[0], values[1], values[2], values[3], values[4], values[5])


def vector3_from_sequence(value: Sequence[Any], field_name: str) -> Vector3:
    values = _float_sequence(value, field_name, 3)
    return (values[0], values[1], values[2])


def _float_sequence(value: Sequence[Any], field_name: str, expected_len: int) -> tuple[float, ...]:
    if isinstance(value, str) or not isinstance(value, Sequence):
        raise RuntimeError(f"{field_name} must be a sequence with {expected_len} values")
    if len(value) != expected_len:
        raise RuntimeError(f"{field_name} must contain {expected_len} values")
    try:
        return tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"{field_name} must contain numeric values") from exc
