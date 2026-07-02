from __future__ import annotations

from pathlib import Path
from typing import Iterable


PROFILE_DIR = "robot_sim/profiles"
VALIDATION_CASE_DIR = "robot_sim/validation_cases"
SCENE_DIR = "robot_sim/scenes"


def package_share_directory(package_name: str) -> Path:
    source_path = source_package_directory(package_name)
    if source_path is not None:
        return source_path

    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory(package_name))


def resolve_profile_path(
    profile_name: str,
    profile_file: str = "",
    profile_package: str = "",
) -> Path:
    if profile_file:
        return _existing_path(profile_file, "sim_profile")
    if profile_package:
        return _package_config_path(profile_package, PROFILE_DIR, profile_name, "sim_profile")
    return (
        package_share_directory("robot_sim_bringup")
        / "config"
        / "sim_profiles"
        / f"{profile_name or 'panda'}.yaml"
    ).resolve()


def resolve_validation_case_path(
    case_name: str | Path,
    case_package: str = "",
) -> Path:
    candidate = Path(case_name).expanduser()
    if candidate.exists():
        return candidate.resolve()
    if candidate.suffix in (".yaml", ".yml") or candidate.parent != Path("."):
        raise RuntimeError(f"validation case file does not exist: {candidate}")
    if case_package:
        return _package_config_path(case_package, VALIDATION_CASE_DIR, str(case_name), "validation_case")
    path = (
        package_share_directory("robot_sim_bringup")
        / "config"
        / "validation_cases"
        / f"{case_name}.yaml"
    )
    if not path.exists():
        raise RuntimeError(f"unknown validation case '{case_name}': {path}")
    return path.resolve()


def resolve_scene_path(scene_name: str | Path, scene_package: str = "") -> Path:
    candidate = Path(scene_name).expanduser()
    if candidate.exists():
        return candidate.resolve()
    if candidate.suffix in (".yaml", ".yml") or candidate.parent != Path("."):
        raise RuntimeError(f"scene file does not exist: {candidate}")
    if scene_package:
        return _package_config_path(scene_package, SCENE_DIR, str(scene_name), "scene")
    path = package_share_directory("robot_sim_scenarios") / "scenes" / f"{scene_name}.yaml"
    if not path.exists():
        raise RuntimeError(f"unknown scene '{scene_name}': {path}")
    return path.resolve()


def source_package_directory(package_name: str) -> Path | None:
    for root in _candidate_roots(Path(__file__).resolve()):
        matches = sorted(root.glob(f"**/{package_name}/package.xml"))
        if matches:
            return matches[0].parent.resolve()
    return None


def _candidate_roots(start: Path) -> Iterable[Path]:
    yielded = set()
    for ancestor in start.parents:
        for root in (ancestor, ancestor / "src"):
            if root.exists() and root not in yielded:
                yielded.add(root)
                yield root


def _existing_path(path_text: str, label: str) -> Path:
    path = Path(path_text).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    if not path.exists():
        raise RuntimeError(f"{label} file does not exist: {path}")
    return path.resolve()


def _package_config_path(
    package_name: str,
    relative_dir: str,
    name: str,
    label: str,
) -> Path:
    share = package_share_directory(package_name)
    candidates = [
        share / relative_dir / f"{name}.yaml",
        share / relative_dir / f"{name}.yml",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    searched = ", ".join(str(candidate) for candidate in candidates)
    raise RuntimeError(f"unknown {label} '{name}' in package '{package_name}'; searched {searched}")
