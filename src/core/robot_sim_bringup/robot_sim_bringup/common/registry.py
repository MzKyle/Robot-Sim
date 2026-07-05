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
    return _builtin_config_path(
        str(profile_name or "panda"),
        [(PROFILE_DIR,)],
        "sim_profile",
        ("config/sim_profiles",),
    )


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
    return _builtin_config_path(
        str(case_name),
        [(VALIDATION_CASE_DIR,)],
        "validation_case",
        ("config/validation_cases",),
    )


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
        for relative in (
            f"{package_name}/package.xml",
            f"*/{package_name}/package.xml",
            f"*/*/{package_name}/package.xml",
            f"*/*/*/{package_name}/package.xml",
        ):
            for match in sorted(root.glob(relative)):
                return match.parent.resolve()
    return None


def _candidate_roots(start: Path) -> Iterable[Path]:
    yielded = set()
    for ancestor in start.parents:
        if (ancestor / "package.xml").is_file() and ancestor.parent not in yielded:
            yielded.add(ancestor.parent)
            yield ancestor.parent
        src_root = ancestor / "src"
        if src_root.is_dir() and src_root not in yielded:
            yielded.add(src_root)
            yield src_root
        if (ancestor / ".git").exists():
            break


def _existing_path(path_text: str, label: str) -> Path:
    path = Path(path_text).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    if not path.exists():
        raise RuntimeError(f"{label} file does not exist: {path}")
    return path.resolve()


def _package_config_path(
    package_name: str,
    relative_dir: str | Iterable[str],
    name: str,
    label: str,
) -> Path:
    share = package_share_directory(package_name)
    relative_dirs = (relative_dir,) if isinstance(relative_dir, str) else tuple(relative_dir)
    candidates = _named_candidates(share, relative_dirs, name)
    matches = [candidate.resolve() for candidate in candidates if candidate.exists()]
    if len(matches) > 1:
        raise RuntimeError(f"duplicate {label} '{name}' in package '{package_name}': {', '.join(str(path) for path in matches)}")
    if matches:
        return matches[0]
    searched = ", ".join(str(candidate) for candidate in candidates)
    raise RuntimeError(f"unknown {label} '{name}' in package '{package_name}'; searched {searched}")


def _builtin_config_path(
    name: str,
    relative_dir_groups: Iterable[Iterable[str]],
    label: str,
    legacy_dirs: Iterable[str] = (),
) -> Path:
    priority_groups: list[list[Path]] = []
    for root in _builtin_roots():
        for relative_dirs in relative_dir_groups:
            priority_groups.append(_named_candidates(root, tuple(relative_dirs), name))
    legacy_root = package_share_directory("robot_sim_bringup")
    for legacy_dir in legacy_dirs:
        priority_groups.append(_named_candidates(legacy_root, (legacy_dir,), name))

    searched: list[str] = []
    for candidates in priority_groups:
        matches = [candidate.resolve() for candidate in candidates if candidate.exists()]
        searched.extend(str(candidate) for candidate in candidates)
        if len(matches) > 1:
            raise RuntimeError(f"duplicate {label} '{name}' at the same priority: {', '.join(str(path) for path in matches)}")
        if matches:
            return matches[0]
    raise RuntimeError(f"unknown {label} '{name}'; searched {', '.join(searched)}")


def _named_candidates(root: Path, relative_dirs: Iterable[str], name: str) -> list[Path]:
    candidates: list[Path] = []
    for relative_dir in relative_dirs:
        candidates.extend([
            root / relative_dir / f"{name}.yaml",
            root / relative_dir / f"{name}.yml",
        ])
    return candidates


def _builtin_roots() -> list[Path]:
    roots: list[Path] = []
    repo_root = _repo_root(Path(__file__).resolve())
    if repo_root is not None:
        roots.extend([
            repo_root / "examples" / "robot_arm",
            repo_root / "integrations" / "welding",
            repo_root / "integrations" / "auto_cover",
        ])

    share = package_share_directory("robot_sim_bringup")
    roots.extend([
        share / "examples" / "robot_arm",
        share / "integrations" / "welding",
        share / "integrations" / "auto_cover",
    ])

    result: list[Path] = []
    seen: set[Path] = set()
    for root in roots:
        resolved = root.resolve()
        if resolved not in seen:
            seen.add(resolved)
            result.append(resolved)
    return result


def _repo_root(start: Path) -> Path | None:
    for ancestor in start.parents:
        if (ancestor / ".git").exists():
            return ancestor
    return None
