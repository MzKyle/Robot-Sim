import os
import re
import shutil
import subprocess

from ament_index_python.packages import get_package_prefix


GZ_ROS2_CONTROL_HARDWARE_PLUGIN = "gz_ros2_control/GazeboSimSystem"
GZ_ROS2_CONTROL_LIBRARY = "libgz_ros2_control-system.so"
GZ_ROS2_CONTROL_SYSTEM_PLUGIN = "gz_ros2_control::GazeboSimROS2ControlPlugin"


def uses_gz_ros2_control(profile, mode):
    return (
        bool(mode["use_gazebo"])
        and profile["control"]["hardware_plugins"]["gazebo"]
        == GZ_ROS2_CONTROL_HARDWARE_PLUGIN
    )


def gz_ros2_control_paths():
    prefix = get_package_prefix("gz_ros2_control")
    lib_dir = os.path.join(prefix, "lib")
    return {
        "prefix": prefix,
        "lib_dir": lib_dir,
        "library": os.path.join(lib_dir, GZ_ROS2_CONTROL_LIBRARY),
    }


def gz_ros2_control_environment(base_env=None):
    env = dict(base_env if base_env is not None else os.environ)
    paths = gz_ros2_control_paths()
    lib_dir = paths["lib_dir"]
    return {
        "GZ_SIM_SYSTEM_PLUGIN_PATH": _prepend_path(
            lib_dir,
            env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
        ),
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": _prepend_path(
            lib_dir,
            env.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ""),
        ),
        "LD_LIBRARY_PATH": _prepend_path(lib_dir, env.get("LD_LIBRARY_PATH", "")),
    }


def check_gz_ros2_control_plugin(expected_gz_major=""):
    result = {
        "ok": True,
        "errors": [],
        "warnings": [],
        "prefix": "",
        "lib_dir": "",
        "library": "",
        "gz_version": "",
    }

    try:
        paths = gz_ros2_control_paths()
    except Exception as exc:
        _error(
            result,
            "gz_ros2_control package is not visible. "
            "Build this workspace and source install/setup.bash: "
            f"{exc}",
        )
        return result

    result.update(paths)
    library = paths["library"]
    if not os.path.exists(library):
        _error(
            result,
            f"gz_ros2_control system plugin library not found: {library}. "
            "Run: colcon build --symlink-install --allow-overriding "
            "gz_ros2_control --packages-select gz_ros2_control",
        )
        return result

    _check_gz_version(result, expected_gz_major)
    _check_plugin_exports(result, library)
    return result


def format_gz_ros2_control_check(result):
    lines = [
        f"gz_ros2_control prefix: {result.get('prefix', '')}",
        f"gz_ros2_control plugin: {result.get('library', '')}",
    ]
    if result.get("gz_version"):
        lines.append(f"gz sim version: {result['gz_version']}")
    for warning in result.get("warnings", []):
        lines.append(f"WARNING: {warning}")
    for error in result.get("errors", []):
        lines.append(f"ERROR: {error}")
    return "\n".join(line for line in lines if line)


def _check_gz_version(result, expected_gz_major):
    expected = str(expected_gz_major or "").strip()
    if not expected:
        return
    if not shutil.which("gz"):
        _error(result, "gz command is not available on PATH")
        return

    completed = subprocess.run(
        ["gz", "sim", "--versions"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    output = (completed.stdout or completed.stderr).strip()
    result["gz_version"] = output
    if completed.returncode != 0:
        _error(result, "failed to query Gazebo version: " + output)
        return

    match = re.search(r"\d+", output)
    actual = match.group(0) if match else ""
    if actual and actual != expected:
        _error(
            result,
            f"profile expects Gazebo Sim major version {expected}, "
            f"but 'gz sim --versions' reports {output}",
        )


def _check_plugin_exports(result, library):
    if not shutil.which("gz"):
        _error(result, "gz command is not available on PATH")
        return

    env = dict(os.environ)
    env.update(gz_ros2_control_environment(env))
    completed = subprocess.run(
        ["gz", "plugin", "-p", library, "--info"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
        check=False,
    )
    output = "\n".join(
        text.strip()
        for text in (completed.stdout, completed.stderr)
        if text and text.strip()
    )
    if completed.returncode != 0 or GZ_ROS2_CONTROL_SYSTEM_PLUGIN not in output:
        _error(
            result,
            "gz_ros2_control library does not export "
            f"{GZ_ROS2_CONTROL_SYSTEM_PLUGIN}. "
            "The workspace overlay is probably missing or the system package is "
            "built for a different Gazebo ABI.\n"
            + output,
        )


def _prepend_path(path, existing):
    if not existing:
        return path
    parts = [part for part in existing.split(os.pathsep) if part]
    if path in parts:
        return existing
    return os.pathsep.join([path, existing])


def _error(result, message):
    result["ok"] = False
    result["errors"].append(message)
