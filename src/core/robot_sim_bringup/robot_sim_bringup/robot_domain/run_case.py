import argparse
from datetime import datetime, timezone
import html
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

import yaml

from robot_sim_bringup.common.registry import resolve_profile_path, resolve_validation_case_path
from robot_sim_bringup.robot_domain.task_runners import get_task_runner
from robot_sim_bringup.robot_domain.validation_cases import load_validation_case


SUCCESS = 0
FAILURE = 1


class CommandRunner:
    def run(self, command, log_path, timeout=None, env=None):
        started = time.monotonic()
        with open(log_path, "w", encoding="utf-8") as handle:
            result = subprocess.run(
                command,
                stdout=handle,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout,
                env=env,
                check=False,
            )
        return {
            "command": [str(item) for item in command],
            "returncode": result.returncode,
            "duration_sec": time.monotonic() - started,
            "log": str(log_path),
        }

    def popen(self, command, log_path, env=None):
        handle = open(log_path, "w", encoding="utf-8")
        process = subprocess.Popen(
            command,
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
            env=env,
            preexec_fn=os.setsid,
        )
        process._robot_sim_log_handle = handle
        return process


def build_parser():
    parser = argparse.ArgumentParser(description="Run one robot_sim validation case and write artifacts.")
    parser.add_argument("--case", required=True, help="Validation case name or YAML path.")
    parser.add_argument("--case-package", default="", help="ROS package containing robot_sim/validation_cases/<case>.yaml.")
    parser.add_argument("--output-dir", default="robot_sim_runs", help="Parent directory for run artifacts.")
    parser.add_argument("--profile", default="", help="Override case launch.profile.")
    parser.add_argument("--profile-file", default="", help="Override or provide an external sim_profile YAML.")
    parser.add_argument("--profile-package", default="", help="ROS package containing robot_sim/profiles/<profile>.yaml.")
    parser.add_argument("--scene", default="", help="Override case scene by name or YAML path.")
    parser.add_argument("--scene-package", default="", help="ROS package containing robot_sim/scenes/<scene>.yaml.")
    parser.add_argument("--scene-variant", default="", help="Scene variant to apply.")
    parser.add_argument("--scene-param", action="append", default=[], help="Scene parameter override as name=value. Repeatable.")
    parser.add_argument("--mode", default=None, choices=("full", "light", "mock"), help="Override case launch.mode.")
    parser.add_argument("--sensor-overrides", default=None, help="Override case launch.sensor_overrides.")
    parser.add_argument("--timeout", type=float, default=None, help="Override case launch.timeout_sec.")
    parser.add_argument("--rosbag-duration", type=float, default=8.0, help="Seconds to record rosbag when enabled.")
    parser.add_argument("--no-rosbag", action="store_true", help="Disable rosbag recording for this run.")
    parser.add_argument("--keep-sim", action="store_true", help="Leave the launched simulation process running.")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    runner = CommandRunner()
    try:
        return run_case(args, runner)
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        return FAILURE


def run_case(args, runner):
    case_path = resolve_validation_case_path(args.case, case_package=args.case_package)
    if _is_schema4_case(case_path):
        print(
            "ERROR: schema 4 generic validation has moved to the robot_validation project. "
            "Run this case with: ros2 run robot_validation run_case --case <case>",
            file=sys.stderr,
        )
        return FAILURE

    scene_parameters = _parse_scene_params(args.scene_param)
    case = load_validation_case(
        args.case,
        case_package=args.case_package,
        scene_override=args.scene,
        scene_package=args.scene_package,
        scene_variant=args.scene_variant,
        scene_parameters=scene_parameters,
    )
    effective = _effective_launch(case, args)
    run_dir = _create_run_dir(args.output_dir, case["name"], effective["profile"])
    logs_dir = run_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    rosbag_dir = run_dir / "rosbag"
    effective_case_file = _write_effective_case(case, effective, run_dir)
    effective["case_file"] = str(effective_case_file)
    effective["profile_file"] = str(_write_effective_profile(case, effective, run_dir))

    manifest = _initial_manifest(case, effective, args, run_dir)
    metrics = {
        "case_name": case["name"],
        "profile": effective["profile"],
        "mode": effective["mode"],
        "scene": case["scene"].name,
        "passed": False,
        "steps": [],
        "artifacts": {
            "run_dir": str(run_dir),
            "logs_dir": str(logs_dir),
            "rosbag_dir": str(rosbag_dir),
        },
        "adapter_health": [],
        "module_services": [],
        "module_topics": [],
        "module_events": [],
        "module_failures": [],
        "adapter_data_sources": [],
    }
    sim_process = None
    bag_process = None
    exit_code = FAILURE

    try:
        _write_json(run_dir / "manifest.json", manifest)
        try:
            _preflight(effective["mode"])
            _record_manual_step(metrics, "preflight", True, "required commands available", None)
        except Exception as exc:
            _record_manual_step(metrics, "preflight", False, str(exc), None)
            raise

        common_args = _common_helper_args(effective)
        lint_args = _lint_args(effective)
        helper = [sys.executable, "-m", "robot_sim_bringup.robot_domain.sim_smoke_helper"]
        linter = [sys.executable, "-m", "robot_sim_bringup.robot_domain.profile_lint"]

        _run_step(metrics, runner, "profile_lint", linter + lint_args, logs_dir / "profile_lint.log", timeout=60.0)
        profile_json_log = logs_dir / "profile.json"
        _run_step(metrics, runner, "profile_summary", helper + ["profile-json", *common_args, "--with-moveit"], profile_json_log, timeout=60.0)
        profile_summary = _load_json_file(profile_json_log)

        urdf_path = run_dir / "robot.urdf"
        _run_step(metrics, runner, "render_urdf", helper + ["render-urdf", *common_args, "--output", str(urdf_path)], logs_dir / "render_urdf.log", timeout=60.0)
        _run_step(metrics, runner, "validate_urdf", ["check_urdf", str(urdf_path)], logs_dir / "check_urdf.log", timeout=30.0)

        sim_process = _start_simulation(runner, effective, logs_dir / "sim.launch.log")
        _record_manual_step(metrics, "simulation_start", True, "simulation process started", logs_dir / "sim.launch.log")

        if profile_summary.get("use_gazebo", False):
            _wait_gazebo_model(
                metrics,
                profile_summary["spawn_name"],
                effective["timeout"],
                sim_process,
                logs_dir / "gazebo_spawn.log",
            )
        else:
            _record_manual_step(metrics, "gazebo_spawn", True, "mock mode: skipped", None)

        step_timeout = effective["timeout"] + 15.0
        _run_step(metrics, runner, "joint_states", helper + ["wait-joint-state", *common_args, "--timeout", str(effective["timeout"])], logs_dir / "joint_states.log", timeout=step_timeout)
        _run_step(metrics, runner, "controllers_active", helper + ["wait-controllers", *common_args, "--timeout", str(effective["timeout"])], logs_dir / "controllers_active.log", timeout=step_timeout)
        _run_step(metrics, runner, "trajectory_action", helper + ["send-trajectory", *common_args, "--timeout", str(effective["timeout"])], logs_dir / "trajectory_action.log", timeout=step_timeout)
        _run_step(metrics, runner, "sensor_hz", helper + ["check-sensors", *common_args], logs_dir / "sensor_hz.log", timeout=step_timeout)
        _run_step(metrics, runner, "tf_tree", helper + ["check-tf", *common_args, "--urdf", str(urdf_path)], logs_dir / "tf_tree.log", timeout=step_timeout)
        _run_step(metrics, runner, "moveit_plan_execute", helper + ["moveit", *common_args, "--timeout", str(effective["timeout"])], logs_dir / "moveit.log", timeout=step_timeout)

        validation_metrics_path = run_dir / "validation_metrics.json"
        task_runner = get_task_runner(case["task_type"])
        metrics["task_type"] = case["task_type"]
        metrics["business_actions"] = task_runner.business_actions(case)
        validation_timeout = effective["timeout"] * max(len(case.get("task_regions", [])), 1) + 45.0
        try:
            _run_step(
                metrics,
                runner,
                f"validation_case:{case['task_type']}",
                task_runner.validation_command(
                    common_args,
                    effective["case_file"],
                    validation_metrics_path,
                    effective["timeout"],
                ),
                logs_dir / "validation_case.log",
                timeout=validation_timeout,
            )
        except Exception:
            if validation_metrics_path.exists():
                validation_metrics = _load_json_file(validation_metrics_path)
                metrics.update(_validation_metrics_summary(validation_metrics))
                metrics["validation"] = validation_metrics
            raise
        else:
            validation_metrics = _load_json_file(validation_metrics_path)
            metrics.update(_validation_metrics_summary(validation_metrics))
            metrics["validation"] = validation_metrics

        if case["artifacts"]["rosbag"]["enabled"] and not args.no_rosbag:
            bag_process = _record_rosbag(
                runner,
                case,
                effective,
                rosbag_dir,
                logs_dir / "rosbag.launch.log",
                args.rosbag_duration,
            )
            bag_path = rosbag_dir / case["name"]
            _record_manual_step(metrics, "rosbag_record", (bag_path / "metadata.yaml").exists(), f"rosbag: {bag_path}", logs_dir / "rosbag.launch.log")
            if not (bag_path / "metadata.yaml").exists():
                raise RuntimeError(f"rosbag metadata was not created: {bag_path / 'metadata.yaml'}")
        else:
            _record_manual_step(metrics, "rosbag_record", True, "rosbag disabled", None)

        metrics["passed"] = True
        exit_code = SUCCESS
        return SUCCESS
    except Exception as exc:
        metrics["passed"] = False
        metrics["error"] = str(exc)
        print(f"ERROR: {exc}", file=sys.stderr)
        return FAILURE
    finally:
        if bag_process is not None:
            _terminate_process(bag_process, signal.SIGINT)
        if sim_process is not None and not args.keep_sim:
            _terminate_process(sim_process, signal.SIGINT)

        manifest["finished_at"] = _utc_now()
        manifest["passed"] = metrics.get("passed", False)
        manifest["exit_code"] = exit_code
        manifest["artifacts"] = _artifact_manifest(run_dir)
        _write_json(run_dir / "metrics.json", metrics)
        _write_json(run_dir / "manifest.json", manifest)
        _write_reports(run_dir, manifest, metrics)
        print(f"Artifacts: {run_dir}")


def _effective_launch(case, args):
    profile = args.profile or case["profile"]
    profile_package = args.profile_package or case.get("profile_package", "")
    profile_file = args.profile_file or case.get("profile_file", "")
    if profile_package and not profile_file:
        profile_file = str(resolve_profile_path(profile, "", profile_package))
    return {
        "profile": profile,
        "profile_file": profile_file,
        "profile_package": profile_package,
        "mode": args.mode or case["mode"],
        "layout": case.get("layout", "single"),
        "timeout": float(args.timeout if args.timeout is not None else case.get("timeout_sec", 120.0)),
        "sensor_overrides": (
            args.sensor_overrides
            if args.sensor_overrides is not None
            else case.get("sensor_overrides", "")
        ),
        "use_gripper": case.get("task_type") == "pick_place" or bool(case.get("task", {}).get("gripper")),
    }


def _parse_scene_params(items):
    result = {}
    for item in items or []:
        if "=" not in item:
            raise RuntimeError("--scene-param must use name=value")
        name, value = item.split("=", 1)
        name = name.strip()
        if not name:
            raise RuntimeError("--scene-param contains an empty name")
        result[name] = _typed_value(value.strip())
    return result


def _typed_value(value):
    lowered = value.lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    try:
        if re.fullmatch(r"[-+]?[0-9]+", value):
            return int(value)
        if re.fullmatch(r"[-+]?[0-9]*\.[0-9]+", value):
            return float(value)
    except ValueError:
        pass
    return value


def _write_effective_case(case, effective, run_dir):
    raw = dict(case.get("raw", {}))
    raw["scene"] = _scene_spec_for_yaml(case["scene_spec"])
    raw.setdefault("launch", {})
    raw["launch"]["profile"] = effective["profile"]
    raw["launch"]["profile_file"] = effective.get("profile_file", "")
    raw["launch"]["profile_package"] = effective.get("profile_package", "")
    raw["launch"]["mode"] = effective["mode"]
    raw["launch"]["layout"] = effective.get("layout", "single")
    raw["launch"]["timeout_sec"] = effective["timeout"]
    raw["launch"]["sensor_overrides"] = effective.get("sensor_overrides", "")
    path = run_dir / "effective_case.yaml"
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(raw, handle, sort_keys=False)
    return path


def _write_effective_profile(case, effective, run_dir):
    profile_path = resolve_profile_path(
        effective["profile"],
        effective.get("profile_file", ""),
        effective.get("profile_package", ""),
    )
    with profile_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, dict):
        raise RuntimeError(f"sim_profile YAML must be a mapping: {profile_path}")
    layout_name = effective.get("layout", "single")
    layouts = raw.get("layouts", {})
    if layout_name not in layouts:
        valid = ", ".join(sorted(layouts))
        raise RuntimeError(f"profile '{effective['profile']}' missing layout '{layout_name}'. Valid layouts: {valid}")
    world_name = layouts[layout_name].get("world", layout_name)
    raw.setdefault("worlds", {})
    raw["worlds"][world_name] = {
        "scene": _scene_spec_for_yaml(case["scene_spec"]),
    }
    path = run_dir / "effective_profile.yaml"
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(raw, handle, sort_keys=False)
    return path


def _scene_spec_for_yaml(scene_spec):
    if scene_spec.get("name") and _looks_like_path(scene_spec["name"]):
        return str(scene_spec["name"])
    if scene_spec.get("name") and not scene_spec.get("path"):
        package_name = scene_spec.get("package") or "robot_sim_scenarios"
        path = (
            f"scenes/{scene_spec['name']}.yaml"
            if package_name == "robot_sim_scenarios"
            else f"robot_sim/scenes/{scene_spec['name']}.yaml"
        )
        scene_spec = {**scene_spec, "package": package_name, "path": path}
    result = {}
    for key in ("package", "path", "variant", "parameters"):
        value = scene_spec.get(key)
        if value not in (None, "", {}):
            result[key] = value
    return result


def _looks_like_path(value):
    path = Path(str(value)).expanduser()
    return path.exists() or path.suffix in (".yaml", ".yml") or path.parent != Path(".")


def _common_helper_args(effective):
    args = ["--profile", effective["profile"], "--mode", effective["mode"]]
    if effective["profile_file"]:
        args.extend(["--profile-file", effective["profile_file"]])
    if effective["sensor_overrides"]:
        args.extend(["--sensor-overrides", effective["sensor_overrides"]])
    return args


def _lint_args(effective):
    args = _common_helper_args(effective)
    args.append("--require-moveit")
    return args


def _preflight(mode):
    missing = [
        command
        for command in ("ros2", "xacro", "check_urdf")
        if shutil.which(command) is None
    ]
    if mode != "mock" and shutil.which("gz") is None:
        missing.append("gz")
    if missing:
        raise RuntimeError("Missing required commands: " + ", ".join(missing))


def _create_run_dir(output_dir, case_name, profile):
    parent = Path(output_dir).expanduser().resolve()
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    base = parent / f"{timestamp}_{_safe_id(case_name)}_{_safe_id(profile)}"
    candidate = base
    suffix = 1
    while candidate.exists():
        suffix += 1
        candidate = Path(f"{base}_{suffix}")
    candidate.mkdir(parents=True)
    return candidate


def _initial_manifest(case, effective, args, run_dir):
    module = dict(case.get("module", {}))
    adapters = [dict(adapter) for adapter in case.get("adapters", [])]
    return {
        "schema": 1,
        "case": {
            "name": case["name"],
            "path": case["path"],
            "description": case.get("raw", {}).get("description", ""),
        },
        "launch": effective,
        "scene": {
            "name": case["scene"].name,
            "path": str(case["scene"].path),
            "variant": case.get("scene_spec", {}).get("variant", ""),
            "parameters": case.get("scene_spec", {}).get("parameters", {}),
        },
        "task": {
            "type": case.get("task_type", ""),
            "regions": case.get("task_regions", []),
        },
        "module": module,
        "adapters": adapters,
        "external_packages": _external_packages(case, effective),
        "external_launches": _external_launches(module),
        "command": [sys.argv[0], *(sys.argv[1:] if args else [])],
        "git_commit": _git_commit(),
        "started_at": _utc_now(),
        "finished_at": None,
        "run_dir": str(run_dir),
        "passed": False,
        "exit_code": None,
        "artifacts": {},
    }


def _git_commit():
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except Exception:
        return ""
    return result.stdout.strip() if result.returncode == 0 else ""


def _run_step(metrics, runner, name, command, log_path, timeout=None):
    step = {
        "name": name,
        "passed": False,
        "status": "RUNNING",
        "started_at": _utc_now(),
        "log": str(log_path),
        "command": [str(item) for item in command],
    }
    metrics["steps"].append(step)
    try:
        result = runner.run(command, log_path, timeout=timeout)
        step.update(result)
        step["passed"] = result["returncode"] == 0
        step["status"] = _step_status(step["passed"])
        if result["returncode"] != 0:
            raise RuntimeError(f"step '{name}' failed; see {log_path}")
    except subprocess.TimeoutExpired as exc:
        step["duration_sec"] = None
        step["error"] = f"timeout after {exc.timeout}s"
        step["status"] = "FAIL"
        raise RuntimeError(f"step '{name}' timed out; see {log_path}") from exc
    finally:
        if step["status"] == "RUNNING":
            step["status"] = _step_status(step.get("passed", False))
        step["finished_at"] = _utc_now()


def _record_manual_step(metrics, name, passed, message, log_path):
    metrics["steps"].append({
        "name": name,
        "passed": bool(passed),
        "status": _step_status(passed),
        "message": message,
        "log": str(log_path) if log_path else "",
        "started_at": _utc_now(),
        "finished_at": _utc_now(),
        "returncode": 0 if passed else 1,
    })


def _start_simulation(runner, effective, log_path):
    launch_file = "distributed_local.launch.py" if effective.get("layout") == "distributed" else "sim.launch.py"
    command = [
        "ros2",
        "launch",
        "robot_sim_bringup",
        launch_file,
        f"sim_profile:={effective['profile']}",
        f"sim_mode:={effective['mode']}",
        "headless:=true",
        "rviz:=false",
        "use_moveit:=true",
    ]
    if effective.get("use_gripper"):
        command.append("use_gripper:=true")
    if effective["profile_file"]:
        command.append(f"sim_profile_file:={effective['profile_file']}")
    if effective["sensor_overrides"]:
        command.append(f"sensor_overrides:={effective['sensor_overrides']}")

    process = runner.popen(command, log_path)
    time.sleep(2.0)
    if process.poll() is not None:
        raise RuntimeError(f"simulation process exited early; see {log_path}")
    return process


def _wait_gazebo_model(metrics, model_name, timeout, sim_process, log_path):
    started = time.monotonic()
    deadline = started + timeout
    with open(log_path, "w", encoding="utf-8") as handle:
        while time.monotonic() < deadline:
            if sim_process.poll() is not None:
                handle.write("simulation process exited early\n")
                _record_manual_step(metrics, "gazebo_spawn", False, "simulation process exited early", log_path)
                raise RuntimeError(f"simulation process exited early; see {log_path}")
            result = subprocess.run(
                ["gz", "model", "--list"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
            )
            handle.write(result.stdout)
            models = {
                line.strip().lstrip("-").strip()
                for line in result.stdout.splitlines()
            }
            if model_name in models:
                _record_manual_step(metrics, "gazebo_spawn", True, f"model spawned: {model_name}", log_path)
                return
            time.sleep(1.0)
    _record_manual_step(metrics, "gazebo_spawn", False, f"timed out waiting for model: {model_name}", log_path)
    raise RuntimeError(f"timed out waiting for Gazebo model '{model_name}'")


def _record_rosbag(runner, case, effective, rosbag_dir, log_path, duration_sec):
    rosbag_dir.mkdir(parents=True, exist_ok=True)
    rosbag_config = case["artifacts"]["rosbag"]
    extra_topics = " ".join(rosbag_config.get("extra_topics", []))
    command = [
        "ros2",
        "launch",
        "robot_sim_bringup",
        "record_bag.launch.py",
        f"sim_profile:={effective['profile']}",
        f"sim_profile_file:={effective['profile_file']}",
        f"layout:={effective.get('layout', 'single')}",
        f"topic_group:={rosbag_config.get('topic_group', 'all')}",
        f"output_dir:={rosbag_dir}",
        f"bag_name:={case['name']}",
        f"compression:={'true' if rosbag_config.get('compression', False) else 'false'}",
    ]
    if effective["sensor_overrides"]:
        command.append(f"sensor_overrides:={effective['sensor_overrides']}")
    if extra_topics:
        command.append(f"extra_topics:={extra_topics}")
    process = runner.popen(command, log_path)
    time.sleep(max(0.0, duration_sec))
    _terminate_process(process, signal.SIGINT)
    return process


def _terminate_process(process, sig):
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), sig)
        except ProcessLookupError:
            pass
        time.sleep(1.0)
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        pass
    handle = getattr(process, "_robot_sim_log_handle", None)
    if handle is not None:
        handle.close()


def _validation_metrics_summary(validation_metrics):
    return {
        "plan_success_rate": validation_metrics.get("plan_success_rate"),
        "planning_time_sec": validation_metrics.get("planning_time_sec"),
        "execution_time_sec": validation_metrics.get("execution_time_sec"),
        "goal_position_error_m": validation_metrics.get("goal_position_error_m"),
        "min_tcp_clearance_m": validation_metrics.get("min_tcp_clearance_m"),
        "max_controller_error_rad": validation_metrics.get("max_controller_error_rad"),
        "peak_controller_error_rad": validation_metrics.get("peak_controller_error_rad"),
        "sensor_hz": validation_metrics.get("sensor_hz", {}),
        "expected_topics": validation_metrics.get("expected_topics", {}),
        "tf_ok": validation_metrics.get("tf_ok"),
        "moveit_error_code": validation_metrics.get("moveit_error_code"),
        "validation_failures": validation_metrics.get("failures", []),
        "phases": validation_metrics.get("phases", []),
        "business_actions": validation_metrics.get("business_actions", []),
        "task_events": validation_metrics.get("task_events", []),
        "adapter_health": validation_metrics.get("adapter_health", []),
        "module_services": validation_metrics.get("module_services", []),
        "module_topics": validation_metrics.get("module_topics", []),
        "module_events": validation_metrics.get("module_events", []),
        "module_failures": validation_metrics.get("module_failures", []),
        "adapter_data_sources": validation_metrics.get("adapter_data_sources", []),
    }


def _write_reports(run_dir, manifest, metrics):
    report_md = _render_markdown_report(manifest, metrics)
    (run_dir / "report.md").write_text(report_md, encoding="utf-8")
    report_html = _render_html_report(report_md)
    (run_dir / "report.html").write_text(report_html, encoding="utf-8")


def _render_markdown_report(manifest, metrics):
    status = "PASS" if metrics.get("passed") else "FAIL"
    lines = [
        f"# robot_sim Validation Report: {manifest['case']['name']}",
        "",
        f"- Status: **{status}**",
        f"- Profile: `{manifest['launch']['profile']}`",
        f"- Mode: `{manifest['launch']['mode']}`",
        f"- Scene: `{manifest['scene']['name']}`",
        f"- Task: `{manifest.get('task', {}).get('type', metrics.get('task_type', ''))}`",
        f"- Started: `{manifest.get('started_at')}`",
        f"- Finished: `{manifest.get('finished_at')}`",
        f"- Run directory: `{manifest['run_dir']}`",
        "",
    ]
    if metrics.get("error"):
        lines.extend(["## Failure", "", metrics["error"], ""])

    lines.extend([
        "## Steps",
        "",
        "| Step | Status | Duration | Log |",
        "| --- | --- | ---: | --- |",
    ])
    for step in metrics.get("steps", []):
        step_status = step.get("status") or _step_status(step.get("passed"))
        duration = step.get("duration_sec")
        duration_text = f"{duration:.2f}s" if isinstance(duration, (int, float)) else ""
        lines.append(f"| {step['name']} | {step_status} | {duration_text} | `{step.get('log', '')}` |")

    lines.extend([
        "",
        "## Business Actions",
        "",
        "| Action | Type | Detail |",
        "| --- | --- | --- |",
    ])
    for action in metrics.get("business_actions", []):
        detail = ", ".join(
            f"{key}={value}"
            for key, value in action.items()
            if key not in ("name", "type")
        )
        lines.append(f"| {action.get('name', '')} | {action.get('type', '')} | `{detail}` |")

    lines.extend([
        "",
        "## Task Events",
        "",
        "| Event | OK | Detail |",
        "| --- | --- | --- |",
    ])
    for event in metrics.get("task_events", []):
        lines.append(
            f"| {event.get('name', '')} | {event.get('ok', '')} | `{event.get('detail', '')}` |"
        )

    if _has_module_metrics(metrics):
        lines.extend([
            "",
            "## External Module",
            "",
        ])
        failures = metrics.get("module_failures", [])
        lines.append(f"- Module failures: `{len(failures)}`")
        for failure in failures:
            lines.append(f"  - `{failure}`")
        lines.extend([
            "",
            "| Adapter | Type | Status | Log |",
            "| --- | --- | --- | --- |",
        ])
        for adapter in metrics.get("adapter_health", []):
            lines.append(
                f"| {adapter.get('name', '')} | {adapter.get('type', '')} | {adapter.get('status', '')} | `{adapter.get('log', '')}` |"
            )
        if metrics.get("adapter_data_sources"):
            lines.extend([
                "",
                "| Adapter Data Source | Policy | Frame | Image | Point Cloud |",
                "| --- | --- | --- | --- | --- |",
            ])
            for source in metrics.get("adapter_data_sources", []):
                for frame in source.get("frames", []):
                    point_cloud = _point_cloud_report_text(frame)
                    lines.append(
                        f"| {source.get('adapter', '')} | {source.get('frame_policy', '')} | "
                        f"{frame.get('data_source', '')} | `{frame.get('image_path', '')}` | `{point_cloud}` |"
                    )
        lines.extend([
            "",
            "| Service/Action | Type | OK | Duration |",
            "| --- | --- | --- | ---: |",
        ])
        for service in metrics.get("module_services", []):
            duration = service.get("duration_sec")
            duration_text = f"{duration:.2f}s" if isinstance(duration, (int, float)) else ""
            lines.append(
                f"| {service.get('name') or service.get('service', '')} | {service.get('service_type', '')} | {service.get('ok', '')} | {duration_text} |"
            )
        lines.extend([
            "",
            "| Topic | Type | Count | OK |",
            "| --- | --- | ---: | --- |",
        ])
        for topic in metrics.get("module_topics", []):
            lines.append(
                f"| `{topic.get('name', '')}` | `{topic.get('type', '')}` | {topic.get('count', '')} | {topic.get('ok', '')} |"
            )
        lines.extend([
            "",
            "| Event | Type | OK |",
            "| --- | --- | --- |",
        ])
        for event in metrics.get("module_events", []):
            lines.append(
                f"| {event.get('name', '')} | {event.get('type', '')} | {event.get('ok', '')} |"
            )

    lines.extend([
        "",
        "## Metrics",
        "",
        f"- MoveIt plan success rate: `{metrics.get('plan_success_rate', '')}`",
        f"- Planning time: `{metrics.get('planning_time_sec', '')}` sec",
        f"- Execution time: `{metrics.get('execution_time_sec', '')}` sec",
        f"- Goal position error: `{metrics.get('goal_position_error_m', '')}` m",
        f"- Minimum TCP clearance: `{metrics.get('min_tcp_clearance_m', '')}` m",
        f"- Max controller error: `{metrics.get('max_controller_error_rad', '')}` rad",
        f"- TF OK: `{metrics.get('tf_ok', '')}`",
        f"- MoveIt error code: `{metrics.get('moveit_error_code', '')}`",
        "",
        "## Sensor Hz",
        "",
        "| Topic | Hz | Samples | OK |",
        "| --- | ---: | ---: | --- |",
    ])
    for topic, values in sorted(metrics.get("sensor_hz", {}).items()):
        lines.append(
            f"| `{topic}` | {values.get('hz', 0.0):.2f} | {values.get('samples', '')} | {values.get('ok', False)} |"
        )

    lines.extend([
        "",
        "## Artifacts",
        "",
        f"- Manifest: `{manifest['artifacts'].get('manifest', '')}`",
        f"- Metrics: `{manifest['artifacts'].get('metrics', '')}`",
        f"- Effective case: `{manifest['artifacts'].get('effective_case', '')}`",
        f"- Effective profile: `{manifest['artifacts'].get('effective_profile', '')}`",
        f"- Rosbag: `{manifest['artifacts'].get('rosbag', '')}`",
        f"- Simulation log: `{manifest['artifacts'].get('simulation_log', '')}`",
        "",
    ])
    return "\n".join(lines)


def _render_html_report(markdown_text):
    escaped = html.escape(markdown_text)
    return (
        "<!doctype html>\n"
        "<html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
        "<title>robot_sim validation report</title>"
        "<style>body{font-family:system-ui,sans-serif;max-width:980px;margin:32px auto;line-height:1.55;}"
        "pre{white-space:pre-wrap;background:#f6f8fa;padding:16px;border-radius:6px;}</style>"
        "</head><body><pre>"
        + escaped
        + "</pre></body></html>\n"
    )


def _artifact_manifest(run_dir):
    rosbag_entries = list((run_dir / "rosbag").glob("*/metadata.yaml"))
    rosbag = str(rosbag_entries[0].parent) if rosbag_entries else ""
    return {
        "manifest": str(run_dir / "manifest.json"),
        "metrics": str(run_dir / "metrics.json"),
        "effective_case": str(run_dir / "effective_case.yaml"),
        "effective_profile": str(run_dir / "effective_profile.yaml"),
        "report_md": str(run_dir / "report.md"),
        "report_html": str(run_dir / "report.html"),
        "robot_urdf": str(run_dir / "robot.urdf"),
        "simulation_log": str(run_dir / "logs" / "sim.launch.log"),
        "rosbag": rosbag,
    }


def _external_packages(case, effective):
    packages = set()
    if effective.get("profile_package"):
        packages.add(str(effective["profile_package"]))
    module = case.get("module", {})
    launch = module.get("launch", {})
    if isinstance(launch, dict) and launch.get("package"):
        packages.add(str(launch["package"]))
    for command_spec in module.get("commands", []) if isinstance(module, dict) else []:
        if not isinstance(command_spec, dict):
            continue
        command = command_spec.get("command", [])
        if len(command) >= 3 and command[0] == "ros2" and command[1] == "run":
            packages.add(str(command[2]))
    return sorted(packages)


def _external_launches(module):
    launch = module.get("launch", {}) if isinstance(module, dict) else {}
    if not isinstance(launch, dict) or not launch.get("package") or not launch.get("file"):
        return []
    return [{
        "package": str(launch["package"]),
        "file": str(launch["file"]),
        "arguments": dict(launch.get("arguments", {})),
    }]


def _has_module_metrics(metrics):
    return any(
        metrics.get(key)
        for key in (
            "adapter_health",
            "module_services",
            "module_topics",
            "module_events",
            "module_failures",
            "adapter_data_sources",
        )
    )


def _point_cloud_report_text(frame):
    if frame.get("point_cloud_path"):
        return str(frame["point_cloud_path"])
    if frame.get("point_cloud_source") == "synthetic":
        params = frame.get("synthetic_point_cloud", {})
        return (
            "synthetic"
            f" x_span_m={params.get('x_span_m', '')}"
            f" y_span_m={params.get('y_span_m', '')}"
            f" z_m={params.get('z_m', '')}"
        )
    return str(frame.get("point_cloud_source", ""))


def _load_json_file(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _write_json(path, value):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, sort_keys=True)
        handle.write("\n")


def _utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def _step_status(passed):
    return "PASS" if passed else "FAIL"


def _safe_id(value):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_")


def _is_schema4_case(path):
    try:
        with Path(path).open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
    except Exception:
        return False
    return isinstance(raw, dict) and raw.get("schema") == 4 and raw.get("kind") == "validation_case"


if __name__ == "__main__":
    sys.exit(main())
