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

from robot_sim_bringup.validation_cases import load_validation_case


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
    parser.add_argument("--output-dir", default="robot_sim_runs", help="Parent directory for run artifacts.")
    parser.add_argument("--profile", default="", help="Override case launch.profile.")
    parser.add_argument("--profile-file", default="", help="Override or provide an external sim_profile YAML.")
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
    case = load_validation_case(args.case)
    effective = _effective_launch(case, args)
    run_dir = _create_run_dir(args.output_dir, case["name"], effective["profile"])
    logs_dir = run_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    rosbag_dir = run_dir / "rosbag"

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
        helper = [sys.executable, "-m", "robot_sim_bringup.sim_smoke_helper"]
        linter = [sys.executable, "-m", "robot_sim_bringup.profile_lint"]

        _run_step(metrics, runner, "profile_lint", linter + lint_args, logs_dir / "profile_lint.log")
        profile_json_log = logs_dir / "profile.json"
        _run_step(metrics, runner, "profile_summary", helper + ["profile-json", *common_args, "--with-moveit"], profile_json_log)
        profile_summary = _load_json_file(profile_json_log)

        urdf_path = run_dir / "robot.urdf"
        _run_step(metrics, runner, "render_urdf", helper + ["render-urdf", *common_args, "--output", str(urdf_path)], logs_dir / "render_urdf.log")
        _run_step(metrics, runner, "validate_urdf", ["check_urdf", str(urdf_path)], logs_dir / "check_urdf.log")

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

        _run_step(metrics, runner, "joint_states", helper + ["wait-joint-state", *common_args, "--timeout", str(effective["timeout"])], logs_dir / "joint_states.log")
        _run_step(metrics, runner, "controllers_active", helper + ["wait-controllers", *common_args, "--timeout", str(effective["timeout"])], logs_dir / "controllers_active.log")
        _run_step(metrics, runner, "trajectory_action", helper + ["send-trajectory", *common_args, "--timeout", str(effective["timeout"])], logs_dir / "trajectory_action.log")
        _run_step(metrics, runner, "sensor_hz", helper + ["check-sensors", *common_args], logs_dir / "sensor_hz.log")
        _run_step(metrics, runner, "tf_tree", helper + ["check-tf", *common_args, "--urdf", str(urdf_path)], logs_dir / "tf_tree.log")
        _run_step(metrics, runner, "moveit_plan_execute", helper + ["moveit", *common_args, "--timeout", str(effective["timeout"])], logs_dir / "moveit.log")

        validation_metrics_path = run_dir / "validation_metrics.json"
        _run_step(
            metrics,
            runner,
            "validation_case",
            helper + [
                "validate-case",
                *common_args,
                "--validation-case",
                args.case,
                "--metrics-output",
                str(validation_metrics_path),
                "--timeout",
                str(effective["timeout"]),
            ],
            logs_dir / "validation_case.log",
        )
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
    return {
        "profile": args.profile or case["profile"],
        "profile_file": args.profile_file or case.get("profile_file", ""),
        "mode": args.mode or case["mode"],
        "layout": case.get("layout", "single"),
        "timeout": float(args.timeout if args.timeout is not None else case.get("timeout_sec", 120.0)),
        "sensor_overrides": (
            args.sensor_overrides
            if args.sensor_overrides is not None
            else case.get("sensor_overrides", "")
        ),
    }


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
        },
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


def _run_step(metrics, runner, name, command, log_path):
    step = {
        "name": name,
        "passed": False,
        "started_at": _utc_now(),
        "log": str(log_path),
        "command": [str(item) for item in command],
    }
    metrics["steps"].append(step)
    try:
        result = runner.run(command, log_path)
        step.update(result)
        step["passed"] = result["returncode"] == 0
        if result["returncode"] != 0:
            raise RuntimeError(f"step '{name}' failed; see {log_path}")
    except subprocess.TimeoutExpired as exc:
        step["duration_sec"] = None
        step["error"] = f"timeout after {exc.timeout}s"
        raise RuntimeError(f"step '{name}' timed out; see {log_path}") from exc
    finally:
        step["finished_at"] = _utc_now()


def _record_manual_step(metrics, name, passed, message, log_path):
    metrics["steps"].append({
        "name": name,
        "passed": bool(passed),
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
        f"sensor_overrides:={effective['sensor_overrides']}",
        f"topic_group:={rosbag_config.get('topic_group', 'all')}",
        f"output_dir:={rosbag_dir}",
        f"bag_name:={case['name']}",
        f"compression:={'true' if rosbag_config.get('compression', False) else 'false'}",
        f"extra_topics:={extra_topics}",
    ]
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
        step_status = "PASS" if step.get("passed") else "FAIL"
        duration = step.get("duration_sec")
        duration_text = f"{duration:.2f}s" if isinstance(duration, (int, float)) else ""
        lines.append(f"| {step['name']} | {step_status} | {duration_text} | `{step.get('log', '')}` |")

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
        "report_md": str(run_dir / "report.md"),
        "report_html": str(run_dir / "report.html"),
        "robot_urdf": str(run_dir / "robot.urdf"),
        "simulation_log": str(run_dir / "logs" / "sim.launch.log"),
        "rosbag": rosbag,
    }


def _load_json_file(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _write_json(path, value):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, sort_keys=True)
        handle.write("\n")


def _utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def _safe_id(value):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_")


if __name__ == "__main__":
    sys.exit(main())
