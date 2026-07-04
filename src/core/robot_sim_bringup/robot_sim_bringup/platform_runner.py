from __future__ import annotations

from datetime import datetime, timezone
import html
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Any, Mapping

import yaml

from robot_sim_bringup.platform_adapter import preflight_adapter
from robot_sim_bringup.platform_assertions import evaluate_assertions
from robot_sim_bringup.platform_config import load_platform_validation_case


SUCCESS = 0
FAILURE = 1


def is_platform_case(path: str | Path) -> bool:
    with Path(path).open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    return isinstance(raw, dict) and raw.get("schema") == 4 and raw.get("kind") == "validation_case"


def run_platform_case(
    args: Any,
    runner: Any,
    parameter_overrides: Mapping[str, Any] | None = None,
    run_name_suffix: str = "",
) -> int:
    case = load_platform_validation_case(
        args.case,
        case_package=getattr(args, "case_package", ""),
        parameter_overrides=parameter_overrides,
    )
    run_dir = _create_run_dir(getattr(args, "output_dir", "robot_sim_runs"), case["name"], run_name_suffix)
    logs_dir = run_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    adapter_dir = run_dir / "adapters"
    adapter_dir.mkdir(parents=True, exist_ok=True)
    rosbag_dir = run_dir / "rosbag"
    effective_case_file = _write_yaml(run_dir / "effective_case.yaml", case["raw"])

    manifest = _initial_manifest(case, run_dir, args)
    metrics: dict[str, Any] = {
        "case_name": case["name"],
        "schema": 4,
        "system_type": case["system"].get("type", "ros2_pipeline"),
        "passed": False,
        "steps": [],
        "system_processes": [],
        "adapter_health": [],
        "data_sources": list(case.get("data_sources", [])),
        "actions": [],
        "assertions": [],
        "artifacts": {
            "run_dir": str(run_dir),
            "logs_dir": str(logs_dir),
            "rosbag_dir": str(rosbag_dir),
        },
    }
    processes: list[tuple[str, subprocess.Popen]] = []
    bag_process = None
    exit_code = FAILURE
    action_results: dict[str, Any] = {}
    process_results: dict[str, Any] = {}

    try:
        _write_json(run_dir / "manifest.json", manifest)
        _record_manual_step(metrics, "load_config", True, "schema v4 platform case loaded", effective_case_file)

        _preflight_case(case)
        _record_manual_step(metrics, "preflight", True, "platform preflight passed", None)

        for spec in case["system"].get("processes", []):
            result = _start_process_spec(runner, spec, logs_dir, "system")
            metrics["system_processes"].append(result["summary"])
            if result.get("process") is not None:
                processes.append((result["name"], result["process"]))
            else:
                process_results[result["name"]] = result["summary"]
                if spec.get("required", True) and result["summary"].get("returncode", 0) != 0:
                    raise RuntimeError(f"system process failed: {result['name']}")

        startup_delay = float(case["system"].get("startup_delay_sec", 0.0) or 0.0)
        if startup_delay > 0:
            time.sleep(startup_delay)

        for adapter in [*case.get("adapters", []), *case.get("inputs", [])]:
            result = _start_adapter(runner, adapter, adapter_dir, logs_dir)
            metrics["adapter_health"].append(result["summary"])
            if result.get("process") is not None:
                processes.append((result["name"], result["process"]))

        for action in case.get("actions", []):
            result = _run_action(runner, action, logs_dir)
            action_results[result["name"]] = result
            metrics["actions"].append(result)
            if not result.get("ok", False):
                raise RuntimeError(f"action failed: {result['name']}")

        if case["artifacts"]["rosbag"]["enabled"] and not getattr(args, "no_rosbag", False):
            bag_process = _record_rosbag(runner, case, rosbag_dir, logs_dir / "rosbag.log", getattr(args, "rosbag_duration", None))
            _record_manual_step(metrics, "rosbag_record", True, f"rosbag: {rosbag_dir}", logs_dir / "rosbag.log")
        else:
            _record_manual_step(metrics, "rosbag_record", True, "rosbag disabled", None)

        process_results.update(_poll_background_processes(processes))
        assertion_context = {
            "actions": action_results,
            "processes": process_results,
            "metrics": metrics,
        }
        assertion_results = evaluate_assertions(case.get("assertions", []), assertion_context)
        metrics["assertions"] = assertion_results
        failures = [item for item in assertion_results if not item.get("ok", False)]
        if failures:
            raise RuntimeError("assertion failed: " + failures[0]["name"])

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
        for _name, process in reversed(processes):
            _terminate_process(process, signal.SIGINT)
        manifest["finished_at"] = _utc_now()
        manifest["passed"] = metrics.get("passed", False)
        manifest["exit_code"] = exit_code
        manifest["artifacts"] = _artifact_manifest(run_dir)
        _write_json(run_dir / "metrics.json", metrics)
        _write_json(run_dir / "manifest.json", manifest)
        _write_reports(run_dir, manifest, metrics)
        print(f"Artifacts: {run_dir}")


def _preflight_case(case: Mapping[str, Any]) -> None:
    for adapter in [*case.get("adapters", []), *case.get("inputs", [])]:
        adapter_type = str(adapter.get("type", ""))
        if adapter_type in ("process_supervisor", "rosbag_replay"):
            continue
        preflight_adapter(adapter)


def _start_process_spec(runner: Any, spec: Mapping[str, Any], logs_dir: Path, group: str) -> dict[str, Any]:
    name = str(spec.get("name", "process"))
    command = [str(part) for part in spec.get("command", [])]
    if not command:
        raise RuntimeError(f"{group} process '{name}' has empty command")
    log_path = logs_dir / f"{group}_{_safe_id(name)}.log"
    env = _merged_env(spec.get("env", {}))
    if bool(spec.get("background", True)):
        process = runner.popen(command, log_path, env=env)
        time.sleep(0.5)
        status = "STARTED" if process.poll() is None else "EXITED"
        summary = {
            "name": name,
            "command": command,
            "background": True,
            "status": status,
            "returncode": process.poll(),
            "log": str(log_path),
        }
        if process.poll() is not None and bool(spec.get("required", True)):
            raise RuntimeError(f"{group} process exited early: {name}; see {log_path}")
        return {"name": name, "process": process, "summary": summary}
    result = runner.run(command, log_path, timeout=_timeout_value(spec.get("timeout_sec", 0.0)), env=env)
    summary = {
        "name": name,
        "command": command,
        "background": False,
        "status": "PASS" if result["returncode"] == 0 else "FAIL",
        "returncode": result["returncode"],
        "duration_sec": result.get("duration_sec"),
        "log": str(log_path),
    }
    return {"name": name, "process": None, "summary": summary}


def _start_adapter(runner: Any, adapter: Mapping[str, Any], adapter_dir: Path, logs_dir: Path) -> dict[str, Any]:
    adapter_type = str(adapter.get("type", ""))
    name = str(adapter.get("name") or adapter_type)
    if adapter_type == "process_supervisor":
        spec = {
            "name": name,
            "command": adapter.get("command", []),
            "background": adapter.get("background", True),
            "timeout_sec": adapter.get("timeout_sec", 0.0),
            "required": adapter.get("required", True),
            "env": adapter.get("env", {}),
        }
        return _start_process_spec(runner, spec, logs_dir, "adapter")
    adapter_file = adapter_dir / f"{_safe_id(name)}.yaml"
    _write_yaml(adapter_file, dict(adapter))
    log_path = logs_dir / f"adapter_{_safe_id(name)}.log"
    command = [
        sys.executable,
        "-m",
        "robot_sim_bringup.platform_adapter",
        "--adapter-file",
        str(adapter_file),
    ]
    process = runner.popen(command, log_path, env=_merged_env(adapter.get("env", {})))
    time.sleep(0.5)
    status = "STARTED" if process.poll() is None else "EXITED"
    if process.poll() is not None and adapter.get("required", True):
        raise RuntimeError(f"adapter exited early: {name}; see {log_path}")
    return {
        "name": name,
        "process": process,
        "summary": {
            "name": name,
            "type": adapter_type,
            "status": status,
            "returncode": process.poll(),
            "log": str(log_path),
        },
    }


def _run_action(runner: Any, action: Mapping[str, Any], logs_dir: Path) -> dict[str, Any]:
    action_type = str(action.get("type", ""))
    name = str(action.get("name") or action_type)
    started = time.monotonic()
    if action_type == "command":
        log_path = logs_dir / f"action_{_safe_id(name)}.log"
        result = runner.run(
            [str(part) for part in action.get("command", [])],
            log_path,
            timeout=_timeout_value(action.get("timeout_sec", 0.0)),
            env=_merged_env(action.get("env", {})),
        )
        return {
            "name": name,
            "type": action_type,
            "ok": result["returncode"] == 0,
            "returncode": result["returncode"],
            "duration_sec": result.get("duration_sec", time.monotonic() - started),
            "log": str(log_path),
            "command": result.get("command", []),
        }
    if action_type == "sleep":
        duration = float(action.get("duration_sec", 1.0))
        time.sleep(duration)
        return {"name": name, "type": action_type, "ok": True, "duration_sec": duration}
    if action_type == "service_call":
        return _service_call_action(action, started)
    if action_type == "wait_service":
        return _wait_service_action(action, started)
    if action_type == "wait_topic":
        return _wait_topic_action(action, started)
    return {
        "name": name,
        "type": action_type,
        "ok": False,
        "duration_sec": time.monotonic() - started,
        "error": f"unsupported action type: {action_type}",
    }


def _service_call_action(action: Mapping[str, Any], started: float) -> dict[str, Any]:
    import rclpy
    from rosidl_runtime_py.utilities import get_service
    from robot_sim_bringup.platform_assertions import assign_fields, message_to_dict

    Service = get_service(str(action["service_type"]))
    service = str(action["service"])
    timeout = float(action.get("timeout_sec", 5.0))
    rclpy.init(args=None)
    node = rclpy.create_node("robot_sim_platform_service_action")
    try:
        client = node.create_client(Service, service)
        if not client.wait_for_service(timeout_sec=timeout):
            return _action_failure(action, started, f"service unavailable: {service}")
        request = Service.Request()
        assign_fields(request, action.get("request", {}))
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done():
            return _action_failure(action, started, f"service call timed out: {service}")
        return {
            "name": str(action.get("name") or service),
            "type": "service_call",
            "ok": True,
            "duration_sec": time.monotonic() - started,
            "service": service,
            "response": message_to_dict(future.result()),
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _wait_service_action(action: Mapping[str, Any], started: float) -> dict[str, Any]:
    import rclpy
    from rosidl_runtime_py.utilities import get_service

    Service = get_service(str(action["service_type"]))
    service = str(action["service"])
    timeout = float(action.get("timeout_sec", 5.0))
    rclpy.init(args=None)
    node = rclpy.create_node("robot_sim_platform_wait_service_action")
    try:
        client = node.create_client(Service, service)
        ok = client.wait_for_service(timeout_sec=timeout)
        return {
            "name": str(action.get("name") or service),
            "type": "wait_service",
            "ok": bool(ok),
            "duration_sec": time.monotonic() - started,
            "service": service,
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _wait_topic_action(action: Mapping[str, Any], started: float) -> dict[str, Any]:
    import rclpy
    from rosidl_runtime_py.utilities import get_message

    Message = get_message(str(action["message_type"]))
    topic = str(action["topic"])
    timeout = float(action.get("timeout_sec", 5.0))
    min_count = int(action.get("min_count", 1))
    messages = []
    rclpy.init(args=None)
    node = rclpy.create_node("robot_sim_platform_wait_topic_action")
    try:
        node.create_subscription(Message, topic, lambda msg: messages.append(msg), 10)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline and len(messages) < min_count:
            rclpy.spin_once(node, timeout_sec=0.1)
        return {
            "name": str(action.get("name") or topic),
            "type": "wait_topic",
            "ok": len(messages) >= min_count,
            "duration_sec": time.monotonic() - started,
            "topic": topic,
            "count": len(messages),
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _action_failure(action: Mapping[str, Any], started: float, error: str) -> dict[str, Any]:
    return {
        "name": str(action.get("name") or action.get("type", "")),
        "type": str(action.get("type", "")),
        "ok": False,
        "duration_sec": time.monotonic() - started,
        "error": error,
    }


def _record_rosbag(runner: Any, case: Mapping[str, Any], rosbag_dir: Path, log_path: Path, duration_override: float | None):
    rosbag = case["artifacts"]["rosbag"]
    topics = [str(topic) for topic in rosbag.get("topics", [])]
    if not topics:
        raise RuntimeError("v4 rosbag recording requires artifacts.rosbag.topics")
    output = rosbag_dir / case["name"]
    command = ["ros2", "bag", "record", "-o", str(output), *topics]
    process = runner.popen(command, log_path)
    duration = float(duration_override if duration_override is not None else rosbag.get("duration_sec", 5.0))
    time.sleep(max(duration, 0.0))
    _terminate_process(process, signal.SIGINT)
    return process


def _poll_background_processes(processes: list[tuple[str, subprocess.Popen]]) -> dict[str, Any]:
    result = {}
    for name, process in processes:
        result[name] = {
            "name": name,
            "returncode": process.poll(),
            "running": process.poll() is None,
        }
    return result


def _create_run_dir(output_dir: str, case_name: str, suffix: str = "") -> Path:
    parent = Path(output_dir).expanduser().resolve()
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    name = f"{timestamp}_{_safe_id(case_name)}"
    if suffix:
        name += f"_{_safe_id(suffix)}"
    base = parent / name
    candidate = base
    index = 1
    while candidate.exists():
        index += 1
        candidate = Path(f"{base}_{index}")
    candidate.mkdir(parents=True)
    return candidate


def _initial_manifest(case: Mapping[str, Any], run_dir: Path, args: Any) -> dict[str, Any]:
    return {
        "schema": 2,
        "case": {
            "name": case["name"],
            "path": case["path"],
            "description": case.get("description", ""),
        },
        "system": case["system"],
        "system_profile": {
            "name": case.get("system_profile", ""),
            "path": case.get("system_profile_path", ""),
        },
        "data_sources": list(case.get("data_sources", [])),
        "command": [sys.argv[0], *(sys.argv[1:] if args else [])],
        "started_at": _utc_now(),
        "finished_at": None,
        "run_dir": str(run_dir),
        "passed": False,
        "exit_code": None,
        "artifacts": {},
    }


def _write_reports(run_dir: Path, manifest: Mapping[str, Any], metrics: Mapping[str, Any]) -> None:
    markdown = _render_markdown_report(manifest, metrics)
    (run_dir / "report.md").write_text(markdown, encoding="utf-8")
    (run_dir / "report.html").write_text(_render_html_report(markdown), encoding="utf-8")


def _render_markdown_report(manifest: Mapping[str, Any], metrics: Mapping[str, Any]) -> str:
    status = "PASS" if metrics.get("passed") else "FAIL"
    lines = [
        f"# robot_sim Platform Report: {manifest['case']['name']}",
        "",
        f"- Status: **{status}**",
        f"- System type: `{metrics.get('system_type', '')}`",
        f"- Started: `{manifest.get('started_at')}`",
        f"- Finished: `{manifest.get('finished_at')}`",
        f"- Run directory: `{manifest['run_dir']}`",
        "",
    ]
    if metrics.get("error"):
        lines.extend(["## Failure", "", str(metrics["error"]), ""])
    lines.extend(["## Steps", "", "| Step | Status | Detail |", "| --- | --- | --- |"])
    for step in metrics.get("steps", []):
        lines.append(f"| {step.get('name', '')} | {step.get('status', '')} | `{step.get('message', step.get('log', ''))}` |")
    lines.extend(["", "## Actions", "", "| Action | Type | OK | Detail |", "| --- | --- | --- | --- |"])
    for action in metrics.get("actions", []):
        detail = action.get("error") or action.get("log") or action.get("service") or action.get("topic") or ""
        lines.append(f"| {action.get('name', '')} | {action.get('type', '')} | {action.get('ok', '')} | `{detail}` |")
    lines.extend(["", "## Assertions", "", "| Assertion | Type | OK | Message |", "| --- | --- | --- | --- |"])
    for assertion in metrics.get("assertions", []):
        lines.append(f"| {assertion.get('name', '')} | {assertion.get('type', '')} | {assertion.get('ok', '')} | `{assertion.get('message', '')}` |")
    lines.extend(["", "## Adapters", "", "| Adapter | Type | Status | Log |", "| --- | --- | --- | --- |"])
    for adapter in metrics.get("adapter_health", []):
        lines.append(f"| {adapter.get('name', '')} | {adapter.get('type', '')} | {adapter.get('status', '')} | `{adapter.get('log', '')}` |")
    if metrics.get("data_sources"):
        lines.extend(["", "## Data Sources", "", "| Name | Type | Topic | Message Type | Records | Path |", "| --- | --- | --- | --- | ---: | --- |"])
        for source in metrics.get("data_sources", []):
            lines.append(
                f"| {source.get('name', '')} | {source.get('type', '')} | `{source.get('topic', '')}` | "
                f"`{source.get('message_type', '')}` | {source.get('records', '')} | `{source.get('path') or source.get('source_path', '')}` |"
            )
    lines.extend(["", "## Artifacts", "", f"- Metrics: `{manifest['artifacts'].get('metrics', '')}`", f"- Manifest: `{manifest['artifacts'].get('manifest', '')}`", ""])
    return "\n".join(lines)


def _render_html_report(markdown_text: str) -> str:
    escaped = html.escape(markdown_text)
    return (
        "<!doctype html>\n<html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
        "<title>robot_sim platform report</title>"
        "<style>body{font-family:system-ui,sans-serif;max-width:980px;margin:32px auto;line-height:1.55;}"
        "pre{white-space:pre-wrap;background:#f6f8fa;padding:16px;border-radius:6px;}</style>"
        "</head><body><pre>"
        + escaped
        + "</pre></body></html>\n"
    )


def _artifact_manifest(run_dir: Path) -> dict[str, str]:
    return {
        "manifest": str(run_dir / "manifest.json"),
        "metrics": str(run_dir / "metrics.json"),
        "effective_case": str(run_dir / "effective_case.yaml"),
        "report_md": str(run_dir / "report.md"),
        "report_html": str(run_dir / "report.html"),
        "rosbag": str(run_dir / "rosbag"),
    }


def _record_manual_step(metrics: dict[str, Any], name: str, passed: bool, message: str, log_path: Path | None) -> None:
    metrics["steps"].append({
        "name": name,
        "passed": bool(passed),
        "status": "PASS" if passed else "FAIL",
        "message": message,
        "log": str(log_path) if log_path else "",
        "started_at": _utc_now(),
        "finished_at": _utc_now(),
        "returncode": 0 if passed else 1,
    })


def _merged_env(extra: Mapping[str, Any]) -> dict[str, str]:
    env = os.environ.copy()
    source_roots = [
        Path(__file__).resolve().parents[1],
        Path(__file__).resolve().parents[2] / "robot_sim_scenarios",
    ]
    existing = [item for item in env.get("PYTHONPATH", "").split(os.pathsep) if item]
    source_entries = [str(source_root) for source_root in source_roots if source_root.exists()]
    existing = [item for item in existing if item not in source_entries]
    existing = [*source_entries, *existing]
    if existing:
        env["PYTHONPATH"] = os.pathsep.join(existing)
    env.update({str(key): str(value) for key, value in dict(extra or {}).items()})
    return env


def _terminate_process(process: subprocess.Popen, sig: signal.Signals) -> None:
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), sig)
        except ProcessLookupError:
            pass
        time.sleep(0.5)
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


def _write_json(path: Path, value: Any) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, sort_keys=True)
        handle.write("\n")


def _write_yaml(path: Path, value: Any) -> Path:
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(value, handle, sort_keys=False)
    return path


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def _safe_id(value: Any) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_")


def _timeout_value(value: Any) -> float | None:
    timeout = float(value or 0.0)
    return timeout if timeout > 0.0 else None
