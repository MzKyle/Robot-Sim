from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any, Mapping

import yaml

from robot_sim_bringup.legacy_integrations.module_adapter import preflight_adapter, scan3d_source_summary
from robot_sim_bringup.robot_domain.validation_cases import load_validation_case


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a robot_sim external module validation.")
    parser.add_argument("--validation-case", required=True)
    parser.add_argument("--metrics-output", required=True)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--run-dir", default="")
    parser.add_argument("--logs-dir", default="")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        metrics = run_module_validation(args)
        _write_json(Path(args.metrics_output), metrics)
        return 0 if metrics.get("passed") else 1
    except Exception as exc:
        metrics = {
            "task_type": "module_validation",
            "passed": False,
            "module_failures": [str(exc)],
            "error": str(exc),
        }
        _write_json(Path(args.metrics_output), metrics)
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


def run_module_validation(args) -> dict[str, Any]:
    case = load_validation_case(args.validation_case)
    module = case.get("module", {})
    adapters = case.get("adapters", [])
    expect_module = case.get("expect", {}).get("module", {})
    run_dir = Path(args.run_dir or Path(args.metrics_output).parent).expanduser().resolve()
    logs_dir = Path(args.logs_dir or run_dir / "logs").expanduser().resolve()
    logs_dir.mkdir(parents=True, exist_ok=True)
    adapter_dir = run_dir / "module_adapters"
    adapter_dir.mkdir(parents=True, exist_ok=True)

    metrics: dict[str, Any] = {
        "case_name": case["name"],
        "task_type": "module_validation",
        "profile": case["profile"],
        "scene": case["scene"].name,
        "passed": False,
        "adapter_health": [],
        "module_services": [],
        "module_topics": [],
        "module_events": [],
        "module_failures": [],
        "adapter_data_sources": [],
        "business_actions": _business_actions(case),
    }
    processes: list[tuple[str, subprocess.Popen]] = []

    try:
        for adapter in adapters:
            adapter_config = dict(adapter)
            preflight_adapter(adapter_config)
            adapter_file = adapter_dir / f"{_safe_id(adapter_config.get('name') or adapter_config.get('type'))}.yaml"
            _write_yaml(adapter_file, adapter_config)
            log_path = logs_dir / f"adapter_{_safe_id(adapter_config.get('name') or adapter_config.get('type'))}.log"
            source_summary = _adapter_source_summary(adapter_config)
            if source_summary:
                metrics["adapter_data_sources"].append(source_summary)
            command = [
                sys.executable,
                "-m",
                "robot_sim_bringup.legacy_integrations.module_adapter",
                "--adapter-file",
                str(adapter_file),
                "--case-file",
                str(args.validation_case),
            ]
            processes.append((f"adapter:{adapter_config.get('name') or adapter_config.get('type')}", _popen(command, log_path)))
            metrics["adapter_health"].append({
                "name": str(adapter_config.get("name") or adapter_config.get("type")),
                "type": str(adapter_config.get("type", "")),
                "log": str(log_path),
                "status": "STARTED",
                "source": source_summary,
            })

        _sleep_and_check(processes, metrics, 2.0)

        for module_process in _module_process_specs(module):
            log_path = logs_dir / f"module_{_safe_id(module_process['name'])}.log"
            processes.append((f"module:{module_process['name']}", _popen(module_process["command"], log_path, module_process.get("env"))))
            metrics["module_events"].append({
                "name": module_process["name"],
                "type": "process_start",
                "command": module_process["command"],
                "log": str(log_path),
                "ok": True,
            })

        _sleep_and_check(processes, metrics, float(module.get("startup_delay_sec", 4.0)))
        _wait_services(module.get("wait_services", []), args.timeout, metrics)

        for action in module.get("actions", []):
            _run_action(action, args.timeout, metrics)

        _observe_topics(expect_module.get("topics", []), args.timeout, metrics)
        _check_required_services(expect_module.get("services", []), args.timeout, metrics)
        _apply_expectations(expect_module, metrics)
        metrics["passed"] = not metrics["module_failures"]
        return metrics
    except Exception as exc:
        message = str(exc)
        if message not in metrics["module_failures"]:
            metrics["module_failures"].append(message)
        metrics["error"] = message
        metrics["passed"] = False
        return metrics
    finally:
        for _name, process in reversed(processes):
            _terminate_process(process)


def _module_process_specs(module: Mapping[str, Any]) -> list[dict[str, Any]]:
    specs = []
    launch = module.get("launch")
    if isinstance(launch, Mapping) and bool(launch.get("enabled", True)):
        command = [
            "ros2",
            "launch",
            str(launch["package"]),
            str(launch["file"]),
        ]
        for key, value in _mapping(launch.get("arguments", {})).items():
            command.append(f"{key}:={value}")
        specs.append({
            "name": str(launch.get("name") or f"{launch['package']}_{launch['file']}"),
            "command": command,
            "env": _merged_env(launch.get("env", {})),
        })
    for item in module.get("commands", []) or []:
        if not isinstance(item, Mapping) or not bool(item.get("enabled", True)):
            continue
        specs.append({
            "name": str(item.get("name") or "module_command"),
            "command": [str(part) for part in item.get("command", [])],
            "env": _merged_env(item.get("env", {})),
        })
    return specs


def _adapter_source_summary(adapter_config: Mapping[str, Any]) -> dict[str, Any]:
    if str(adapter_config.get("type", "")) != "scan3d_service":
        return {}
    return scan3d_source_summary(adapter_config)


def _run_action(action: Mapping[str, Any], default_timeout: float, metrics: dict[str, Any]) -> None:
    action_type = str(action.get("type", "service_call"))
    name = str(action.get("name") or action.get("service") or action_type)
    if action_type != "service_call":
        metrics["module_failures"].append(f"unsupported module action type: {action_type}")
        return
    started = time.monotonic()
    try:
        result = _call_service(
            str(action["service"]),
            str(action["service_type"]),
            _mapping(action.get("request", {})),
            float(action.get("timeout_sec", default_timeout)),
        )
    except Exception as exc:
        result = {
            "response": None,
            "ok": False,
            "error": str(exc),
        }
    result.update({
        "name": name,
        "service": str(action["service"]),
        "service_type": str(action["service_type"]),
        "duration_sec": time.monotonic() - started,
    })
    expect = _mapping(action.get("expect", {}))
    success_field = str(expect.get("success_field", ""))
    if result.get("error"):
        result["ok"] = False
    elif success_field:
        result["ok"] = bool(_nested_value(result.get("response", {}), success_field))
    else:
        result["ok"] = True
    metrics["module_services"].append(result)
    if not result["ok"]:
        metrics["module_failures"].append(f"module action failed: {name}")


def _call_service(service_name: str, service_type: str, request_fields: Mapping[str, Any], timeout: float) -> dict[str, Any]:
    import rclpy
    from rosidl_runtime_py.utilities import get_service

    Service = get_service(service_type)
    rclpy.init(args=None)
    node = rclpy.create_node(f"robot_sim_module_service_client_{_safe_id(service_name)}")
    try:
        client = node.create_client(Service, service_name)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service not available: {service_name}")
        request = Service.Request()
        _assign_fields(request, request_fields)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done():
            raise RuntimeError(f"service call timed out: {service_name}")
        response = future.result()
        return {"response": _message_to_dict(response)}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _observe_topics(topics: list[Any], default_timeout: float, metrics: dict[str, Any]) -> None:
    for topic in topics or []:
        if not isinstance(topic, Mapping):
            continue
        try:
            result = _collect_topic(
                str(topic["name"]),
                str(topic["type"]),
                float(topic.get("timeout_sec", default_timeout)),
                int(topic.get("min_count", 1)),
                _mapping(topic.get("expect", {})),
            )
        except Exception as exc:
            result = {
                "name": str(topic.get("name", "")),
                "type": str(topic.get("type", "")),
                "count": 0,
                "min_count": int(topic.get("min_count", 1)),
                "last": None,
                "expectation_failures": [str(exc)],
                "ok": False,
            }
        metrics["module_topics"].append(result)
        if not metrics["module_topics"][-1]["ok"]:
            metrics["module_failures"].append(f"module topic failed: {topic['name']}")


def _collect_topic(topic_name: str, type_name: str, timeout: float, min_count: int, expect: Mapping[str, Any]) -> dict[str, Any]:
    import rclpy
    from rosidl_runtime_py.utilities import get_message

    Message = get_message(type_name)
    rclpy.init(args=None)
    node = rclpy.create_node(f"robot_sim_module_topic_probe_{_safe_id(topic_name)}")
    messages: list[Any] = []
    node.create_subscription(Message, topic_name, lambda msg: messages.append(msg), 10)
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline and len(messages) < min_count:
        rclpy.spin_once(node, timeout_sec=0.1)
    last = _message_to_dict(messages[-1]) if messages else None
    ok = len(messages) >= min_count
    expectation_failures = []
    for field, expected in expect.items():
        actual = _nested_value(last or {}, str(field))
        field_ok, reason = _matches_expectation(actual, expected)
        if not field_ok:
            ok = False
            expectation_failures.append(f"{field}: {reason}")
    node.destroy_node()
    rclpy.shutdown()
    return {
        "name": topic_name,
        "type": type_name,
        "count": len(messages),
        "min_count": min_count,
        "last": last,
        "expectation_failures": expectation_failures,
        "ok": ok,
    }


def _wait_services(services: list[Any], timeout: float, metrics: dict[str, Any]) -> None:
    for service in services or []:
        if not isinstance(service, Mapping):
            continue
        error = ""
        try:
            result = _wait_service(str(service["name"]), str(service["type"]), float(service.get("timeout_sec", timeout)))
        except Exception as exc:
            result = False
            error = str(exc)
        metrics["module_events"].append({
            "name": str(service["name"]),
            "type": "wait_service",
            "ok": result,
            "error": error,
        })
        if not result:
            metrics["module_failures"].append(f"service unavailable: {service['name']}")


def _check_required_services(services: list[Any], timeout: float, metrics: dict[str, Any]) -> None:
    for service in services or []:
        if not isinstance(service, Mapping):
            continue
        error = ""
        try:
            available = _wait_service(str(service["name"]), str(service["type"]), float(service.get("timeout_sec", timeout)))
        except Exception as exc:
            available = False
            error = str(exc)
        metrics["module_services"].append({
            "name": str(service["name"]),
            "service": str(service["name"]),
            "service_type": str(service["type"]),
            "available": available,
            "error": error,
            "ok": available,
        })
        if not available:
            metrics["module_failures"].append(f"required service unavailable: {service['name']}")


def _wait_service(service_name: str, service_type: str, timeout: float) -> bool:
    import rclpy
    from rosidl_runtime_py.utilities import get_service

    Service = get_service(service_type)
    rclpy.init(args=None)
    node = rclpy.create_node(f"robot_sim_module_wait_{_safe_id(service_name)}")
    try:
        client = node.create_client(Service, service_name)
        return bool(client.wait_for_service(timeout_sec=timeout))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _apply_expectations(expect_module: Mapping[str, Any], metrics: dict[str, Any]) -> None:
    required_actions = [str(name) for name in expect_module.get("required_actions", [])]
    service_results = {item.get("name"): item for item in metrics.get("module_services", [])}
    for name in required_actions:
        if not service_results.get(name, {}).get("ok", False):
            metrics["module_failures"].append(f"required module action did not pass: {name}")


def _business_actions(case: Mapping[str, Any]) -> list[dict[str, Any]]:
    module = case.get("module", {})
    actions = []
    for action in module.get("actions", []) or []:
        if isinstance(action, Mapping):
            actions.append({
                "name": str(action.get("name") or action.get("service", "")),
                "type": str(action.get("type", "service_call")),
                "service": str(action.get("service", "")),
            })
    return actions


def _popen(command: list[str], log_path: Path, env: Mapping[str, str] | None = None) -> subprocess.Popen:
    if not command:
        raise RuntimeError("module process command is empty")
    handle = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        stdout=handle,
        stderr=subprocess.STDOUT,
        text=True,
        env=dict(env) if env else None,
        preexec_fn=os.setsid,
    )
    process._robot_sim_log_handle = handle
    return process


def _sleep_and_check(processes: list[tuple[str, subprocess.Popen]], metrics: dict[str, Any], delay: float) -> None:
    time.sleep(max(0.0, delay))
    for name, process in processes:
        if process.poll() is not None:
            metrics["module_failures"].append(f"process exited early: {name}")
            raise RuntimeError(f"process exited early: {name}")


def _terminate_process(process: subprocess.Popen) -> None:
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
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


def _assign_fields(message, values: Mapping[str, Any]) -> None:
    for key, value in values.items():
        if not hasattr(message, key):
            raise RuntimeError(f"request field does not exist: {key}")
        current = getattr(message, key)
        if isinstance(value, Mapping) and hasattr(current, "get_fields_and_field_types"):
            _assign_fields(current, value)
        else:
            setattr(message, key, value)


def _message_to_dict(value: Any, depth: int = 0):
    if depth > 5:
        return str(value)
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, bytes):
        return f"<bytes:{len(value)}>"
    if isinstance(value, (list, tuple)):
        if len(value) > 24:
            return [_message_to_dict(item, depth + 1) for item in value[:24]] + [f"<truncated:{len(value) - 24}>"]
        return [_message_to_dict(item, depth + 1) for item in value]
    if hasattr(value, "get_fields_and_field_types"):
        return {
            name: _message_to_dict(getattr(value, name), depth + 1)
            for name in value.get_fields_and_field_types()
        }
    return str(value)


def _nested_value(value: Any, path: str) -> Any:
    current = value
    for part in path.split("."):
        if isinstance(current, Mapping):
            current = current.get(part)
        else:
            return None
    return current


def _matches_expectation(actual: Any, expected: Any) -> tuple[bool, str]:
    if not isinstance(expected, Mapping):
        return actual == expected, f"expected {expected!r}, got {actual!r}"

    if bool(expected.get("exists", False)):
        return actual is not None, f"expected value to exist, got {actual!r}"
    if "equals" in expected:
        value = expected["equals"]
        return actual == value, f"expected {value!r}, got {actual!r}"
    if "contains" in expected:
        value = str(expected["contains"])
        actual_text = str(actual)
        return value in actual_text, f"expected {actual_text!r} to contain {value!r}"

    try:
        actual_number = float(actual)
    except (TypeError, ValueError):
        return False, f"expected numeric value, got {actual!r}"
    if "min" in expected and actual_number < float(expected["min"]):
        return False, f"expected >= {expected['min']}, got {actual_number}"
    if "max" in expected and actual_number > float(expected["max"]):
        return False, f"expected <= {expected['max']}, got {actual_number}"
    if "abs_max" in expected and abs(actual_number) > float(expected["abs_max"]):
        return False, f"expected abs <= {expected['abs_max']}, got {actual_number}"
    return True, "ok"


def _merged_env(env: Any) -> dict[str, str]:
    result = dict(os.environ)
    for key, value in _mapping(env).items():
        result[str(key)] = str(value)
    return result


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, sort_keys=True)
        handle.write("\n")


def _write_yaml(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(value, handle, sort_keys=False)


def _safe_id(value: Any) -> str:
    import re

    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_") or "module"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


if __name__ == "__main__":
    raise SystemExit(main())
