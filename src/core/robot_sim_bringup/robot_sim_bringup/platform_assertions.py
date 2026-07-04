from __future__ import annotations

import math
import time
from typing import Any, Mapping


def evaluate_assertions(assertions: list[Mapping[str, Any]], context: Mapping[str, Any]) -> list[dict[str, Any]]:
    results = []
    for assertion in assertions or []:
        assertion_type = str(assertion.get("type", ""))
        try:
            if assertion_type == "action_result":
                result = _assert_action_result(assertion, context)
            elif assertion_type == "process_exit":
                result = _assert_process_exit(assertion, context)
            elif assertion_type == "topic":
                result = _assert_topic(assertion)
            elif assertion_type == "service":
                result = _assert_service(assertion)
            elif assertion_type == "tf":
                result = _assert_tf(assertion)
            elif assertion_type == "state_sequence":
                result = _assert_state_sequence(assertion)
            elif assertion_type == "ground_truth_error":
                result = _assert_ground_truth_error(assertion, context)
            else:
                result = _result(assertion, False, f"unsupported assertion type: {assertion_type}")
        except Exception as exc:
            result = _result(assertion, False, str(exc))
        results.append(result)
    return results


def matches_expectation(actual: Any, expected: Any) -> tuple[bool, str]:
    if not isinstance(expected, Mapping):
        return (actual == expected, f"expected {expected!r}, got {actual!r}")
    if expected.get("exists") is True and actual is None:
        return False, "field is missing"
    if "equals" in expected and actual != expected["equals"]:
        return False, f"expected {expected['equals']!r}, got {actual!r}"
    if "contains" in expected and str(expected["contains"]) not in str(actual):
        return False, f"expected {actual!r} to contain {expected['contains']!r}"
    if "min" in expected and not (actual is not None and float(actual) >= float(expected["min"])):
        return False, f"expected >= {expected['min']}, got {actual!r}"
    if "max" in expected and not (actual is not None and float(actual) <= float(expected["max"])):
        return False, f"expected <= {expected['max']}, got {actual!r}"
    if "abs_max" in expected and not (actual is not None and abs(float(actual)) <= float(expected["abs_max"])):
        return False, f"expected abs <= {expected['abs_max']}, got {actual!r}"
    if "len_min" in expected and not (actual is not None and len(actual) >= int(expected["len_min"])):
        return False, f"expected length >= {expected['len_min']}, got {len(actual) if actual is not None else None}"
    if "len_max" in expected and not (actual is not None and len(actual) <= int(expected["len_max"])):
        return False, f"expected length <= {expected['len_max']}, got {len(actual) if actual is not None else None}"
    return True, "ok"


def nested_value(value: Any, path: str) -> Any:
    current = value
    for part in path.split("."):
        if isinstance(current, Mapping):
            current = current.get(part)
        elif isinstance(current, list) and part.isdigit():
            index = int(part)
            current = current[index] if index < len(current) else None
        else:
            current = getattr(current, part, None)
        if current is None:
            return None
    return current


def message_to_dict(message: Any) -> Any:
    if isinstance(message, (str, int, float, bool)) or message is None:
        return message
    if isinstance(message, (list, tuple)):
        return [message_to_dict(item) for item in message]
    if hasattr(message, "get_fields_and_field_types"):
        return {
            field: message_to_dict(getattr(message, field))
            for field in message.get_fields_and_field_types()
        }
    if hasattr(message, "__dict__"):
        return {
            key.lstrip("_"): message_to_dict(value)
            for key, value in vars(message).items()
            if not key.startswith("_abc")
        }
    return str(message)


def assign_fields(message: Any, fields: Mapping[str, Any]) -> None:
    for field, value in fields.items():
        if "." in str(field):
            head, tail = str(field).split(".", 1)
            assign_fields(getattr(message, head), {tail: value})
            continue
        current = getattr(message, str(field), None)
        if hasattr(current, "get_fields_and_field_types") and isinstance(value, Mapping):
            assign_fields(current, value)
        else:
            setattr(message, str(field), value)


def _assert_action_result(assertion: Mapping[str, Any], context: Mapping[str, Any]) -> dict[str, Any]:
    action_name = str(assertion.get("action", assertion.get("target", "")))
    actions = context.get("actions", {})
    if action_name not in actions:
        return _result(assertion, False, f"unknown action result: {action_name}")
    failures = _expectation_failures(actions[action_name], assertion.get("expect", {}))
    return _result(assertion, not failures, "; ".join(failures) or "ok", {"actual": actions[action_name]})


def _assert_process_exit(assertion: Mapping[str, Any], context: Mapping[str, Any]) -> dict[str, Any]:
    process_name = str(assertion.get("process", assertion.get("target", "")))
    processes = context.get("processes", {})
    if process_name not in processes:
        return _result(assertion, False, f"unknown process: {process_name}")
    failures = _expectation_failures(processes[process_name], assertion.get("expect", {"returncode": {"equals": 0}}))
    return _result(assertion, not failures, "; ".join(failures) or "ok", {"actual": processes[process_name]})


def _assert_topic(assertion: Mapping[str, Any]) -> dict[str, Any]:
    import rclpy
    from rosidl_runtime_py.utilities import get_message

    topic_name = str(assertion["topic"])
    Message = get_message(str(assertion["message_type"]))
    timeout = float(assertion.get("timeout_sec", 5.0))
    min_count = int(assertion.get("min_count", 1))
    messages: list[Any] = []
    stamps: list[float] = []
    rclpy.init(args=None)
    node = rclpy.create_node("robot_sim_assert_topic")
    try:
        node.create_subscription(Message, topic_name, lambda msg: (messages.append(msg), stamps.append(time.monotonic())), 10)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline and len(messages) < min_count:
            rclpy.spin_once(node, timeout_sec=0.1)
        actual = message_to_dict(messages[-1]) if messages else None
        failures = []
        if len(messages) < min_count:
            failures.append(f"expected at least {min_count} messages, got {len(messages)}")
        min_hz = assertion.get("min_hz")
        hz = _hz(stamps)
        if min_hz is not None and hz < float(min_hz):
            failures.append(f"expected hz >= {min_hz}, got {hz:.3f}")
        failures.extend(_expectation_failures(actual or {}, assertion.get("expect", {})))
        return _result(assertion, not failures, "; ".join(failures) or "ok", {
            "count": len(messages),
            "hz": hz,
            "last": actual,
        })
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _assert_service(assertion: Mapping[str, Any]) -> dict[str, Any]:
    import rclpy
    from rosidl_runtime_py.utilities import get_service

    service_name = str(assertion["service"])
    Service = get_service(str(assertion["service_type"]))
    timeout = float(assertion.get("timeout_sec", 5.0))
    rclpy.init(args=None)
    node = rclpy.create_node("robot_sim_assert_service")
    try:
        client = node.create_client(Service, service_name)
        if not client.wait_for_service(timeout_sec=timeout):
            return _result(assertion, False, f"service unavailable: {service_name}")
        if not assertion.get("call", False):
            return _result(assertion, True, "available")
        request = Service.Request()
        assign_fields(request, assertion.get("request", {}))
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done():
            return _result(assertion, False, f"service call timed out: {service_name}")
        actual = message_to_dict(future.result())
        failures = _expectation_failures(actual, assertion.get("expect", {}))
        return _result(assertion, not failures, "; ".join(failures) or "ok", {"response": actual})
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _assert_tf(assertion: Mapping[str, Any]) -> dict[str, Any]:
    import rclpy
    from tf2_ros import Buffer, TransformListener

    parent = str(assertion["parent_frame"])
    child = str(assertion["child_frame"])
    timeout = float(assertion.get("timeout_sec", 5.0))
    rclpy.init(args=None)
    node = rclpy.create_node("robot_sim_assert_tf")
    buffer = Buffer()
    TransformListener(buffer, node)
    try:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                transform = buffer.lookup_transform(parent, child, rclpy.time.Time())
                return _result(assertion, True, "ok", {"transform": message_to_dict(transform)})
            except Exception:
                pass
        return _result(assertion, False, f"missing transform {parent}->{child}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _assert_state_sequence(assertion: Mapping[str, Any]) -> dict[str, Any]:
    topic_assertion = dict(assertion)
    topic_assertion["type"] = "topic"
    topic_assertion.setdefault("min_count", len(assertion.get("sequence", [])))
    topic_result = _assert_topic(topic_assertion)
    return topic_result


def _assert_ground_truth_error(assertion: Mapping[str, Any], context: Mapping[str, Any]) -> dict[str, Any]:
    actual = float(nested_value(context, str(assertion.get("actual", ""))) or math.nan)
    expected = float(nested_value(context, str(assertion.get("expected", ""))) or math.nan)
    max_error = float(assertion.get("max_error", 0.0))
    error = abs(actual - expected)
    return _result(assertion, error <= max_error, f"error={error}", {"error": error})


def _expectation_failures(actual: Any, expect: Mapping[str, Any]) -> list[str]:
    failures = []
    for field, expected in (expect or {}).items():
        value = nested_value(actual, str(field))
        ok, reason = matches_expectation(value, expected)
        if not ok:
            failures.append(f"{field}: {reason}")
    return failures


def _hz(stamps: list[float]) -> float:
    if len(stamps) < 2:
        return 0.0
    duration = stamps[-1] - stamps[0]
    if duration <= 0:
        return 0.0
    return (len(stamps) - 1) / duration


def _result(assertion: Mapping[str, Any], ok: bool, message: str, extra: Mapping[str, Any] | None = None) -> dict[str, Any]:
    result = {
        "name": str(assertion.get("name", assertion.get("type", ""))),
        "type": str(assertion.get("type", "")),
        "ok": bool(ok),
        "message": message,
    }
    if extra:
        result.update(dict(extra))
    return result
