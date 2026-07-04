from __future__ import annotations

import argparse
from datetime import datetime, timezone
import html
import json
from pathlib import Path
import re
from types import SimpleNamespace
from typing import Any
import xml.etree.ElementTree as ET

from robot_sim_bringup.platform_config import expand_suite_cases, load_validation_suite
from robot_sim_bringup.platform_runner import is_platform_case, run_platform_case
from robot_sim_bringup.registry import resolve_validation_case_path
from robot_sim_bringup.run_case import CommandRunner, run_case


SUCCESS = 0
FAILURE = 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a robot_sim validation suite.")
    parser.add_argument("--suite", required=True, help="Validation suite name or YAML path.")
    parser.add_argument("--suite-package", default="", help="ROS package containing robot_sim/validation_suites/<suite>.yaml.")
    parser.add_argument("--output-dir", default="robot_sim_runs", help="Parent directory for suite artifacts.")
    parser.add_argument("--timeout", type=float, default=None, help="Per-case timeout override.")
    parser.add_argument("--rosbag-duration", type=float, default=8.0)
    parser.add_argument("--no-rosbag", action="store_true")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return run_suite(args, CommandRunner())


def run_suite(args: Any, runner: CommandRunner) -> int:
    suite = load_validation_suite(args.suite, suite_package=getattr(args, "suite_package", ""))
    suite_dir = _create_suite_dir(getattr(args, "output_dir", "robot_sim_runs"), suite["name"])
    cases_dir = suite_dir / "cases"
    cases_dir.mkdir(parents=True, exist_ok=True)
    started_at = _utc_now()
    case_results = []
    continue_on_failure = bool(suite.get("execution", {}).get("continue_on_failure", True))

    for case_spec in expand_suite_cases(suite):
        case_name = case_spec["case"]
        case_package = case_spec.get("case_package", "")
        before = set(cases_dir.glob("*"))
        case_args = _case_args(args, case_name, case_package, cases_dir)
        case_path = resolve_validation_case_path(case_name, case_package=case_package)
        if is_platform_case(case_path):
            exit_code = run_platform_case(
                case_args,
                runner,
                parameter_overrides=case_spec.get("parameters", {}),
                run_name_suffix=case_spec.get("id_suffix", ""),
            )
        else:
            exit_code = run_case(case_args, runner)
        after = set(cases_dir.glob("*"))
        new_dirs = sorted(after - before, key=lambda path: path.stat().st_mtime)
        run_dir = str(new_dirs[-1]) if new_dirs else ""
        case_results.append({
            "case": case_name,
            "case_package": case_package,
            "parameters": case_spec.get("parameters", {}),
            "run_dir": run_dir,
            "exit_code": exit_code,
            "passed": exit_code == SUCCESS,
        })
        if exit_code != SUCCESS and not continue_on_failure:
            break

    passed = all(item["passed"] for item in case_results)
    metrics = {
        "schema": 1,
        "suite_name": suite["name"],
        "suite_path": suite["path"],
        "passed": passed,
        "started_at": started_at,
        "finished_at": _utc_now(),
        "case_count": len(case_results),
        "failed_count": len([item for item in case_results if not item["passed"]]),
        "cases": case_results,
    }
    _write_json(suite_dir / "suite_metrics.json", metrics)
    report_md = _render_suite_markdown(metrics)
    (suite_dir / "suite_report.md").write_text(report_md, encoding="utf-8")
    (suite_dir / "suite_report.html").write_text(_render_html_report(report_md), encoding="utf-8")
    _write_junit(suite_dir / "junit.xml", metrics)
    print(f"Suite artifacts: {suite_dir}")
    return SUCCESS if passed else FAILURE


def _case_args(args: Any, case_name: str, case_package: str, output_dir: Path) -> SimpleNamespace:
    return SimpleNamespace(
        case=case_name,
        case_package=case_package,
        output_dir=str(output_dir),
        profile="",
        profile_file="",
        profile_package="",
        scene="",
        scene_package="",
        scene_variant="",
        scene_param=[],
        mode=None,
        sensor_overrides=None,
        timeout=getattr(args, "timeout", None),
        rosbag_duration=getattr(args, "rosbag_duration", 8.0),
        no_rosbag=bool(getattr(args, "no_rosbag", False)),
        keep_sim=False,
    )


def _render_suite_markdown(metrics: dict[str, Any]) -> str:
    status = "PASS" if metrics.get("passed") else "FAIL"
    lines = [
        f"# robot_sim Suite Report: {metrics['suite_name']}",
        "",
        f"- Status: **{status}**",
        f"- Cases: `{metrics['case_count']}`",
        f"- Failed: `{metrics['failed_count']}`",
        f"- Started: `{metrics['started_at']}`",
        f"- Finished: `{metrics['finished_at']}`",
        "",
        "| Case | Status | Parameters | Run Dir |",
        "| --- | --- | --- | --- |",
    ]
    for item in metrics.get("cases", []):
        case_status = "PASS" if item.get("passed") else "FAIL"
        lines.append(
            f"| {item.get('case', '')} | {case_status} | `{item.get('parameters', {})}` | `{item.get('run_dir', '')}` |"
        )
    lines.append("")
    return "\n".join(lines)


def _write_junit(path: Path, metrics: dict[str, Any]) -> None:
    suite = ET.Element(
        "testsuite",
        {
            "name": str(metrics["suite_name"]),
            "tests": str(metrics["case_count"]),
            "failures": str(metrics["failed_count"]),
        },
    )
    for item in metrics.get("cases", []):
        case = ET.SubElement(suite, "testcase", {"name": str(item.get("case", ""))})
        if not item.get("passed"):
            failure = ET.SubElement(case, "failure", {"message": "validation case failed"})
            failure.text = str(item.get("run_dir", ""))
    tree = ET.ElementTree(suite)
    tree.write(path, encoding="utf-8", xml_declaration=True)


def _render_html_report(markdown_text: str) -> str:
    escaped = html.escape(markdown_text)
    return (
        "<!doctype html>\n<html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
        "<title>robot_sim suite report</title>"
        "<style>body{font-family:system-ui,sans-serif;max-width:980px;margin:32px auto;line-height:1.55;}"
        "pre{white-space:pre-wrap;background:#f6f8fa;padding:16px;border-radius:6px;}</style>"
        "</head><body><pre>"
        + escaped
        + "</pre></body></html>\n"
    )


def _create_suite_dir(output_dir: str, suite_name: str) -> Path:
    parent = Path(output_dir).expanduser().resolve()
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    base = parent / f"{timestamp}_suite_{_safe_id(suite_name)}"
    candidate = base
    index = 1
    while candidate.exists():
        index += 1
        candidate = Path(f"{base}_{index}")
    candidate.mkdir(parents=True)
    return candidate


def _write_json(path: Path, value: Any) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, sort_keys=True)
        handle.write("\n")


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def _safe_id(value: Any) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_")


if __name__ == "__main__":
    raise SystemExit(main())
