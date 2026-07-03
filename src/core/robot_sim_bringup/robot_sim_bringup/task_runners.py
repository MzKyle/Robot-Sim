from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Mapping


class TaskRunner:
    task_type = ""

    def business_actions(self, case: Mapping[str, Any]) -> list[dict[str, Any]]:
        regions = case.get("task_regions", [])
        return [
            {
                "name": f"move_to_{region}",
                "region": region,
                "type": "moveit_pose_goal",
            }
            for region in regions
        ]

    def validation_command(
        self,
        helper_args: list[str],
        case_file: str,
        metrics_output: Path,
        timeout: float,
    ) -> list[str]:
        return [
            sys.executable,
            "-m",
            "robot_sim_bringup.sim_smoke_helper",
            "validate-case",
            *helper_args,
            "--validation-case",
            case_file,
            "--metrics-output",
            str(metrics_output),
            "--timeout",
            str(timeout),
        ]


class EmptyMotionRunner(TaskRunner):
    task_type = "empty_motion"


class ObstacleClearanceRunner(TaskRunner):
    task_type = "obstacle_clearance"

    def business_actions(self, case):
        return [
            {"name": "apply_collision_objects", "type": "planning_scene"},
            *super().business_actions(case),
            {"name": "measure_tcp_clearance", "type": "metric"},
        ]


class FixtureToPalletRunner(TaskRunner):
    task_type = "fixture_to_pallet"

    def business_actions(self, case):
        task = case.get("task", {})
        obj = task.get("object", {})
        return [
            {"name": "approach_fixture", "region": case["start_region"], "type": "moveit_pose_goal"},
            {"name": "logical_attach", "object": obj.get("model", ""), "type": "moveit_attach"},
            {"name": "transfer_to_pallet", "region": case["goal_region"], "type": "moveit_pose_goal"},
            {"name": "logical_detach", "object": obj.get("model", ""), "type": "moveit_detach"},
            {"name": "verify_target_pose", "region": case["goal_region"], "type": "gazebo_pose_check"},
        ]


class PickPlaceRunner(TaskRunner):
    task_type = "pick_place"

    def business_actions(self, case):
        task = case.get("task", {})
        return [
            {"name": "open_gripper", "controller": task.get("gripper", {}).get("controller", ""), "type": "gripper"},
            {"name": "move_to_pick", "region": task.get("pick_region", ""), "type": "moveit_pose_goal"},
            {"name": "close_gripper", "controller": task.get("gripper", {}).get("controller", ""), "type": "gripper"},
            {"name": "attach_object", "object": task.get("object", {}).get("model", ""), "type": "moveit_attach"},
            {"name": "move_to_place", "region": task.get("place_region", ""), "type": "moveit_pose_goal"},
            {"name": "detach_object", "object": task.get("object", {}).get("model", ""), "type": "moveit_detach"},
            {"name": "open_gripper", "controller": task.get("gripper", {}).get("controller", ""), "type": "gripper"},
        ]


class SensorCalibrationRunner(TaskRunner):
    task_type = "sensor_calibration"

    def business_actions(self, case):
        return [
            {"name": f"capture_view_{index + 1}", "region": region, "type": "sensor_sample"}
            for index, region in enumerate(case.get("task", {}).get("calibration_regions", []))
        ] + [
            {"name": "compute_tf_camera_residual", "type": "metric"},
        ]


class ConveyorSortingRunner(TaskRunner):
    task_type = "conveyor_sorting"

    def business_actions(self, case):
        task = case.get("task", {})
        return [
            {"name": "start_conveyor", "topic": task.get("conveyor", {}).get("command_topic", ""), "type": "ros_topic"},
            {"name": "track_pick_window", "region": case["start_region"], "type": "sensor_sample"},
            {"name": "pick_parcel", "region": case["start_region"], "type": "moveit_pose_goal"},
            {"name": "sort_to_bin", "region": case["goal_region"], "type": "moveit_pose_goal"},
            {"name": "verify_bin_pose", "targets": task.get("sort_targets", {}), "type": "gazebo_pose_check"},
        ]


class ModuleValidationRunner(TaskRunner):
    task_type = "module_validation"

    def business_actions(self, case):
        actions = []
        module = case.get("module", {})
        for action in module.get("actions", []) or []:
            if not isinstance(action, Mapping):
                continue
            actions.append({
                "name": str(action.get("name") or action.get("service", "")),
                "type": str(action.get("type", "service_call")),
                "service": str(action.get("service", "")),
            })
        return actions

    def validation_command(
        self,
        helper_args: list[str],
        case_file: str,
        metrics_output: Path,
        timeout: float,
    ) -> list[str]:
        del helper_args
        return [
            sys.executable,
            "-m",
            "robot_sim_bringup.module_runner",
            "--validation-case",
            case_file,
            "--metrics-output",
            str(metrics_output),
            "--timeout",
            str(timeout),
            "--run-dir",
            str(metrics_output.parent),
            "--logs-dir",
            str(metrics_output.parent / "logs"),
        ]


TASK_RUNNERS = {
    runner.task_type: runner
    for runner in (
        EmptyMotionRunner(),
        ObstacleClearanceRunner(),
        FixtureToPalletRunner(),
        PickPlaceRunner(),
        SensorCalibrationRunner(),
        ConveyorSortingRunner(),
        ModuleValidationRunner(),
    )
}


def get_task_runner(task_type: str) -> TaskRunner:
    try:
        return TASK_RUNNERS[task_type]
    except KeyError as exc:
        valid = ", ".join(sorted(TASK_RUNNERS))
        raise RuntimeError(f"unknown task runner '{task_type}'. Valid task types: {valid}") from exc
