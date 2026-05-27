import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction


CORE_TOPICS = [
    "/clock",
    "/tf",
    "/tf_static",
]

CONTROL_TOPICS = [
    "/joint_states",
    "/arm_controller/controller_state",
    "/arm_controller/joint_trajectory",
    "/gripper_controller/controller_state",
    "/gripper_controller/joint_trajectory",
]

CONTROL_ACTION_TOPICS = [
    "/arm_controller/follow_joint_trajectory/_action/feedback",
    "/arm_controller/follow_joint_trajectory/_action/status",
    "/gripper_controller/follow_joint_trajectory/_action/feedback",
    "/gripper_controller/follow_joint_trajectory/_action/status",
]

SENSOR_TOPICS = [
    "/camera/color/image_raw",
    "/camera/color/camera_info",
    "/camera/depth/image_raw",
    "/camera/depth/camera_info",
    "/camera/points",
    "/scan",
    "/lidar/points",
    "/imu/data",
]

DISTRIBUTED_CONTROL_TOPICS = [
    "/joint_states",
    "/robot/joint_states",
    "/robot/arm_controller/controller_state",
    "/robot/arm_controller/joint_trajectory",
    "/robot/gripper_controller/controller_state",
    "/robot/gripper_controller/joint_trajectory",
]

DISTRIBUTED_CONTROL_ACTION_TOPICS = [
    "/robot/arm_controller/follow_joint_trajectory/_action/feedback",
    "/robot/arm_controller/follow_joint_trajectory/_action/status",
    "/robot/gripper_controller/follow_joint_trajectory/_action/feedback",
    "/robot/gripper_controller/follow_joint_trajectory/_action/status",
]

DISTRIBUTED_SENSOR_TOPICS = [
    "/sensors/camera/color/image_raw",
    "/sensors/camera/color/camera_info",
    "/sensors/camera/depth/image_raw",
    "/sensors/camera/depth/camera_info",
    "/sensors/camera/points",
    "/sensors/scan",
    "/sensors/lidar/points",
    "/sensors/imu/data",
]


def _get_config(context, name):
    return context.launch_configurations.get(name, "").strip()


def _bool_value(value, name):
    normalized = value.strip().lower()
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off", ""):
        return False
    raise RuntimeError(f"{name} must be true or false; got '{value}'")


def _split_topics(value):
    if not value:
        return []
    return [item for item in value.replace(",", " ").split() if item]


def _dedupe(topics):
    result = []
    seen = set()
    for topic in topics:
        if topic not in seen:
            seen.add(topic)
            result.append(topic)
    return result


def _topic_group_topics(topic_group, include_action_topics):
    if topic_group == "control":
        topics = [*CORE_TOPICS, *CONTROL_TOPICS]
        if include_action_topics:
            topics.extend(CONTROL_ACTION_TOPICS)
        return topics
    if topic_group == "sensors":
        return [*CORE_TOPICS, *SENSOR_TOPICS]
    if topic_group == "all":
        topics = [*CORE_TOPICS, *CONTROL_TOPICS, *SENSOR_TOPICS]
        if include_action_topics:
            topics.extend(CONTROL_ACTION_TOPICS)
        return topics
    if topic_group == "distributed":
        topics = [
            *CORE_TOPICS,
            *DISTRIBUTED_CONTROL_TOPICS,
            *DISTRIBUTED_SENSOR_TOPICS,
        ]
        if include_action_topics:
            topics.extend(DISTRIBUTED_CONTROL_ACTION_TOPICS)
        return topics
    if topic_group == "custom":
        return []
    raise RuntimeError(
        "topic_group must be control, sensors, all, distributed, or custom; "
        f"got '{topic_group}'"
    )


def _launch_setup(context, *args, **kwargs):
    topic_group = _get_config(context, "topic_group").lower()
    requested_hidden_topics = _bool_value(
        _get_config(context, "include_hidden_topics"), "include_hidden_topics"
    )
    include_action_topics = _bool_value(
        _get_config(context, "include_action_topics"),
        "include_action_topics",
    )
    compression = _bool_value(_get_config(context, "compression"), "compression")

    topics = _topic_group_topics(topic_group, include_action_topics)
    topics.extend(_split_topics(_get_config(context, "extra_topics")))
    topics = _dedupe(topics)
    if not topics:
        raise RuntimeError(
            "No topics selected. Use topic_group:=control|sensors|all|distributed "
            "or pass extra_topics:='/topic_a /topic_b'."
        )

    output_dir = os.path.expanduser(_get_config(context, "output_dir"))
    if not output_dir:
        output_dir = os.path.join(os.environ.get("HOME", "."), "robot_sim_bags")
    os.makedirs(output_dir, exist_ok=True)

    bag_name = _get_config(context, "bag_name")
    if not bag_name or bag_name.lower() == "auto":
        bag_name = f"robot_sim_{topic_group}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    output_path = bag_name if os.path.isabs(bag_name) else os.path.join(output_dir, bag_name)

    cmd = [
        "ros2",
        "bag",
        "record",
        "--storage",
        _get_config(context, "storage_id") or "sqlite3",
        "-o",
        output_path,
    ]

    max_bag_size = _get_config(context, "max_bag_size")
    if max_bag_size and max_bag_size != "0":
        cmd.extend(["--max-bag-size", max_bag_size])

    max_bag_duration = _get_config(context, "max_bag_duration")
    if max_bag_duration and max_bag_duration != "0":
        cmd.extend(["--max-bag-duration", max_bag_duration])

    if compression:
        cmd.extend([
            "--compression-mode",
            _get_config(context, "compression_mode") or "file",
            "--compression-format",
            _get_config(context, "compression_format") or "zstd",
        ])

    action_topics_selected = include_action_topics and topic_group in (
        "control",
        "all",
        "distributed",
    )
    include_hidden_topics = requested_hidden_topics or action_topics_selected
    if include_hidden_topics:
        cmd.append("--include-hidden-topics")

    cmd.extend(topics)

    return [
        LogInfo(msg=f"Recording rosbag2 to: {output_path}"),
        LogInfo(msg=f"Topic group: {topic_group}; topics: {' '.join(topics)}"),
        ExecuteProcess(cmd=cmd, output="screen", emulate_tty=True),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "topic_group",
            default_value="all",
            description="Topic preset: control, sensors, all, distributed, or custom.",
        ),
        DeclareLaunchArgument(
            "extra_topics",
            default_value="",
            description="Additional topics separated by spaces or commas.",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="~/robot_sim_bags",
            description="Directory for relative bag names.",
        ),
        DeclareLaunchArgument(
            "bag_name",
            default_value="auto",
            description="Bag directory name. Use auto for timestamped names.",
        ),
        DeclareLaunchArgument("storage_id", default_value="sqlite3"),
        DeclareLaunchArgument(
            "include_hidden_topics",
            default_value="false",
            description="Pass --include-hidden-topics to ros2 bag record.",
        ),
        DeclareLaunchArgument(
            "include_action_topics",
            default_value="true",
            description="Include FollowJointTrajectory feedback/status topics.",
        ),
        DeclareLaunchArgument(
            "compression",
            default_value="false",
            description="Enable rosbag2 compression.",
        ),
        DeclareLaunchArgument("compression_mode", default_value="file"),
        DeclareLaunchArgument("compression_format", default_value="zstd"),
        DeclareLaunchArgument(
            "max_bag_size",
            default_value="0",
            description="Maximum bag size in bytes. 0 disables splitting by size.",
        ),
        DeclareLaunchArgument(
            "max_bag_duration",
            default_value="0",
            description="Maximum bag duration in seconds. 0 disables splitting by time.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
