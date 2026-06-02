import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def _default_path(package_name, *parts):
    return os.path.join(get_package_share_directory(package_name), *parts)


def _config(context, name):
    return context.launch_configurations.get(name, "").strip()


def _load_yaml(path):
    _require_file(path)
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _load_text(path):
    _require_file(path)
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def _require_file(path):
    if not os.path.exists(path):
        raise RuntimeError(f"MoveIt config file does not exist: {path}")


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")
    monitored_planning_scene_topic = LaunchConfiguration("monitored_planning_scene_topic")
    camera_points_topic = LaunchConfiguration("camera_points_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    robot_xacro = _config(context, "robot_xacro")
    srdf_file = _config(context, "srdf_file")
    kinematics_yaml = _config(context, "kinematics_yaml")
    joint_limits_yaml = _config(context, "joint_limits_yaml")
    moveit_controllers_yaml = _config(context, "moveit_controllers_yaml")
    ompl_planning_yaml = _config(context, "ompl_planning_yaml")
    rviz_config = _config(context, "rviz_config")

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", robot_xacro]),
            value_type=str,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": _load_text(srdf_file)
    }
    robot_description_kinematics = {
        "robot_description_kinematics": _load_yaml(kinematics_yaml)
    }

    ompl_config = _load_yaml(ompl_planning_yaml)

    moveit_parameters = [
        {"use_sim_time": use_sim_time},
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        _load_yaml(joint_limits_yaml),
        {
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
            "ompl": ompl_config,
        },
        _load_yaml(moveit_controllers_yaml),
        {
            "allow_trajectory_execution": True,
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        },
    ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=moveit_parameters,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=moveit_parameters,
        remappings=[
            ("/monitored_planning_scene", monitored_planning_scene_topic),
            ("/camera/points", camera_points_topic),
            ("/scan", scan_topic),
        ],
        condition=IfCondition(use_rviz),
    )

    return [
        GroupAction([
            PushRosNamespace(namespace),
            move_group,
            rviz,
        ])
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "robot_xacro",
            default_value=_default_path(
                "robot_sim_description",
                "robots",
                "panda",
                "panda.urdf.xacro",
            ),
        ),
        DeclareLaunchArgument(
            "srdf_file",
            default_value=_default_path(
                "robot_sim_moveit_config",
                "config",
                "robots",
                "panda",
                "panda.srdf",
            ),
        ),
        DeclareLaunchArgument(
            "kinematics_yaml",
            default_value=_default_path(
                "robot_sim_moveit_config",
                "config",
                "robots",
                "panda",
                "kinematics.yaml",
            ),
        ),
        DeclareLaunchArgument(
            "joint_limits_yaml",
            default_value=_default_path(
                "robot_sim_moveit_config",
                "config",
                "robots",
                "panda",
                "joint_limits.yaml",
            ),
        ),
        DeclareLaunchArgument(
            "moveit_controllers_yaml",
            default_value=_default_path(
                "robot_sim_moveit_config",
                "config",
                "robots",
                "panda",
                "moveit_controllers.yaml",
            ),
        ),
        DeclareLaunchArgument(
            "ompl_planning_yaml",
            default_value=_default_path(
                "robot_sim_moveit_config",
                "config",
                "robots",
                "panda",
                "ompl_planning.yaml",
            ),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=_default_path(
                "robot_sim_moveit_config",
                "rviz",
                "robots",
                "panda.rviz",
            ),
        ),
        DeclareLaunchArgument(
            "monitored_planning_scene_topic",
            default_value="/monitored_planning_scene",
        ),
        DeclareLaunchArgument("camera_points_topic", default_value="/camera/points"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        OpaqueFunction(function=_launch_setup),
    ])
