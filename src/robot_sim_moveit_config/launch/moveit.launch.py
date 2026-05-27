import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def load_text(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")
    monitored_planning_scene_topic = LaunchConfiguration("monitored_planning_scene_topic")
    camera_points_topic = LaunchConfiguration("camera_points_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    robot_xacro = PathJoinSubstitution([
        FindPackageShare("robot_sim_description"),
        "urdf",
        "panda_arm.urdf.xacro",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", robot_xacro]),
            value_type=str,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": load_text(
            "robot_sim_moveit_config",
            "config/panda.srdf",
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "robot_sim_moveit_config",
            "config/kinematics.yaml",
        )
    }

    ompl_config = load_yaml("robot_sim_moveit_config", "config/ompl_planning.yaml")

    moveit_parameters = [
        {"use_sim_time": use_sim_time},
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        load_yaml("robot_sim_moveit_config", "config/joint_limits.yaml"),
        {
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
            "ompl": ompl_config,
        },
        load_yaml("robot_sim_moveit_config", "config/moveit_controllers.yaml"),
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

    rviz_config = PathJoinSubstitution([
        FindPackageShare("robot_sim_moveit_config"),
        "rviz",
        "robot_sim_moveit.rviz",
    ])
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

    moveit_group = GroupAction([
        PushRosNamespace(namespace),
        move_group,
        rviz,
    ])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "monitored_planning_scene_topic",
            default_value="/monitored_planning_scene",
        ),
        DeclareLaunchArgument("camera_points_topic", default_value="/camera/points"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        moveit_group,
    ])
