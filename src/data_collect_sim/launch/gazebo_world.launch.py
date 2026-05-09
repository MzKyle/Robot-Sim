from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share_dir = get_package_share_directory("data_collect_sim")
    package_share_parent_dir = os.path.dirname(package_share_dir)
    package_share_path = FindPackageShare("data_collect_sim")

    world_path = PathJoinSubstitution([
        package_share_path,
        "worlds",
        "weld_cell.world.sdf",
    ])

    gazebo_classic_models_path = TextSubstitution(text="/usr/share/gazebo-11/models")

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=world_path),
        DeclareLaunchArgument("use_gz_sensors", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("gz_partition", default_value=TextSubstitution(text="data_collect_sim")),
        SetEnvironmentVariable("GZ_PARTITION", LaunchConfiguration("gz_partition")),
        SetEnvironmentVariable("IGN_PARTITION", LaunchConfiguration("gz_partition")),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [
                TextSubstitution(text=package_share_parent_dir),
                TextSubstitution(text=":"),
                package_share_path,
                TextSubstitution(text=":"),
                gazebo_classic_models_path,
                TextSubstitution(text=":"),
                EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            [
                TextSubstitution(text=package_share_parent_dir),
                TextSubstitution(text=":"),
                package_share_path,
                TextSubstitution(text=":"),
                gazebo_classic_models_path,
                TextSubstitution(text=":"),
                EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ])
            ),
            launch_arguments={
                "gz_args": [
                    TextSubstitution(text="-r "),
                    LaunchConfiguration("world"),
                ],
                "gz_version": "6",
                "on_exit_shutdown": "true",
            }.items(),
        ),
        Node(
            package="data_collect_sim",
            executable="spawn_fanuc_m20i_node",
            name="spawn_fanuc_m20i",
            output="screen",
            parameters=[{
                "model_name": "fanuc_m20i",
                "world_name": "weld_cell",
                "spawn_delay": 3.0,
                "use_gz_sensors": LaunchConfiguration("use_gz_sensors"),
            }],
        ),
        Node(
            package="data_collect_sim",
            executable="check_gz_model_node",
            name="check_gz_model",
            output="screen",
            parameters=[{
                "model_name": "fanuc_m20i",
                "world_name": "weld_cell",
                "check_delay": 7.0,
            }],
        ),
    ])
