from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_path = PathJoinSubstitution([
        FindPackageShare("data_collect_sim"),
        "worlds",
        "weld_cell.world.sdf",
    ])

    package_share_path = FindPackageShare("data_collect_sim")

    gazebo_classic_models_path = TextSubstitution(text="/usr/share/gazebo-11/models")

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=world_path),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [
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
                "gz_args": LaunchConfiguration("world"),
                "gz_version": "6",
            }.items(),
        ),
    ])
