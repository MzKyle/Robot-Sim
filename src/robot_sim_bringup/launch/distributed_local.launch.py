from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from robot_sim_bringup.sim_launch_builder import build_sim_launch_actions


def _launch_setup(context, *args, **kwargs):
    return build_sim_launch_actions(context, "distributed")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim_profile", default_value="panda"),
        DeclareLaunchArgument("sim_profile_file", default_value=""),
        DeclareLaunchArgument("sim_mode", default_value="light"),
        DeclareLaunchArgument("sensor_overrides", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="auto"),
        DeclareLaunchArgument("use_moveit", default_value="auto"),
        DeclareLaunchArgument("rviz", default_value="auto"),
        DeclareLaunchArgument("headless", default_value="auto"),
        DeclareLaunchArgument("use_gripper", default_value="false"),
        DeclareLaunchArgument("rqt_graph", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
