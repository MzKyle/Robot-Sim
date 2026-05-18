import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace


def _bool_value(context, name):
    value = context.launch_configurations.get(name, "").strip().lower()
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False
    raise RuntimeError(f"{name} must be true or false; got '{value}'")


def _bridge_node(config_dir, group):
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"{group}_bridge",
        output="screen",
        parameters=[{
            "config_file": os.path.join(config_dir, f"ros_gz_bridge_{group}.yaml"),
        }],
    )


def _launch_setup(context, *args, **kwargs):
    namespace = context.launch_configurations.get("namespace", "")
    config_dir = os.path.join(get_package_share_directory("robot_sim_bringup"), "config")
    bridge_nodes = [_bridge_node(config_dir, "clock")]
    for group in ("camera", "depth", "lidar", "imu"):
        if _bool_value(context, f"enable_{group}"):
            bridge_nodes.append(_bridge_node(config_dir, group))

    if namespace:
        return [GroupAction([PushRosNamespace(namespace), *bridge_nodes])]
    return bridge_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("enable_camera", default_value="true"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("enable_lidar", default_value="true"),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
