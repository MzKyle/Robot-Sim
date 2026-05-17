from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    bridge_config = PathJoinSubstitution([
        FindPackageShare("robot_sim_bringup"),
        "config",
        "ros_gz_bridge_sensors.yaml",
    ])

    sensor_bridge = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="sensor_bridge",
            output="screen",
            parameters=[{"config_file": bridge_config}],
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        sensor_bridge,
    ])
