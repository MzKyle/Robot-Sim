from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _optional_int(context, name):
    value = LaunchConfiguration(name).perform(context).strip()
    if not value:
        return None
    return int(value)


def _optional_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if not value:
        return None
    return value in ("1", "true", "yes", "on")


def _optional_str(context, name):
    value = LaunchConfiguration(name).perform(context).strip()
    return value if value else None


def _node_actions(context):
    nodemanage_yaml = LaunchConfiguration("nodemanage_yaml")

    fanuc_overrides = {}
    fanuc_so_path = _optional_str(context, "fanuc_so_path")
    robot_ip = _optional_str(context, "robot_ip")
    robot_port = _optional_int(context, "robot_port")
    target_register_index = _optional_int(context, "target_register_index")
    if fanuc_so_path is not None:
        fanuc_overrides["so_file_path"] = fanuc_so_path
    if robot_ip is not None:
        fanuc_overrides["robot_ip"] = robot_ip
    if robot_port is not None:
        fanuc_overrides["robot_port"] = robot_port
    if target_register_index is not None:
        fanuc_overrides["target_register_index"] = target_register_index

    camera_3d_overrides = {}
    publish_tf = _optional_bool(context, "publish_tf")
    if publish_tf is not None:
        camera_3d_overrides["publish_tf"] = publish_tf

    return [
        Node(
            package="fanuc_robot",
            executable="fanuc_robot",
            output="screen",
            parameters=[nodemanage_yaml] + ([fanuc_overrides] if fanuc_overrides else []),
            condition=IfCondition(LaunchConfiguration("enable_fanuc")),
        ),
        Node(
            package="camera_3d_driver",
            executable="camera_3d_driver",
            output="screen",
            parameters=[nodemanage_yaml] + ([camera_3d_overrides] if camera_3d_overrides else []),
            condition=IfCondition(LaunchConfiguration("enable_camera_3d")),
        ),
        Node(
            package="camera_pool_driver",
            executable="camera_pool_driver",
            output="screen",
            parameters=[nodemanage_yaml],
            condition=IfCondition(LaunchConfiguration("enable_camera_2d")),
        ),
        Node(
            package="data_collect",
            executable="data_collect_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_data_collect")),
        ),
        Node(
            package="data_collect_quality",
            executable="data_collect_quality_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_data_collect_quality")),
        ),
    ]


def generate_launch_description():
    nodemanage_yaml = LaunchConfiguration("nodemanage_yaml")
    rvc_lib_dir = LaunchConfiguration("rvc_lib_dir")

    default_nodemanage_yaml = PathJoinSubstitution([
        FindPackageShare("data_collect_bringup"),
        "config",
        "nodemanage.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("nodemanage_yaml", default_value=default_nodemanage_yaml),
        DeclareLaunchArgument(
            "fanuc_so_path",
            default_value=TextSubstitution(text=""),
        ),
        DeclareLaunchArgument("robot_ip", default_value=TextSubstitution(text="")),
        DeclareLaunchArgument("robot_port", default_value=TextSubstitution(text="")),
        DeclareLaunchArgument("target_register_index", default_value=TextSubstitution(text="")),
        DeclareLaunchArgument("publish_tf", default_value=TextSubstitution(text="")),
        DeclareLaunchArgument(
            "rvc_lib_dir",
            default_value=EnvironmentVariable(
                "RVC_LIB_DIR",
                default_value=PathJoinSubstitution([
                    EnvironmentVariable("RVC_ROOT", default_value=TextSubstitution(text="/opt/RVC")),
                    "lib",
                ]),
            ),
        ),
        DeclareLaunchArgument("enable_fanuc", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("enable_camera_3d", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("enable_camera_2d", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("enable_data_collect", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("enable_data_collect_quality", default_value=TextSubstitution(text="true")),

        SetEnvironmentVariable("AUTOCOVER_NODEMANAGE_YAML", nodemanage_yaml),
        SetEnvironmentVariable(
            "LD_LIBRARY_PATH",
            [rvc_lib_dir, TextSubstitution(text=":"), EnvironmentVariable("LD_LIBRARY_PATH", default_value="")],
        ),

        OpaqueFunction(function=_node_actions),
    ])
