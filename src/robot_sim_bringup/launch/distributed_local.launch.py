import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


ROBOT_NAMESPACE = "robot"
SENSOR_NAMESPACE = "sensors"


def _normalized(context, name):
    return context.launch_configurations.get(name, "").strip().lower()


def _bool_value(value, name):
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False
    raise RuntimeError(f"{name} must be auto, true, or false; got '{value}'")


def _auto_bool(context, name, default):
    value = _normalized(context, name)
    if value == "auto":
        return default
    return _bool_value(value, name)


def _bool_text(value):
    return "true" if value else "false"


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
    sim_mode = _normalized(context, "sim_mode")
    if sim_mode not in ("mock", "light", "full"):
        raise RuntimeError(f"sim_mode must be mock, light, or full; got '{sim_mode}'")

    mode_defaults = {
        "mock": {
            "use_gazebo": False,
            "use_sim_time": False,
            "use_moveit": False,
            "rviz": False,
            "headless": True,
            "sensors": False,
        },
        "light": {
            "use_gazebo": True,
            "use_sim_time": True,
            "use_moveit": False,
            "rviz": False,
            "headless": True,
            "sensors": False,
        },
        "full": {
            "use_gazebo": True,
            "use_sim_time": True,
            "use_moveit": True,
            "rviz": True,
            "headless": False,
            "sensors": True,
        },
    }[sim_mode]

    use_gazebo = mode_defaults["use_gazebo"]
    use_sim_time = _auto_bool(context, "use_sim_time", mode_defaults["use_sim_time"])
    use_moveit = _auto_bool(context, "use_moveit", mode_defaults["use_moveit"])
    use_rviz = _auto_bool(context, "rviz", mode_defaults["rviz"])
    headless = _auto_bool(context, "headless", mode_defaults["headless"])
    rqt_graph = _bool_value(_normalized(context, "rqt_graph"), "rqt_graph")

    enable_camera = _auto_bool(context, "enable_camera", mode_defaults["sensors"])
    enable_depth = _auto_bool(context, "enable_depth", mode_defaults["sensors"])
    enable_lidar = _auto_bool(context, "enable_lidar", mode_defaults["sensors"])
    enable_imu = _auto_bool(context, "enable_imu", mode_defaults["sensors"])

    description_share = get_package_share_directory("robot_sim_description")
    bringup_share = get_package_share_directory("robot_sim_bringup")
    control_share = get_package_share_directory("robot_sim_control")
    moveit_share = get_package_share_directory("robot_sim_moveit_config")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    scenarios_share = get_package_share_directory("robot_sim_scenarios")

    world_path = os.path.join(scenarios_share, "worlds", "robot_lab_distributed.world.sdf")
    robot_xacro = os.path.join(description_share, "urdf", "panda_arm.urdf.xacro")
    controllers_yaml = os.path.join(control_share, "config", "panda_controllers.yaml")
    mesh_root = "file://" + os.path.join(description_share, "models", "panda_arm", "meshes")
    camera_mesh = (
        "file://"
        + os.path.join(description_share, "models", "rgbd_camera", "meshes", "3dCamera.DAE")
    )

    hardware_plugin = (
        "gz_ros2_control/GazeboSimSystem"
        if use_gazebo
        else "mock_components/GenericSystem"
    )
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            robot_xacro,
            " hardware_plugin:=",
            hardware_plugin,
            " controllers_file:=",
            controllers_yaml,
            " mesh_root:=",
            mesh_root,
            " camera_mesh:=",
            camera_mesh,
            " use_gz_ros2_control:=",
            _bool_text(use_gazebo),
            " ros_namespace:=",
            ROBOT_NAMESPACE,
            " enable_camera:=",
            _bool_text(enable_camera),
            " enable_depth:=",
            _bool_text(enable_depth),
            " enable_lidar:=",
            _bool_text(enable_lidar),
            " enable_imu:=",
            _bool_text(enable_imu),
        ]),
        value_type=str,
    )

    robot_stack_actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }],
        )
    ]
    if not use_gazebo:
        robot_stack_actions.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": robot_description,
                    },
                    controllers_yaml,
                ],
            )
        )

    actions = [
        LogInfo(
            msg=(
                "robot_sim_bringup distributed: "
                f"sim_mode={sim_mode}, gazebo={_bool_text(use_gazebo)}, "
                f"use_sim_time={_bool_text(use_sim_time)}, "
                f"camera={_bool_text(enable_camera)}, depth={_bool_text(enable_depth)}, "
                f"lidar={_bool_text(enable_lidar)}, imu={_bool_text(enable_imu)}"
            )
        ),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [
                description_share,
                ":",
                os.path.join(description_share, "models"),
                ":",
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            ],
        ),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            [
                description_share,
                ":",
                os.path.join(description_share, "models"),
                ":",
                os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
            ],
        ),
        GroupAction([
            PushRosNamespace(ROBOT_NAMESPACE),
            *robot_stack_actions,
        ]),
    ]

    if use_gazebo:
        actions.extend([
            TimerAction(
                period=1.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
                        ),
                        launch_arguments={
                            "gz_args": ["-r -s " if headless else "-r ", world_path],
                            "gz_version": "8",
                            "on_exit_shutdown": "true",
                        }.items(),
                    )
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        name="spawn_panda",
                        output="screen",
                        arguments=[
                            "-name",
                            "panda",
                            "-topic",
                            f"/{ROBOT_NAMESPACE}/robot_description",
                            "-allow_renaming",
                            "false",
                        ],
                    ),
                    GroupAction([
                        PushRosNamespace(SENSOR_NAMESPACE),
                        _bridge_node(os.path.join(bringup_share, "config"), "clock"),
                    ]),
                ],
            ),
        ])

        sensor_bridge_nodes = []
        config_dir = os.path.join(bringup_share, "config")
        if enable_camera:
            sensor_bridge_nodes.append(_bridge_node(config_dir, "camera"))
        if enable_depth:
            sensor_bridge_nodes.append(_bridge_node(config_dir, "depth"))
        if enable_lidar:
            sensor_bridge_nodes.append(_bridge_node(config_dir, "lidar"))
        if enable_imu:
            sensor_bridge_nodes.append(_bridge_node(config_dir, "imu"))
        if sensor_bridge_nodes:
            actions.append(
                TimerAction(
                    period=2.0,
                    actions=[
                        GroupAction([
                            PushRosNamespace(SENSOR_NAMESPACE),
                            *sensor_bridge_nodes,
                        ])
                    ],
                )
            )

    controller_manager_name = f"/{ROBOT_NAMESPACE}/controller_manager"
    actions.append(
        TimerAction(
            period=5.0 if use_gazebo else 2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    output="screen",
                    arguments=[
                        "joint_state_broadcaster",
                        "-c",
                        controller_manager_name,
                        "-p",
                        controllers_yaml,
                        "-t",
                        "joint_state_broadcaster/JointStateBroadcaster",
                        "--controller-manager-timeout",
                        "90",
                    ],
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    output="screen",
                    arguments=[
                        "arm_controller",
                        "-c",
                        controller_manager_name,
                        "-p",
                        controllers_yaml,
                        "-t",
                        "joint_trajectory_controller/JointTrajectoryController",
                        "--controller-manager-timeout",
                        "90",
                    ],
                ),
            ],
        )
    )

    if use_moveit:
        actions.append(
            TimerAction(
                period=7.0 if use_gazebo else 4.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(moveit_share, "launch", "moveit.launch.py")
                        ),
                        launch_arguments={
                            "namespace": ROBOT_NAMESPACE,
                            "use_sim_time": _bool_text(use_sim_time),
                            "rviz": _bool_text(use_rviz),
                        }.items(),
                    )
                ],
            )
        )

    if rqt_graph:
        actions.append(
            Node(
                package="rqt_graph",
                executable="rqt_graph",
                name="supervisor_graph",
                namespace="supervisor",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim_mode", default_value="light"),
        DeclareLaunchArgument("enable_camera", default_value="auto"),
        DeclareLaunchArgument("enable_depth", default_value="auto"),
        DeclareLaunchArgument("enable_lidar", default_value="auto"),
        DeclareLaunchArgument("enable_imu", default_value="auto"),
        DeclareLaunchArgument("use_sim_time", default_value="auto"),
        DeclareLaunchArgument("use_moveit", default_value="auto"),
        DeclareLaunchArgument("rviz", default_value="auto"),
        DeclareLaunchArgument("headless", default_value="auto"),
        DeclareLaunchArgument("rqt_graph", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
