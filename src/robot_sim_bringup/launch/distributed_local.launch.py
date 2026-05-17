from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_moveit = LaunchConfiguration("use_moveit")
    use_rviz = LaunchConfiguration("rviz")
    headless = LaunchConfiguration("headless")

    description_share = FindPackageShare("robot_sim_description")
    world_path = PathJoinSubstitution([
        FindPackageShare("robot_sim_scenarios"),
        "worlds",
        "robot_lab_distributed.world.sdf",
    ])
    robot_xacro = PathJoinSubstitution([
        description_share,
        "urdf",
        "panda_arm.urdf.xacro",
    ])
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("robot_sim_control"),
        "config",
        "panda_controllers.yaml",
    ])
    joint_bridge_config = PathJoinSubstitution([
        FindPackageShare("robot_sim_bringup"),
        "config",
        "ros_gz_bridge_joints.yaml",
    ])

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", robot_xacro]),
        value_type=str,
    )

    robot_stack = GroupAction([
        PushRosNamespace("robot"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }],
        ),
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
        ),
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ),
        launch_arguments={
            "gz_args": [
                PythonExpression([
                    "'-r -s ' if '",
                    headless,
                    "' == 'true' else '-r '",
                ]),
                world_path,
            ],
            "gz_version": "8",
            "on_exit_shutdown": "true",
        }.items(),
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot_sim_bringup"),
                "launch",
                "sensors.launch.py",
            ])
        ),
        launch_arguments={"namespace": "sensors"}.items(),
    )
    joint_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="joint_command_bridge",
        output="screen",
        parameters=[{"config_file": joint_bridge_config}],
    )
    joint_command_follower = Node(
        package="robot_sim_control",
        executable="joint_state_to_gz_joint_cmd_node",
        name="joint_state_to_gz_joint_cmd",
        output="screen",
        parameters=[{
            "model_name": "panda",
            "joint_state_topic": "/robot/joint_states",
            "publish_rate": 50.0,
        }],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/robot/controller_manager",
            "-p",
            controllers_yaml,
            "-t",
            "joint_state_broadcaster/JointStateBroadcaster",
            "--controller-manager-timeout",
            "90",
        ],
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "-c",
            "/robot/controller_manager",
            "-p",
            controllers_yaml,
            "-t",
            "joint_trajectory_controller/JointTrajectoryController",
            "--controller-manager-timeout",
            "90",
        ],
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot_sim_moveit_config"),
                "launch",
                "moveit.launch.py",
            ])
        ),
        launch_arguments={
            "namespace": "robot",
            "use_sim_time": use_sim_time,
            "rviz": use_rviz,
        }.items(),
        condition=IfCondition(use_moveit),
    )

    supervisor_shell = Node(
        package="rqt_graph",
        executable="rqt_graph",
        name="supervisor_graph",
        namespace="supervisor",
        condition=IfCondition(LaunchConfiguration("rqt_graph")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_moveit", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rqt_graph", default_value="false"),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [
                description_share,
                TextSubstitution(text=":"),
                PathJoinSubstitution([description_share, "models"]),
                TextSubstitution(text=":"),
                EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            [
                description_share,
                TextSubstitution(text=":"),
                PathJoinSubstitution([description_share, "models"]),
                TextSubstitution(text=":"),
                EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
            ],
        ),
        robot_stack,
        TimerAction(period=1.0, actions=[gz_sim]),
        TimerAction(period=2.0, actions=[sensors, joint_bridge]),
        TimerAction(period=5.0, actions=[joint_state_spawner, arm_spawner]),
        TimerAction(period=6.0, actions=[joint_command_follower]),
        TimerAction(period=7.0, actions=[moveit, supervisor_shell]),
    ])
