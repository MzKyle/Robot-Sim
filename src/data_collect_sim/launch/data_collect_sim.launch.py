from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_config = PathJoinSubstitution([
        FindPackageShare("data_collect_sim"),
        "config",
        "nodemanage_sim.yaml",
    ])
    robot_xacro = PathJoinSubstitution([
        FindPackageShare("data_collect_sim"),
        "urdf",
        "fanuc_m20ib25_gz.xacro",
    ])
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                robot_xacro,
                " use_gz_sensors:=",
                LaunchConfiguration("use_gz_sensors"),
                " base_x:=0.2",
                " base_y:=0.0",
                " base_z:=0.125",
                " base_roll:=0.0",
                " base_pitch:=0.0",
                " base_yaw:=0.0",
            ]),
            value_type=str,
        )
    }

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("data_collect_sim"),
                "launch",
                "gazebo_world.launch.py",
            ])
        ),
        launch_arguments={
            "use_gz_sensors": LaunchConfiguration("use_gz_sensors"),
            "gz_partition": "data_collect_sim",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("data_collect_bringup"),
                "launch",
                "data_collect.launch.py",
            ])
        ),
        launch_arguments={
            "nodemanage_yaml": sim_config,
            "enable_fanuc": "false",
            "enable_camera_3d": "false",
            "enable_camera_2d": "false",
            "enable_data_collect": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_gazebo", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("use_sim_camera_2d", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("use_sim_camera_3d", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("use_sim_fanuc", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("use_tf_to_tcp", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("use_gz_sensors", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("use_robot_state_publisher", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("use_joint_state_gui", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("use_gz_joint_control", default_value=TextSubstitution(text="false")),
        gazebo_launch,
        bringup_launch,
        Node(
            package="data_collect_sim",
            executable="robot_description_publisher_node",
            name="robot_description_publisher",
            output="screen",
            parameters=[{
                "use_gz_sensors": LaunchConfiguration("use_gz_sensors"),
                "base_x": 0.2,
                "base_y": 0.0,
                "base_z": 0.125,
                "base_roll": 0.0,
                "base_pitch": 0.0,
                "base_yaw": 0.0,
                "publish_period": 1.0,
            }],
            condition=IfCondition(LaunchConfiguration("use_robot_state_publisher")),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
            condition=IfCondition(LaunchConfiguration("use_robot_state_publisher")),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[robot_description, {"rate": 30}],
            condition=IfCondition(PythonExpression([
                "'",
                LaunchConfiguration("use_robot_state_publisher"),
                "' == 'true' and '",
                LaunchConfiguration("use_joint_state_gui"),
                "' != 'true'",
            ])),
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            parameters=[robot_description],
            condition=IfCondition(LaunchConfiguration("use_joint_state_gui")),
        ),
        Node(
            package="data_collect_sim",
            executable="joint_state_to_gz_joint_cmd_node",
            name="joint_state_to_gz_joint_cmd",
            output="screen",
            parameters=[{
                "model_name": "fanuc_m20i",
                "joint_state_topic": "/joint_states",
                "publish_rate": 30.0,
            }],
            condition=IfCondition(PythonExpression([
                "'",
                LaunchConfiguration("use_gazebo"),
                "' == 'true' and '",
                LaunchConfiguration("use_gz_joint_control"),
                "' == 'true'",
            ])),
        ),
        Node(
            package="data_collect_sim",
            executable="sim_camera_2d_node",
            name="camera_node",
            output="screen",
            parameters=[sim_config],
            condition=IfCondition(PythonExpression([
                "'",
                LaunchConfiguration("use_sim_camera_2d"),
                "' == 'true' and '",
                LaunchConfiguration("use_gz_sensors"),
                "' != 'true'",
            ])),
        ),
        Node(
            package="data_collect_sim",
            executable="sim_camera_3d_node",
            name="camera_driver_3d",
            output="screen",
            parameters=[sim_config],
            condition=IfCondition(PythonExpression([
                "'",
                LaunchConfiguration("use_sim_camera_3d"),
                "' == 'true' and '",
                LaunchConfiguration("use_gz_sensors"),
                "' != 'true'",
            ])),
        ),
        Node(
            package="data_collect_sim",
            executable="sim_fanuc_robot_node",
            name="robot_driver_fanuc",
            output="screen",
            parameters=[sim_config],
            condition=IfCondition(LaunchConfiguration("use_sim_fanuc")),
        ),
        Node(
            package="data_collect_sim",
            executable="tf_to_tcp_node",
            name="tf_to_tcp",
            output="screen",
            parameters=[sim_config],
            condition=IfCondition(LaunchConfiguration("use_tf_to_tcp")),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_sensor_bridge",
            output="screen",
            parameters=[{
                "config_file": PathJoinSubstitution([
                    FindPackageShare("data_collect_sim"),
                    "config",
                    "ros_gz_bridge_sensors.yaml",
                ]),
                "override_frame_id": "camera_mount",
            }],
            condition=IfCondition(PythonExpression([
                "'",
                LaunchConfiguration("use_gazebo"),
                "' == 'true' and ('",
                LaunchConfiguration("use_gz_sensors"),
                "' == 'true' or '",
                LaunchConfiguration("use_gz_joint_control"),
                "' == 'true')",
            ])),
        ),
    ])
