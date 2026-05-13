from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_config = PathJoinSubstitution([
        FindPackageShare("data_collect_sim"),
        "config",
        "nodemanage_sim.yaml",
    ])
    camera_bridge_config = PathJoinSubstitution([
        FindPackageShare("data_collect_sim"),
        "config",
        "ros_gz_bridge_sensors.yaml",
    ])
    tf_bridge_config = PathJoinSubstitution([
        FindPackageShare("data_collect_sim"),
        "config",
        "ros_gz_bridge_tf.yaml",
    ])
    joint_bridge_config = PathJoinSubstitution([
        FindPackageShare("data_collect_sim"),
        "config",
        "ros_gz_bridge_joints.yaml",
    ])

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
        DeclareLaunchArgument("use_tf_to_tcp", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("use_gz_sensors", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("use_gz_joint_control", default_value=TextSubstitution(text="false")),
        gazebo_launch,
        bringup_launch,
        Node(
            package="data_collect_sim",
            executable="panda_joint_demo_node",
            name="panda_joint_demo",
            output="screen",
            parameters=[{
                "model_name": "panda_weld_arm",
                "publish_rate": 30.0,
                "motion_period": 12.0,
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
            parameters=[sim_config, {
                "base_frame": "world",
                "tool_frame": "panda_weld_arm/panda_link8",
            }],
            condition=IfCondition(LaunchConfiguration("use_tf_to_tcp")),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_camera_bridge",
            output="screen",
            parameters=[{"config_file": camera_bridge_config}],
            condition=IfCondition(PythonExpression([
                "'",
                LaunchConfiguration("use_gazebo"),
                "' == 'true' and '",
                LaunchConfiguration("use_gz_sensors"),
                "' == 'true'",
            ])),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_tf_bridge",
            output="screen",
            parameters=[{"config_file": tf_bridge_config}],
            condition=IfCondition(PythonExpression([
                "'",
                LaunchConfiguration("use_gazebo"),
                "' == 'true' and '",
                LaunchConfiguration("use_tf_to_tcp"),
                "' == 'true'",
            ])),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_joint_bridge",
            output="screen",
            parameters=[{"config_file": joint_bridge_config}],
            condition=IfCondition(PythonExpression([
                "'",
                LaunchConfiguration("use_gazebo"),
                "' == 'true' and '",
                LaunchConfiguration("use_gz_joint_control"),
                "' == 'true'",
            ])),
        ),
    ])
