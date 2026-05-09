from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_config = PathJoinSubstitution([
        FindPackageShare("data_collect_sim"),
        "config",
        "nodemanage_sim.yaml",
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
            "enable_gz_camera_plugins": LaunchConfiguration("enable_gz_camera_plugins"),
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
        DeclareLaunchArgument("use_gz_bridge", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("enable_gz_camera_plugins", default_value=TextSubstitution(text="false")),
        gazebo_launch,
        bringup_launch,
        Node(
            package="data_collect_sim",
            executable="sim_camera_2d_node",
            name="camera_node",
            output="screen",
            parameters=[sim_config],
            condition=IfCondition(LaunchConfiguration("use_sim_camera_2d")),
        ),
        Node(
            package="data_collect_sim",
            executable="sim_camera_3d_node",
            name="camera_driver_3d",
            output="screen",
            parameters=[sim_config],
            condition=IfCondition(LaunchConfiguration("use_sim_camera_3d")),
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
        # parameter_bridge from ros_gz_bridge: map Gazebo sensors to ROS2 topics
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/image_topic@sensor_msgs/msg/Image@ignition.msgs.Image',
                 '/tcp_depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
                 '/tcp_cloud_raw@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'],
            output='screen',
            condition=IfCondition(LaunchConfiguration("use_gz_bridge")),
        ),
    ])
