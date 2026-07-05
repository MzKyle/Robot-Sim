# 数据流

```mermaid
flowchart LR
    Xacro[URDF/xacro]
    Profile[sim_profile]
    World[Generated world]
    Mock[Mock ros2_control]
    Gazebo[Gazebo Harmonic]
    JointStates[/joint_states/]
    Controllers[ros2_control controllers]
    MoveGroup[move_group]
    Bridge[ros_gz bridge]
    Receivers[robot_sim_sensor_*]
    Diagnostics[/diagnostics/]

    Profile --> Xacro
    Profile --> World
    Xacro --> Gazebo
    World --> Gazebo
    Xacro --> Mock
    Gazebo --> JointStates
    Gazebo --> Controllers
    Mock --> Controllers
    Controllers --> MoveGroup
    Gazebo --> Bridge
    Bridge --> Receivers
    Receivers --> Diagnostics
```

关键路径：

1. profile 决定机器人、world、controller、MoveIt、传感器和 bridge。
2. xacro 渲染出机器人描述并传给 `robot_state_publisher`、Gazebo 和 MoveIt。
3. `mock` 模式通过 `mock_components/GenericSystem` 创建控制链；`light/full` 模式通过 Gazebo 的 `gz_ros2_control/GazeboSimSystem` 创建控制链。
4. MoveIt 使用 controller action 执行规划轨迹。
5. `full` 或显式开启传感器时，传感器数据经 `ros_gz_bridge` 到 ROS 话题，再由 receiver 统计并发布 diagnostics。
