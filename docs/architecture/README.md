# 架构总览

`robot_sim` 当前采用“仿真 bringup + 运控配置 + 仿真传感器接收”的方式组织。仿真主链路由 `robot_sim_bringup` 驱动，world 由 `robot_sim_scenarios` 的 base/assets/scenario 组合生成；传感器数据通过 `ros_gz_bridge` 和 `robot_sim_sensors` 接收验证。旧硬件采集启动链路已移除。

如果你的目标是把它继续演进成通用数据采集平台，建议先看 [目标架构草案](target-architecture.md)。这份文档会把 rosbag2、预览、质量评估和前端控制重新拆分成更轻的边界。

## 组件分层

```mermaid
flowchart TB
    subgraph Layer1[仿真入口层]
        SIM[robot_sim_bringup]
        DESC[robot_sim_description]
        CTRL[robot_sim_control]
        MOVEIT[robot_sim_moveit_config]
        SCENE[robot_sim_scenarios]
    end

    subgraph Layer2[Gazebo 与传感器层]
        BRIDGE[ros_gz_bridge]
        RECV[robot_sim_sensors]
    end

    subgraph Layer3[契约层]
        TASK[robot_task_interfaces]
        SIMIF[simulation_interfaces]
    end

    SIM --> DESC
    SIM --> CTRL
    SIM --> MOVEIT
    SIM --> SCENE
    SIM --> BRIDGE
    BRIDGE --> RECV
    SIMIF --> SIM
    TASK --> SIM
```

## 架构特点

- 节点职责清晰，机器人描述、控制、规划、场景和传感器接收各自独立。
- 仿真链路由 `robot_sim_bringup` 的 `sim_mode`、scenario world 和传感器组开关控制，对应 gz sim 8、Panda/Fanuc M20iD 机械臂、Gazebo hardware plugin 和标准 ROS 2 控制器。
- 仿真传感器接收由 profile 中的 `sensors.<name>.receiver` 声明，receiver 订阅原生仿真话题并发布 diagnostics。
- 具体机械臂型号只保留在 profile、URDF、控制器、MoveIt 和模型资源子目录中。

## 建议阅读顺序

1. [目标架构草案](target-architecture.md)
2. [模块全景](module-overview.md)
3. [数据流](data-flow.md)
4. [状态模型](state-model.md)
5. [ROS 主题与服务](../interfaces/ros-api.md)
