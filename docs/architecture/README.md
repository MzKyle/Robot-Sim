# 架构总览

`robot_sim` 当前采用“仿真 bringup + 运控配置 + 采集测试辅助”的方式组织。仿真主链路由 `robot_sim_bringup` 驱动，world 由 `robot_sim_scenarios` 的 base/assets/scenario 组合生成；真实设备和采集测试链路由 `data_collect_bringup` 按需启动。配置文件通过 `robot_sim_control/config/panda_controllers.yaml`、launch 参数和 `src/config/nodemanage.yaml` 注入，数据流通过 ROS 主题、服务和 action 连接各个节点。

如果你的目标是把它继续演进成通用数据采集平台，建议先看 [目标架构草案](target-architecture.md)。这份文档会把 rosbag2、预览、质量评估和前端控制重新拆分成更轻的边界。

## 组件分层

```mermaid
flowchart TB
    subgraph Layer1[设备层]
        C2D[camera_pool_driver]
        C3D[camera_3d_driver]
        FANUC[fanuc_robot]
        QUALITY[data_collect_quality]
    end

    subgraph Layer2[采集层]
        COLLECT[data_collect]
        BRINGUP[data_collect_bringup]
        SIM[robot_sim_bringup]
    end

    subgraph Layer3[界面层]
        UI[data_collect_ui]
    end

    subgraph Layer4[契约层]
        TASK[robot_task_interfaces]
        ACQ[acquisition_interfaces]
        SIMIF[simulation_interfaces]
        WELD[weld_interface]
        FILE[file_reader]
    end

    BRINGUP --> C2D
    BRINGUP --> C3D
    BRINGUP --> FANUC
    BRINGUP --> QUALITY
    BRINGUP --> COLLECT
    SIM --> COLLECT
    TASK --> UI
    TASK --> COLLECT
    ACQ --> UI
    ACQ --> COLLECT
    SIMIF --> SIM
    WELD --> COLLECT
    FILE --> BRINGUP
    FILE --> UI
    C2D --> COLLECT
    C3D --> COLLECT
    FANUC --> COLLECT
    QUALITY --> COLLECT
    UI <--> COLLECT
```

## 架构特点

- 节点职责清晰，采集、展示和配置分层管理。
- 实际采集与仿真采集优先共享中性的 task/acquisition/simulation 接口，焊接字段由 `weld_interface` 作为 adapter 承载。
- 所有硬件参数集中在 `nodemanage.yaml`，避免在代码中硬编码。
- 仿真链路由 `robot_sim_bringup` 的 `sim_mode`、scenario world 和传感器组开关控制，对应 gz sim 8、Panda 机械臂、Gazebo hardware plugin 和标准 ROS 2 控制器。
- 采集状态通过 ROS 话题向外广播，UI 通过服务完成控制动作。
- 任务和历史数据都可从同一个工作空间中追踪。

## 建议阅读顺序

1. [目标架构草案](target-architecture.md)
2. [模块全景](module-overview.md)
3. [数据流](data-flow.md)
4. [状态模型](state-model.md)
5. [ROS 主题与服务](../interfaces/ros-api.md)
