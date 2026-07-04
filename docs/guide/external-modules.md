# 外部 ROS2 模块接入指南

`robot_sim` 可以把外部 ROS2 模块放进同一次仿真验收。典型场景是：机器人、Gazebo、MoveIt 和传感器由 `robot_sim` 启动，业务模块由 `validation_case.module` 启动，缺失的现场接口由 `adapters` 模拟，最后统一输出 `manifest.json`、`metrics.json`、rosbag 和报告。

当前第一版目标是“通用模块仿真验收”，不是高保真工艺物理仿真。它适合验证服务调用、topic 输出、状态机、目标点、运动请求和报告指标；不模拟焊接熔池、弧光、烟尘、热变形或真实夹具接触。

## 适用边界

适合接入：

- 依赖机器人 TCP、TF、点云、图像、MoveIt 或控制服务的 ROS2 算法模块。
- 需要在 Gazebo/RViz 中复现完整链路的定位、纠偏、抓取、检测、分拣模块。
- 需要把服务返回、topic 频率、状态机状态、误差指标写入验收报告的模块。

暂不适合：

- 必须依赖真实硬件驱动时序、工艺热过程或专用 Gazebo 插件的闭环。
- 要求 Web UI、批量矩阵 runner 或大规模数据集评测的任务。

## validation_case 结构

外部模块用例仍然是 `schema: 3` 的 `validation_case`，新增三个可选块：

```yaml
task:
  type: module_validation
  seed: 41
  start_region: planning_start
  goal_region: planning_goal
  moveit:
    group: manipulator
    target_link: tool0
    frame: world
    planning_time_sec: 10.0
    velocity_scaling: 0.2
    acceleration_scaling: 0.2

module:
  launch:
    package: my_algorithm_pkg
    file: algorithm.launch.py
    arguments:
      config_path: /path/to/config.yaml
  wait_services:
    - name: /algorithm/start
      type: std_srvs/srv/Trigger
  actions:
    - name: run_algorithm
      type: service_call
      service: /algorithm/start
      service_type: std_srvs/srv/Trigger
      expect:
        success_field: success

adapters:
  - name: tool_pos_from_tf
    type: tf_to_tcp_pos
    topic: /tool_pos
    parent_frame: world
    child_frame: tool0

expect:
  module:
    required_actions: [run_algorithm]
    topics:
      - name: /algorithm/state
        type: std_msgs/msg/String
        min_count: 1
        expect:
          data:
            contains: RUNNING
```

`run_case` 会先启动仿真，再启动 adapters，再启动外部模块，最后执行 `module.actions` 和 `expect.module` 检查。外部模块失败时仍会生成完整 artifact。

## 已内置 Adapter

| Adapter | 作用 |
| --- | --- |
| `tf_to_tcp_pos` | 读取 TF `parent_frame -> child_frame`，发布 `weld_interface/msg/TcpPos` 到 `/tool_pos` |
| `moveit_pose_service` | 提供 `/any_mov_jog`，把 `weld_interface/srv/SpecialSpeedl` 请求转成 MoveIt pose goal |
| `scan3d_service` | 提供 `/scan_3d`，支持 replay record，也支持 dataset 目录中的真实图片、同名 `.npz` 点云和合成点云 fallback |
| `synthetic_weld_vision` | 发布 `/welding/vision_result`，用于 2D 纠偏干运行 |
| `loop_motion_services` | 提供连续纠偏相关 no-op/低频接口：loop position、rate、enable、stop |

adapter 使用动态 ROS 类型加载。`robot_sim` 核心构建不依赖外部接口包；运行相关 case 前需要 source 外部工作空间，例如：

```bash
source /home/kyle/sany/ROS2_Motion_Planner/install/setup.bash
```

缺少 `weld_interface`、服务类型或 action 类型时，preflight 会在报告中明确失败。

## `/scan_3d` 数据源

`scan3d_service` 返回 `weld_interface/srv/Scan3d`，包含 `sensor_msgs/Image`、
`sensor_msgs/PointCloud2` 和 `weld_interface/msg/TcpPos scan_pose`。数据源有两类：

```yaml
adapters:
  - name: dataset_scan3d
    type: scan3d_service
    service: /scan_3d
    source:
      type: dataset
      dataset_dir: /home/kyle/sany/data/3dcamera_2d_img
      image_glob: "*.png"
      frame_policy: first
      fallback_record_path: /path/to/scan3d_capture.json
      x_span_m: 1.2
      y_span_m: 0.9
      z_m: 0.88
      image_frame_id: camera_frame
      point_cloud_frame_id: camera_frame
      scan_pose:
        x: 0.82
        y: -0.12
        z: 1.05
        rx: 3.14159
        ry: 0.0
        rz: 0.0
```

dataset 模式按以下优先级取数据：

1. `dataset_dir` 中可用的 capture JSON 记录，记录需能解析出图片和 `.npz` 点云。
2. `dataset_dir` 中匹配 `image_glob` 的图片和同名 `.npz` 点云，例如 `1.png` + `1.npz`。
3. 只有图片时，按 `x_span_m`、`y_span_m`、`z_m` 生成规则平面点云。
4. 目录没有可用图片或记录时，使用 `fallback_record_path`。

点云 `.npz` 必须包含 `points` 字段，shape 为 `(H, W, 3)`；平铺 `(H*W, 3)` 也会按图片尺寸 reshape。灰度图片默认转换为 `bgr8`，避免外部视觉算法只按彩色图处理。

`frame_policy` 控制多帧行为：

| 策略 | 行为 |
| --- | --- |
| `first` | 默认策略，总是使用排序后的第一帧 |
| `index` | 使用 `frame_index` 指定的单帧 |
| `sequential` | `/scan_3d` 每被调用一次就返回下一帧，到末尾后循环 |

报告和 `metrics.json` 会记录 `adapter_data_sources`，包括数据源类型、图片路径、点云路径或合成点云参数、帧策略和选中的帧。

## 参考接入：ROS2_Motion_Planner

当前内置两个参考用例：

| Case | 验收内容 |
| --- | --- |
| `weld_pre_positioning_scan_and_move` | 启动 Fanuc 工业场景和焊前定位模块，`/scan_3d` 优先使用 `/home/kyle/sany/data/3dcamera_2d_img` 的真实图片/点云，缺点云时合成，缺数据时回退 replay；调用 `/scan_and_detect_welding_seam` 与 `/move_to_detected_welding_seam` |
| `weld_2d_lateral_correction_dry_run` | 启动 `welding_executor_node` 干运行，合成 `/welding/vision_result`，验证 state、lateral error 和 target TCP |

运行前准备：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source /home/kyle/sany/ROS2_Motion_Planner/install/setup.bash
```

焊前定位：

```bash
ros2 run robot_sim_bringup run_case \
  --case weld_pre_positioning_scan_and_move \
  --output-dir robot_sim_runs \
  --timeout 180
```

2D 纠偏干运行：

```bash
ros2 run robot_sim_bringup run_case \
  --case weld_2d_lateral_correction_dry_run \
  --output-dir robot_sim_runs \
  --timeout 120
```

报告中的“External Module”章节会展示 adapter 状态、服务调用结果、topic 采样、失败原因和日志路径。结构化字段在 `metrics.json` 中：

| 字段 | 含义 |
| --- | --- |
| `adapter_health` | 每个 adapter 的类型、状态和日志 |
| `module_services` | 服务等待和服务调用结果 |
| `module_topics` | 关键 topic 的采样次数、最后一条消息和断言结果 |
| `module_events` | 外部 launch/command 启动、服务等待等事件 |
| `module_failures` | 外部模块验收失败原因 |
| `adapter_data_sources` | adapter 数据源摘要，例如 `/scan_3d` 使用的图片、点云来源和帧策略 |

## 写自己的模块用例

1. 明确模块需要的 ROS 接口：启动方式、输入 topic/service/action、输出 topic/service/action。
2. 能由仿真直接提供的接口，优先接 Gazebo、TF、MoveIt 或传感器 topic。
3. 现场硬件接口用 adapter 补齐，adapter 必须声明类型、topic/service 名称和数据源。
4. 在 `module.wait_services` 中列出启动后必须出现的服务。
5. 在 `module.actions` 中列出验收动作；服务返回建议用 `expect.success_field` 检查。
6. 在 `expect.module.topics` 中写状态机、误差、目标点和关键输出断言。
7. 把关键 topic 加到 `artifacts.rosbag.extra_topics`，方便失败后复盘。

topic 断言支持：

```yaml
expect:
  data:
    equals: RUNNING
  error:
    abs_max: 0.02
  header.frame_id:
    contains: world
  x:
    exists: true
```

数值断言支持 `min`、`max`、`abs_max`；字符串支持 `equals` 和 `contains`。
