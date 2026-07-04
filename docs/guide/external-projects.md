# 外部项目资产规范

外部 ROS package 可以提供 robot_sim 资产，不需要改核心 runner。推荐目录：

```text
share/<pkg>/robot_sim/
  profiles/
  validation_cases/
  suites/
  data_sources/
  adapters/
```

## 推荐模型

新项目优先使用 `schema: 4`：

- `profiles/`：放 `kind: system_profile`，描述被测 ROS2 pipeline 的进程和环境变量。
- `validation_cases/`：放 `kind: validation_case`，描述输入、adapter、action、assertion 和 artifact。
- `suites/`：放 `kind: validation_suite`，组合多个 case，可带参数矩阵。
- `data_sources/`：放 topic/service/image/video/rosbag 数据源。
- `adapters/`：放可复用 adapter 模板，case 中用 `type: adapter_ref` 引用。
- 复杂业务判断放外部 evaluator 命令，核心只解析 evaluator JSON 结果并汇总报告。

机器人项目仍可在 `profiles/` 中放 v3 `sim_profile`。`validation_suites/` 和 `system_profiles/` 仍被兼容搜索，但新资产不要继续写旧目录。

## 脚手架

```bash
robot-sim scaffold-system --package my_robot_sim --name minimal_system --output /tmp
robot-sim scaffold-case --package my_robot_sim --name smoke_case --system minimal_system --output /tmp
robot-sim scaffold-suite --package my_robot_sim --name smoke_suite --case smoke_case --output /tmp
robot-sim scaffold-adapter --package my_robot_sim --name smoke_adapter --output /tmp
```

生成的 package 会包含 `package.xml`、`CMakeLists.txt` 和标准 `robot_sim/` 目录，可以直接由 colcon 安装。

## 运行

```bash
ros2 run robot_sim_bringup run_case \
  --case-package my_robot_sim \
  --case smoke_case \
  --output-dir robot_sim_runs \
  --no-rosbag

ros2 run robot_sim_bringup run_suite \
  --suite-package my_robot_sim \
  --suite smoke_suite \
  --output-dir robot_sim_runs \
  --no-rosbag
```

也可以传入直接 YAML 路径作为 escape hatch。维护新项目时优先只改外部 package 里的 YAML、fixture 数据和少量 adapter 模板。

## 外部 Evaluator

简单 topic/service/TF/process 检查用 `assertions`。复杂判断，例如视觉精度、定位误差、焊缝纠偏理论值或移动机器人轨迹质量，放到外部 evaluator：

```yaml
evaluators:
  - name: weld_correction_oracle
    type: command
    command:
      - python3
      - -m
      - my_project_validation.evaluators.weld_correction
      - --metrics
      - ${metrics_path}
      - --output
      - ${evaluator_output}
    output: evaluators/weld_correction_oracle.json
    timeout_sec: 30.0
    required: true
```

支持的占位符：

```text
${run_dir}
${logs_dir}
${rosbag_dir}
${metrics_path}
${manifest_path}
${effective_case_path}
${evaluator_output}
```

evaluator 必须写出 JSON object：

```json
{
  "passed": true,
  "summary": "within tolerance",
  "metrics": {
    "error_m": 0.001
  },
  "failures": [],
  "artifacts": []
}
```

`required: true` 的 evaluator 失败会让 case 失败；`required: false` 只进入报告。核心不理解领域指标，只记录 `summary`、`metrics`、`failures`、`artifacts` 和日志路径。
