# 部署建议

## 推荐方式

- 开发环境使用源码工作空间。
- 发布环境使用 tag workflow 生成的 deb。
- Gazebo full smoke 建议在有足够 CPU/GPU 资源的 runner 或本机执行。

## 环境变量

| 变量 | 推荐值 | 说明 |
| --- | --- | --- |
| `ROS_DISTRO` | `humble` | ROS 发行版 |
| `GZ_VERSION` | `harmonic` | `gz_ros2_control` 构建 ABI |
| `GZ_SIM_RESOURCE_PATH` | 由 launch 注入 | Gazebo resource path |
| `GZ_SIM_SYSTEM_PLUGIN_PATH` | 由 smoke/launch 注入 | Gazebo system plugin path |

## 发布

推送 `vMAJOR.MINOR.PATCH` tag 后，GitHub Actions 会构建 deb 并上传 Release。安装后可用：

```bash
robot-sim-check
robot-sim sim_profile:=panda sim_mode:=light
```
