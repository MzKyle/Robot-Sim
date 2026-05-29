# 环境依赖与准备

## 基础环境

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- OpenCV、PCL、cv_bridge、ros_gz_bridge、ros_gz_sim、rosbag2 等 ROS 依赖

## 桌面界面依赖

```bash
sudo apt install python3-pyqt5 python3-yaml
```

界面优先使用 PySide6；如果系统没有 PySide6，会自动尝试使用 PyQt5。生产安装建议使用 apt 安装 PyQt5，便于把依赖闭合到系统包管理中。

## 仿真依赖

- 运行仿真链路需要 gz sim 8 可用，并且主机具备可用的图形渲染环境。
- ROS 2 Humble + gz sim 8/Harmonic 需要源码版 `gz_ros2_control` overlay；构建前设置 `GZ_VERSION=harmonic`。
- 旧真实硬件驱动包已移除；旧 `data_collect` 硬件启动和 packaging 链路本轮暂不维护。

## 建议准备项

- 有写权限的数据保存目录。
- 建议先准备仿真链路和 `robot_sim_sensors` receiver，再逐步定义真实设备适配层。
- 适合调试的终端环境，方便同时查看 ROS 日志和节点输出。
