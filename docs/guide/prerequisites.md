# 环境依赖与准备

## 基础环境

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- OpenCV、PCL、cv_bridge、ros_gz_bridge、ros_gz_sim 等 ROS 依赖

## 桌面界面依赖

```bash
sudo apt install python3-pyqt5 python3-yaml
```

界面优先使用 PySide6；如果系统没有 PySide6，会自动尝试使用 PyQt5。生产安装建议使用 apt 安装 PyQt5，便于把依赖闭合到系统包管理中。

## 硬件与 SDK 依赖

- 运行真实 3D 相机节点需要 RVC SDK。
- 运行真实 2D 相机节点需要对应 MVSDK。
- 运行真实 Fanuc 机器人节点需要 Fanuc 共享库及其依赖。
- 运行仿真链路需要 gz sim 8 可用，并且主机具备可用的图形渲染环境。
- ROS 2 Humble + gz sim 8/Harmonic 需要源码版 `gz_ros2_control` overlay；构建前设置 `GZ_VERSION=harmonic`。

## 建议准备项

- 可访问的机器人控制器 IP 和端口。
- 已安装的相机 SDK 和驱动库。
- 有写权限的数据保存目录。
- 如果优先做联调，建议先准备仿真链路，再逐步切到真实设备。
- 适合调试的终端环境，方便同时查看 ROS 日志和节点输出。
