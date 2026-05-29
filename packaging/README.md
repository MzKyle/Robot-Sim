# Debian 打包说明

> 注意：旧真实硬件驱动包已经移除，当前 `packaging/build_deb.sh` 仍包含旧相机/Fanuc 入口引用，本轮暂不维护。新仿真主线请优先使用 `robot_sim_bringup` 和 `robot_sim_sensors` 的源码构建流程。

## 推荐策略

主应用打成一个 deb：

- 安装 ROS 2 工作空间产物到 `/opt/weld_data_collect`。
- 安装默认配置到 `/etc/weld_data_collect/nodemanage.yaml`。
- 安装数据目录到 `/var/lib/weld_data_collect/data`。
- 安装命令：
  - `weld-data-collect`
  - `weld-data-collect-ui`
  - `weld-data-collect-check`
- 通过 `Depends` 安装 ROS 2、PCL、cv_bridge、yaml-cpp、PyQt5 等通用运行依赖。
- 界面 `参数设置` 页会修改 `/etc/weld_data_collect/nodemanage.yaml`，安装脚本会允许 `sudo` 组用户写入该配置文件。

旧底层硬件 SDK 不建议直接塞进主 deb：

- RVC SDK 约数百 MB，通常带授权、udev、网络/USB 调优脚本，适合单独安装或做单独 vendor deb。
- MVSDK 通常包含相机驱动、动态库、udev 规则和厂商授权，也适合单独安装或做单独 vendor deb。
- Fanuc `libFanucRobot.so` 已随旧驱动包移除；如后续恢复真实设备适配，建议独立做 vendor 包。
- PySide6 不放入主 deb；界面已兼容 PyQt5，主 deb 依赖 `python3-pyqt5`，便于 apt 自动安装。

## 构建

```bash
cd /home/kyle/sany/weld_data_collect_ws
bash packaging/build_deb.sh
```

输出位置：

```text
dist/weld-data-collect_<version>-<revision>_<arch>.deb
```

自定义版本：

```bash
PACKAGE_VERSION=0.2.0 PACKAGE_REVISION=1 bash packaging/build_deb.sh
```

旧脚本保留了相机驱动开关；该路径本轮未维护：

```bash
INCLUDE_CAMERA_DRIVERS=0 bash packaging/build_deb.sh
```

## 安装

```bash
sudo apt install ./dist/weld-data-collect_0.1.0-1_amd64.deb
weld-data-collect-check
```

启动后端：

```bash
weld-data-collect
```

启动界面：

```bash
weld-data-collect-ui
```

## 重要风险

如果构建时提示程序链接到了 `/usr/local/lib/libopencv*.so`，说明打包机有自定义 OpenCV。这个 deb 到目标机可能无法运行。推荐在干净的 Ubuntu 22.04 + ROS 2 Humble 环境中打包。
