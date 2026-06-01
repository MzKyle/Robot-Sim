# 安装与打包

当前打包脚本面向通用 Gazebo 仿真主线，安装 `robot_sim_*` 包、场景资源和仿真传感器 receiver。

## 打包命令

```bash
cd /home/kyle/sany/robot_sim
bash packaging/build_deb.sh
```

## 产物位置

```text
dist/robot-sim_0.1.0-1_amd64.deb
```

## 安装方式

```bash
sudo apt install ./dist/robot-sim_0.1.0-1_amd64.deb
robot-sim-check
```

## 安装后常用命令

```bash
robot-sim sim_profile:=panda sim_mode:=light
robot-sim sim_profile:=fanuc_m20id12l sim_mode:=full
```

## 打包注意事项

- 生产包建议在干净的 Ubuntu 22.04 + ROS 2 Humble 环境中重新构建。
- 旧真实硬件驱动和采集链路不再由本项目 deb 打包。
