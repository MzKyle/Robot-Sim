# Debian 打包说明

`packaging/build_deb.sh` 只打包当前通用仿真主线，不再包含旧真实硬件、采集 UI 或厂商 SDK。

## 构建

```bash
cd /home/kyle/sany/robot_sim
bash packaging/build_deb.sh
```

输出位置：

```text
dist/robot-sim_<version>-<revision>_<arch>.deb
```

自定义版本：

```bash
PACKAGE_VERSION=0.2.0 PACKAGE_REVISION=1 bash packaging/build_deb.sh
```

## 安装和运行

```bash
sudo apt install ./dist/robot-sim_0.1.0-1_amd64.deb
robot-sim-check
robot-sim sim_profile:=panda sim_mode:=light
robot-sim sim_profile:=fanuc_m20id12l sim_mode:=full
```

当前 deb 安装到 `/opt/robot_sim`，提供 `robot-sim` 和 `robot-sim-check` 两个命令。
