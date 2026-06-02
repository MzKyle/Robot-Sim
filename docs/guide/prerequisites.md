# 环境依赖

## 基础环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic / `gz sim 8`
- MoveIt2
- `colcon`、`rosdep`、`git`

## 安装 Gazebo Harmonic

```bash
sudo apt-get update
sudo apt-get install -y curl gnupg lsb-release
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic libgz-sim8-dev libgz-plugin2-dev
```

## 安装 ROS 依赖

```bash
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-yaml \
  ros-humble-moveit \
  ros-humble-ros-gzharmonic \
  ros-humble-ros2-control \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-xacro \
  ros-humble-urdfdom-py
```

首次使用 rosdep：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update --rosdistro humble
```

## Gazebo ABI 约定

本项目固定使用 `GZ_VERSION=harmonic`。Humble 与 Gazebo Harmonic 组合需要构建仓库内的 `src/vendor/gz_ros2_control` overlay，并在构建时使用：

```bash
export GZ_VERSION=harmonic
colcon build --symlink-install --allow-overriding gz_ros2_control --packages-select gz_ros2_control
```
