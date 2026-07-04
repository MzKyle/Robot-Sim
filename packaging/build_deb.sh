#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PACKAGE_NAME="${PACKAGE_NAME:-robot-sim}"
PACKAGE_VERSION="${PACKAGE_VERSION:-0.1.0}"
PACKAGE_REVISION="${PACKAGE_REVISION:-1}"
ARCH="${ARCH:-$(dpkg --print-architecture)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
BUILD_DIR="${BUILD_DIR:-${WORKSPACE_ROOT}/build_deb}"
STAGING_DIR="${BUILD_DIR}/${PACKAGE_NAME}_${PACKAGE_VERSION}-${PACKAGE_REVISION}_${ARCH}"
PKG_ROOT="${STAGING_DIR}/root"
COLCON_BUILD_BASE="${BUILD_DIR}/colcon_build"
COLCON_LOG_BASE="${BUILD_DIR}/colcon_log"
INSTALL_PREFIX="/opt/robot_sim"
DEB_OUT_DIR="${WORKSPACE_ROOT}/dist"

if [ ! -f "${ROS_SETUP}" ]; then
    echo "ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
fi

if ! command -v dpkg-deb >/dev/null 2>&1; then
    echo "dpkg-deb is required." >&2
    exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
    echo "colcon is required." >&2
    exit 1
fi

set +u
source "${ROS_SETUP}"
set -u

rm -rf "${STAGING_DIR}" "${COLCON_BUILD_BASE}" "${COLCON_LOG_BASE}"
mkdir -p "${PKG_ROOT}${INSTALL_PREFIX}" "${PKG_ROOT}/usr/bin" "${PKG_ROOT}/DEBIAN" "${DEB_OUT_DIR}"

PACKAGES=(
    gz_ros2_control
    robot_sim_description
    robot_sim_control
    robot_sim_scenarios
    robot_sim_moveit_config
    robot_sim_sensor_camera
    robot_sim_sensor_depth
    robot_sim_sensor_lidar
    robot_sim_sensor_imu
    robot_sim_bringup
    robot_task_interfaces
    simulation_interfaces
)

echo "Building packages: ${PACKAGES[*]}"
colcon --log-base "${COLCON_LOG_BASE}" build \
    --merge-install \
    --build-base "${COLCON_BUILD_BASE}" \
    --install-base "${PKG_ROOT}${INSTALL_PREFIX}" \
    --packages-select "${PACKAGES[@]}" \
    --allow-overriding gz_ros2_control \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

cat >"${PKG_ROOT}/usr/bin/robot-sim" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
INSTALL_PREFIX="/opt/robot_sim"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${INSTALL_PREFIX}/setup.bash"
set -u

if [[ "${1:-}" == "run-case" ]]; then
  shift
  exec ros2 run robot_sim_bringup run_case "$@"
fi
if [[ "${1:-}" == "run-suite" ]]; then
  shift
  exec ros2 run robot_sim_bringup run_suite "$@"
fi
if [[ "${1:-}" == "migrate-config" ]]; then
  shift
  exec ros2 run robot_sim_bringup migrate_config "$@"
fi
if [[ "${1:-}" == "scaffold-robot" ]]; then
  shift
  exec ros2 run robot_sim_bringup scaffold_robot "$@"
fi
if [[ "${1:-}" == "scaffold-system" ]]; then
  shift
  exec ros2 run robot_sim_bringup scaffold_system "$@"
fi
if [[ "${1:-}" == "scaffold-case" ]]; then
  shift
  exec ros2 run robot_sim_bringup scaffold_case "$@"
fi
if [[ "${1:-}" == "scaffold-suite" ]]; then
  shift
  exec ros2 run robot_sim_bringup scaffold_suite "$@"
fi
if [[ "${1:-}" == "scaffold-adapter" ]]; then
  shift
  exec ros2 run robot_sim_bringup scaffold_adapter "$@"
fi

exec ros2 launch robot_sim_bringup sim.launch.py "$@"
EOF

cat >"${PKG_ROOT}/usr/bin/robot-sim-check" <<'EOF'
#!/usr/bin/env bash
set -u

ok=1

check_file() {
    local path="$1"
    local name="$2"
    if [ -e "$path" ]; then
        echo "[OK] ${name}: ${path}"
    else
        echo "[MISSING] ${name}: ${path}"
        ok=0
    fi
}

check_command() {
    local command="$1"
    local name="$2"
    if command -v "$command" >/dev/null 2>&1; then
        echo "[OK] ${name}: $(command -v "$command")"
    else
        echo "[MISSING] ${name}: ${command}"
        ok=0
    fi
}

check_file /opt/ros/humble/setup.bash "ROS 2 Humble"
check_file /opt/robot_sim/setup.bash "robot_sim workspace"
check_command ros2 "ros2 command"

exit "$ok"
EOF

chmod 0755 \
    "${PKG_ROOT}/usr/bin/robot-sim" \
    "${PKG_ROOT}/usr/bin/robot-sim-check"

cat >"${PKG_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE_NAME}
Version: ${PACKAGE_VERSION}-${PACKAGE_REVISION}
Section: robotics
Priority: optional
Architecture: ${ARCH}
Maintainer: MzKyle <19862681939@163.com>
Depends: bash, python3, python3-jsonschema, python3-yaml, gz-harmonic, ros-${ROS_DISTRO}-rclcpp, ros-${ROS_DISTRO}-rclpy, ros-${ROS_DISTRO}-sensor-msgs, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-trajectory-msgs, ros-${ROS_DISTRO}-control-msgs, ros-${ROS_DISTRO}-controller-manager, ros-${ROS_DISTRO}-joint-state-broadcaster, ros-${ROS_DISTRO}-joint-trajectory-controller, ros-${ROS_DISTRO}-robot-state-publisher, ros-${ROS_DISTRO}-ros-gzharmonic, ros-${ROS_DISTRO}-moveit, ros-${ROS_DISTRO}-rviz2, ros-${ROS_DISTRO}-xacro
Description: Generic ROS 2 robot simulation stack
 Gazebo and MoveIt2 simulation workspace with Panda and Fanuc M20iD/12L
 profiles, reusable robot assets, scenarios, and simulated sensor receivers.
EOF

cat >"${PKG_ROOT}/DEBIAN/postinst" <<'EOF'
#!/usr/bin/env bash
set -e

cat <<'MSG'
robot-sim installed.

Useful commands:
  robot-sim-check
  robot-sim run-case --case industrial_fixture_to_pallet
  robot-sim migrate-config --input old.yaml --output new.yaml
  robot-sim scaffold-robot --package my_robot_sim --robot-name my_robot --output /tmp --joint-names joint_1 joint_2 joint_3 joint_4 joint_5 joint_6
  robot-sim scaffold-system --package my_robot_sim --name minimal_system --output /tmp
  robot-sim scaffold-case --package my_robot_sim --name smoke_case --system minimal_system --output /tmp
  robot-sim scaffold-suite --package my_robot_sim --name smoke_suite --case smoke_case --output /tmp
  robot-sim scaffold-adapter --package my_robot_sim --name smoke_adapter --output /tmp
  robot-sim sim_profile:=panda sim_mode:=light
  robot-sim sim_profile:=fanuc_m20id12l sim_mode:=full
MSG
EOF

chmod 0755 "${PKG_ROOT}/DEBIAN/postinst"

find "${PKG_ROOT}" -type d -name __pycache__ -prune -exec rm -rf {} +
find "${PKG_ROOT}" -type d -exec chmod 0755 {} +
find "${PKG_ROOT}" -type f -exec chmod 0644 {} +
find "${PKG_ROOT}${INSTALL_PREFIX}/lib" -mindepth 2 -maxdepth 2 -type f -exec chmod 0755 {} +
chmod 0755 \
    "${PKG_ROOT}/usr/bin/robot-sim" \
    "${PKG_ROOT}/usr/bin/robot-sim-check" \
    "${PKG_ROOT}/DEBIAN/postinst"

DEB_PATH="${DEB_OUT_DIR}/${PACKAGE_NAME}_${PACKAGE_VERSION}-${PACKAGE_REVISION}_${ARCH}.deb"
fakeroot dpkg-deb --build "${PKG_ROOT}" "${DEB_PATH}"

echo "Built: ${DEB_PATH}"
