#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PACKAGE_NAME="${PACKAGE_NAME:-weld-data-collect}"
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
INSTALL_PREFIX="/opt/weld_data_collect"
DEB_OUT_DIR="${WORKSPACE_ROOT}/dist"
INCLUDE_CAMERA_DRIVERS="${INCLUDE_CAMERA_DRIVERS:-1}"
INCLUDE_FANUC_VENDOR_LIB="${INCLUDE_FANUC_VENDOR_LIB:-1}"

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
mkdir -p "${PKG_ROOT}${INSTALL_PREFIX}" "${PKG_ROOT}/usr/bin" "${PKG_ROOT}/etc/weld_data_collect" "${PKG_ROOT}/DEBIAN" "${DEB_OUT_DIR}"

PACKAGES=(
    weld_interface
    file_reader
    fanuc_robot
    data_collect
    data_collect_quality
    data_collect_cloud_renderer
    data_collect_bringup
    data_collect_ui
)

if [ "${INCLUDE_CAMERA_DRIVERS}" = "1" ]; then
    PACKAGES+=(camera_pool_driver camera_3d_driver)
fi

echo "Building packages: ${PACKAGES[*]}"
colcon --log-base "${COLCON_LOG_BASE}" build \
    --merge-install \
    --build-base "${COLCON_BUILD_BASE}" \
    --install-base "${PKG_ROOT}${INSTALL_PREFIX}" \
    --packages-select "${PACKAGES[@]}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ "${INCLUDE_FANUC_VENDOR_LIB}" = "1" ]; then
    mkdir -p "${PKG_ROOT}${INSTALL_PREFIX}/vendor/fanuc/lib"
    cp -a "${WORKSPACE_ROOT}/src/fanuc_robot/lib/." "${PKG_ROOT}${INSTALL_PREFIX}/vendor/fanuc/lib/"
fi

cat >"${PKG_ROOT}/etc/weld_data_collect/nodemanage.yaml" <<EOF
camera_driver_3d:
  ros__parameters:
    cfg: '${INSTALL_PREFIX}/share/camera_3d_driver/config/cameratcp.yaml'
    publish_tf: true

camera_node:
  ros__parameters:
    trigger_mode: 2
    strobe_polarity: 0
    saturation: 64
    gamma: 106
    exposure_time: 4.3
    analog_gain: 64
    frame_rate: 60.0

robot_driver_fanuc:
  ros__parameters:
    so_file_path: '${INSTALL_PREFIX}/vendor/fanuc/lib/libFanucRobot.so'
    robot_ip: '10.16.140.114'
    robot_port: 60008
    target_register_index: 100

data_collect_node:
  ros__parameters:
    save_dir_root: '/var/lib/weld_data_collect/data'
    image_save_interval: 12
    image_log_save_interval: 3
    height_log_save_interval: 4
    fix_scan_interval: 6
    auto_save_flag: 0
    target_register_index: 100
EOF

cat >"${PKG_ROOT}/usr/bin/weld-data-collect" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
INSTALL_PREFIX="/opt/weld_data_collect"
NODEMANAGE_YAML="${AUTOCOVER_NODEMANAGE_YAML:-/etc/weld_data_collect/nodemanage.yaml}"
DEFAULT_FANUC_SO_PATH="${INSTALL_PREFIX}/vendor/fanuc/lib/libFanucRobot.so"
FANUC_SO_PATH="${FANUC_SO_PATH:-}"
RVC_LIB_DIR="${RVC_LIB_DIR:-/opt/RVC/lib}"
FANUC_LIB_DIR="$(dirname "${FANUC_SO_PATH:-${DEFAULT_FANUC_SO_PATH}}")"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${INSTALL_PREFIX}/setup.bash"
set -u

export AUTOCOVER_NODEMANAGE_YAML="${NODEMANAGE_YAML}"
if [ -d "${FANUC_LIB_DIR}" ]; then
    export LD_LIBRARY_PATH="${FANUC_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
if [ -d "${RVC_LIB_DIR}" ]; then
    export LD_LIBRARY_PATH="${RVC_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

launch_args=(
    "nodemanage_yaml:=${NODEMANAGE_YAML}"
    "rvc_lib_dir:=${RVC_LIB_DIR}"
)
if [ -n "${FANUC_SO_PATH}" ]; then
    launch_args+=("fanuc_so_path:=${FANUC_SO_PATH}")
fi

exec ros2 launch data_collect_bringup data_collect.launch.py "${launch_args[@]}" "$@"
EOF

cat >"${PKG_ROOT}/usr/bin/weld-data-collect-ui" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
INSTALL_PREFIX="/opt/weld_data_collect"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${INSTALL_PREFIX}/setup.bash"
set -u

exec ros2 run data_collect_ui data_collect_ui "$@"
EOF

cat >"${PKG_ROOT}/usr/bin/weld-data-collect-check" <<'EOF'
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
check_file /opt/weld_data_collect/setup.bash "weld_data_collect workspace"
check_file /etc/weld_data_collect/nodemanage.yaml "nodemanage config"
check_file /opt/weld_data_collect/vendor/fanuc/lib/libFanucRobot.so "Fanuc vendor library"
if [ -x /opt/weld_data_collect/lib/camera_3d_driver/camera_3d_driver ]; then
    check_file /opt/RVC/lib/libRVC.so "RVC SDK"
else
    echo "[SKIP] RVC SDK: camera_3d_driver is not installed"
fi
if [ -x /opt/weld_data_collect/lib/camera_pool_driver/camera_pool_driver ]; then
    check_file /lib/libMVSDK.so "MVSDK"
else
    echo "[SKIP] MVSDK: camera_pool_driver is not installed"
fi
check_command ros2 "ros2 command"

if python3 - <<'PY'
try:
    import PySide6
    print("[OK] PySide6")
except Exception as exc:
    try:
        import PyQt5
        print("[OK] PyQt5")
    except Exception as pyqt_exc:
        print("[MISSING] Qt Python binding: PySide6:", exc)
        print("[MISSING] Qt Python binding: PyQt5:", pyqt_exc)
        raise SystemExit(1)
PY
then
    :
else
    ok=0
fi

if python3 - <<'PY'
import yaml
print("[OK] PyYAML")
PY
then
    :
else
    echo "[MISSING] PyYAML"
    ok=0
fi

if [ -f /opt/ros/humble/setup.bash ] && [ -f /opt/weld_data_collect/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    source /opt/weld_data_collect/setup.bash
    set -u

    if [ -d /opt/weld_data_collect/vendor/fanuc/lib ]; then
        export LD_LIBRARY_PATH="/opt/weld_data_collect/vendor/fanuc/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    fi
    if [ -d /opt/RVC/lib ]; then
        export LD_LIBRARY_PATH="/opt/RVC/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    fi

    for binary in \
        /opt/weld_data_collect/lib/data_collect/data_collect_node \
        /opt/weld_data_collect/lib/fanuc_robot/fanuc_robot \
        /opt/weld_data_collect/lib/camera_pool_driver/camera_pool_driver \
        /opt/weld_data_collect/lib/camera_3d_driver/camera_3d_driver
    do
        if [ ! -x "$binary" ]; then
            continue
        fi
        ldd_output="$(ldd "$binary" 2>/dev/null || true)"
        missing="$(printf '%s\n' "$ldd_output" | awk '/not found/{print "  " $1 " => not found"}')"
        if [ -n "$missing" ]; then
            echo "[MISSING] runtime libraries for $binary"
            printf '%s\n' "$missing"
            ok=0
        else
            echo "[OK] runtime libraries: $binary"
        fi
        if printf '%s\n' "$ldd_output" | grep -q '/usr/local/lib/libopencv'; then
            echo "[WARN] $binary links to /usr/local OpenCV; production debs should be built in a clean ROS environment."
        fi
    done
fi

exit "$ok"
EOF

chmod 0755 \
    "${PKG_ROOT}/usr/bin/weld-data-collect" \
    "${PKG_ROOT}/usr/bin/weld-data-collect-ui" \
    "${PKG_ROOT}/usr/bin/weld-data-collect-check"

mkdir -p "${PKG_ROOT}/var/lib/weld_data_collect/data"

cat >"${PKG_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE_NAME}
Version: ${PACKAGE_VERSION}-${PACKAGE_REVISION}
Section: robotics
Priority: optional
Architecture: ${ARCH}
Maintainer: SANY Weld Data Collect <maintainer@example.com>
Depends: bash, python3, python3-pyqt5, python3-yaml, ros-${ROS_DISTRO}-rclcpp, ros-${ROS_DISTRO}-rclpy, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-std-srvs, ros-${ROS_DISTRO}-sensor-msgs, ros-${ROS_DISTRO}-cv-bridge, ros-${ROS_DISTRO}-pcl-ros, ros-${ROS_DISTRO}-pcl-conversions, ros-${ROS_DISTRO}-tf2, ros-${ROS_DISTRO}-tf2-ros, ros-${ROS_DISTRO}-tf2-eigen, ros-${ROS_DISTRO}-tf2-geometry-msgs, ros-${ROS_DISTRO}-launch, ros-${ROS_DISTRO}-launch-ros, libyaml-cpp0.7
Recommends: python3-pip
Description: Weld robot data collection stack
 Standalone ROS 2 Humble data collection software for 2D camera, 3D scan,
 Fanuc robot status, collection metadata, desktop operation UI, and history
 data management.
EOF

cat >"${PKG_ROOT}/DEBIAN/conffiles" <<EOF
/etc/weld_data_collect/nodemanage.yaml
EOF

cat >"${PKG_ROOT}/DEBIAN/postinst" <<'EOF'
#!/usr/bin/env bash
set -e

mkdir -p /var/lib/weld_data_collect/data

if getent group sudo >/dev/null 2>&1; then
    chgrp sudo /etc/weld_data_collect/nodemanage.yaml || true
    chmod 0664 /etc/weld_data_collect/nodemanage.yaml || true
fi

cat <<'MSG'
weld-data-collect installed.

Useful commands:
  weld-data-collect-check
  weld-data-collect enable_camera_3d:=false enable_camera_2d:=false
  weld-data-collect-ui

Hardware SDK notes:
  - RVC SDK is expected at /opt/RVC.
  - MVSDK is expected to provide /lib/libMVSDK.so.
  - Fanuc libFanucRobot.so is installed under /opt/weld_data_collect/vendor/fanuc/lib.
MSG
EOF

chmod 0755 "${PKG_ROOT}/DEBIAN/postinst"

find "${PKG_ROOT}" -type d -name __pycache__ -prune -exec rm -rf {} +
find "${PKG_ROOT}" -type d -exec chmod 0755 {} +
find "${PKG_ROOT}" -type f -exec chmod 0644 {} +
find "${PKG_ROOT}${INSTALL_PREFIX}/lib" -mindepth 2 -maxdepth 2 -type f -exec chmod 0755 {} +
chmod 0755 \
    "${PKG_ROOT}/usr/bin/weld-data-collect" \
    "${PKG_ROOT}/usr/bin/weld-data-collect-ui" \
    "${PKG_ROOT}/usr/bin/weld-data-collect-check" \
    "${PKG_ROOT}/DEBIAN/postinst"

if find "${PKG_ROOT}${INSTALL_PREFIX}/lib" -mindepth 2 -maxdepth 2 -type f -perm /111 -print0 \
    | xargs -0r sh -c 'for binary do ldd "$binary" 2>/dev/null || true; done' sh \
    | grep -q '/usr/local/lib/libopencv'; then
    cat >&2 <<'EOF'
WARNING: Built binaries link to /usr/local/lib/libopencv*.so.
For a portable deb, build on a clean ROS 2 Humble machine without a custom
/usr/local OpenCV, or make sure target machines have the same OpenCV libraries.
EOF
fi

DEB_PATH="${DEB_OUT_DIR}/${PACKAGE_NAME}_${PACKAGE_VERSION}-${PACKAGE_REVISION}_${ARCH}.deb"
fakeroot dpkg-deb --build "${PKG_ROOT}" "${DEB_PATH}"

echo "Built: ${DEB_PATH}"
