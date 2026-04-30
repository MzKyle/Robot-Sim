#!/usr/bin/env bash

set -u
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-${WORKSPACE_ROOT}/install/setup.bash}"
LOG_DIR="${LOG_DIR:-${WORKSPACE_ROOT}/log/data_collect_startup}"

DEFAULT_NODEMANAGE_YAML="${WORKSPACE_ROOT}/src/config/nodemanage.yaml"
AUTOCOVER_NODEMANAGE_YAML="${AUTOCOVER_NODEMANAGE_YAML:-}"

RVC_LIB_DIR="${RVC_LIB_DIR:-/opt/RVC/lib}"
FANUC_SO_PATH="${FANUC_SO_PATH:-${WORKSPACE_ROOT}/src/fanuc_robot/lib/libFanucRobot.so}"

ENABLE_FANUC="${ENABLE_FANUC:-1}"
ENABLE_CAMERA_3D="${ENABLE_CAMERA_3D:-1}"
ENABLE_CAMERA_2D="${ENABLE_CAMERA_2D:-1}"
ENABLE_DATA_COLLECT="${ENABLE_DATA_COLLECT:-1}"
AUTO_START_FIX_SCAN="${AUTO_START_FIX_SCAN:-1}"
AUTO_ACTIVATE_COLLECT="${AUTO_ACTIVATE_COLLECT:-1}"
PUBLISH_TF="${PUBLISH_TF:-true}"

FANUC_ROBOT_IP="${FANUC_ROBOT_IP:-}"
FANUC_ROBOT_PORT="${FANUC_ROBOT_PORT:-}"

STARTUP_SETTLE_SEC="${STARTUP_SETTLE_SEC:-2}"
SERVICE_WAIT_SEC="${SERVICE_WAIT_SEC:-20}"

declare -a NODE_PIDS=()
declare -a NODE_NAMES=()
CLEANUP_RUNNING=0

info() {
    printf '[INFO] %s\n' "$*"
}

warn() {
    printf '[WARN] %s\n' "$*" >&2
}

error() {
    printf '[ERROR] %s\n' "$*" >&2
}

usage() {
    cat <<EOF
Usage:
  $(basename "$0")

Optional environment overrides:
  AUTOCOVER_NODEMANAGE_YAML=/path/to/nodemanage.yaml
  FANUC_SO_PATH=/path/to/libFanucRobot.so
  FANUC_ROBOT_IP=10.16.141.114
  FANUC_ROBOT_PORT=60008
  ENABLE_FANUC=0
  ENABLE_CAMERA_3D=0
  ENABLE_CAMERA_2D=0
  ENABLE_DATA_COLLECT=0
  AUTO_START_FIX_SCAN=0
  AUTO_ACTIVATE_COLLECT=0
  PUBLISH_TF=false
  LOG_DIR=${WORKSPACE_ROOT}/log/data_collect_startup
EOF
}

ensure_file_exists() {
    local file_path="$1"
    local description="$2"

    if [ ! -f "$file_path" ]; then
        error "${description} not found: ${file_path}"
        exit 1
    fi
}

setup_env() {
    ensure_file_exists "$ROS_SETUP" "ROS setup"
    ensure_file_exists "$WORKSPACE_SETUP" "Workspace setup"

    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    # shellcheck disable=SC1090
    source "$WORKSPACE_SETUP"

    if ! command -v ros2 >/dev/null 2>&1; then
        error "ros2 command is unavailable after sourcing the environment."
        exit 1
    fi

    if [ -z "$AUTOCOVER_NODEMANAGE_YAML" ]; then
        if [ -f /etc/WR/Project/nodemanage.yaml ]; then
            AUTOCOVER_NODEMANAGE_YAML="/etc/WR/Project/nodemanage.yaml"
        elif [ -f "$DEFAULT_NODEMANAGE_YAML" ]; then
            AUTOCOVER_NODEMANAGE_YAML="$DEFAULT_NODEMANAGE_YAML"
            warn "Using workspace nodemanage yaml: ${AUTOCOVER_NODEMANAGE_YAML}"
        else
            error "nodemanage yaml was not found in /etc/WR/Project or ${DEFAULT_NODEMANAGE_YAML}"
            exit 1
        fi
    fi

    export AUTOCOVER_NODEMANAGE_YAML

    if [ -d "$RVC_LIB_DIR" ]; then
        export LD_LIBRARY_PATH="${RVC_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    else
        warn "RVC SDK directory not found: ${RVC_LIB_DIR}"
    fi

    mkdir -p "$LOG_DIR"
    cd "$WORKSPACE_ROOT" || exit 1

    info "Workspace root: ${WORKSPACE_ROOT}"
    info "Log directory: ${LOG_DIR}"
    info "nodemanage yaml: ${AUTOCOVER_NODEMANAGE_YAML}"
}

start_node() {
    local display_name="$1"
    shift

    local log_file="${LOG_DIR}/${display_name}.log"

    info "Starting ${display_name}"
    "$@" >"$log_file" 2>&1 &
    local pid=$!

    NODE_PIDS+=("$pid")
    NODE_NAMES+=("$display_name")

    sleep "$STARTUP_SETTLE_SEC"

    if kill -0 "$pid" >/dev/null 2>&1; then
        info "${display_name} is running (pid=${pid}, log=${log_file})"
        return 0
    fi

    error "${display_name} exited during startup. See ${log_file}"
    tail -n 20 "$log_file" 2>/dev/null || true
    stop_all_nodes
    exit 1
}

wait_for_service() {
    local service_name="$1"
    local timeout_sec="$2"
    local deadline=$((SECONDS + timeout_sec))

    while [ "$SECONDS" -lt "$deadline" ]; do
        if ros2 service list 2>/dev/null | grep -Fxq "$service_name"; then
            return 0
        fi
        sleep 1
    done

    return 1
}

call_empty_service() {
    local service_name="$1"
    local timeout_sec="$2"
    local log_name="${service_name#/}"
    log_name="${log_name//\//_}"
    local log_file="${LOG_DIR}/${log_name}.call.log"

    if ! wait_for_service "$service_name" "$timeout_sec"; then
        error "Timed out waiting for service ${service_name}"
        return 1
    fi

    info "Calling ${service_name}"
    if timeout 10 ros2 service call "$service_name" std_srvs/srv/Empty "{}" >"$log_file" 2>&1; then
        info "${service_name} completed"
        return 0
    fi

    error "${service_name} failed. See ${log_file}"
    return 1
}

stop_all_nodes() {
    if [ "$CLEANUP_RUNNING" -eq 1 ]; then
        return
    fi
    CLEANUP_RUNNING=1

    info "Stopping data collection stack"

    if [ "$AUTO_ACTIVATE_COLLECT" = "1" ] && [ "$ENABLE_DATA_COLLECT" = "1" ]; then
        call_empty_service "/data_collect_deactivate" 5 || warn "Failed to deactivate data collection cleanly."
    fi

    if [ "$AUTO_START_FIX_SCAN" = "1" ] && [ "$ENABLE_CAMERA_3D" = "1" ]; then
        call_empty_service "/stop_fix_scan" 5 || warn "Failed to stop fixed scan cleanly."
    fi

    local idx
    for ((idx=${#NODE_PIDS[@]}-1; idx>=0; idx--)); do
        local pid="${NODE_PIDS[$idx]}"
        local name="${NODE_NAMES[$idx]}"

        if kill -0 "$pid" >/dev/null 2>&1; then
            kill "$pid" >/dev/null 2>&1 || true
            wait "$pid" 2>/dev/null || true
            info "Stopped ${name} (pid=${pid})"
        fi
    done
}

monitor_nodes() {
    while true; do
        local idx
        for idx in "${!NODE_PIDS[@]}"; do
            if ! kill -0 "${NODE_PIDS[$idx]}" >/dev/null 2>&1; then
                error "${NODE_NAMES[$idx]} exited unexpectedly."
                stop_all_nodes
                exit 1
            fi
        done
        sleep 2
    done
}

start_fanuc_node() {
    ensure_file_exists "$FANUC_SO_PATH" "Fanuc SDK"

    local -a cmd=(
        ros2 run fanuc_robot fanuc_robot
        --ros-args
        -p "so_file_path:=${FANUC_SO_PATH}"
    )

    if [ -f "$AUTOCOVER_NODEMANAGE_YAML" ]; then
        cmd+=(--params-file "$AUTOCOVER_NODEMANAGE_YAML")
    fi
    if [ -n "$FANUC_ROBOT_IP" ]; then
        cmd+=(-p "robot_ip:=${FANUC_ROBOT_IP}")
    fi
    if [ -n "$FANUC_ROBOT_PORT" ]; then
        cmd+=(-p "robot_port:=${FANUC_ROBOT_PORT}")
    fi

    start_node "fanuc_robot" "${cmd[@]}"
}

start_camera_3d_node() {
    local -a cmd=(
        ros2 run camera_3d_driver camera_3d_driver
        --ros-args
        -p "publish_tf:=${PUBLISH_TF}"
    )

    start_node "camera_3d_driver" "${cmd[@]}"
}

start_camera_2d_node() {
    start_node "camera_pool_driver" ros2 run camera_pool_driver camera_pool_driver
}

start_data_collect_node() {
    start_node "data_collect_node" ros2 run data_collect data_collect_node
}

main() {
    if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
        usage
        exit 0
    fi

    setup_env
    trap stop_all_nodes SIGINT SIGTERM

    if [ "$ENABLE_FANUC" = "1" ]; then
        start_fanuc_node
    else
        warn "Skipping fanuc_robot. /tool_pos and /fanuc_robot_info will not be available."
    fi

    if [ "$ENABLE_CAMERA_3D" = "1" ]; then
        start_camera_3d_node
    fi

    if [ "$ENABLE_CAMERA_2D" = "1" ]; then
        start_camera_2d_node
    fi

    if [ "$ENABLE_DATA_COLLECT" = "1" ]; then
        start_data_collect_node
    fi

    if [ "$AUTO_START_FIX_SCAN" = "1" ] && [ "$ENABLE_CAMERA_3D" = "1" ]; then
        call_empty_service "/start_fix_scan" "$SERVICE_WAIT_SEC" || {
            stop_all_nodes
            exit 1
        }
    fi

    if [ "$AUTO_ACTIVATE_COLLECT" = "1" ] && [ "$ENABLE_DATA_COLLECT" = "1" ]; then
        call_empty_service "/data_collect_activate" "$SERVICE_WAIT_SEC" || {
            stop_all_nodes
            exit 1
        }
    fi

    info "Data collection stack is ready. Press Ctrl+C to stop."
    monitor_nodes
}

main "$@"
