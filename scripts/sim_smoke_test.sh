#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="panda"
PROFILE_FILE=""
MODE="full"
SENSOR_OVERRIDES=""
WITH_MOVEIT=false
WITH_ROSBAG=false
KEEP_SIM=false
KEEP_LOGS=false
TIMEOUT=120

usage() {
  cat <<'EOF'
Usage:
  scripts/sim_smoke_test.sh [options]

Options:
  --profile NAME              sim_profile name (default: panda)
  --profile-file PATH         external sim_profile YAML
  --mode full|light|mock      simulation mode (default: full)
  --sensor-overrides TEXT     e.g. camera=true,depth=false
  --with-moveit               run optional MoveIt plan/execute check
  --with-rosbag               run optional short rosbag record check
  --keep-sim                  leave launched simulation running
  --keep-logs                 keep logs after a successful run
  --timeout SECONDS           readiness/action timeout (default: 120)
  -h, --help                  show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:?--profile requires a value}"
      shift 2
      ;;
    --profile-file)
      PROFILE_FILE="${2:?--profile-file requires a value}"
      shift 2
      ;;
    --mode)
      MODE="${2:?--mode requires a value}"
      shift 2
      ;;
    --sensor-overrides)
      SENSOR_OVERRIDES="${2:?--sensor-overrides requires a value}"
      shift 2
      ;;
    --with-moveit)
      WITH_MOVEIT=true
      shift
      ;;
    --with-rosbag)
      WITH_ROSBAG=true
      shift
      ;;
    --keep-sim)
      KEEP_SIM=true
      shift
      ;;
    --keep-logs)
      KEEP_LOGS=true
      shift
      ;;
    --timeout)
      TIMEOUT="${2:?--timeout requires a value}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$MODE" in
  full|light|mock) ;;
  *)
    echo "--mode must be full, light, or mock; got '$MODE'" >&2
    exit 2
    ;;
esac

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source_setup /opt/ros/humble/setup.bash
fi
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  source_setup "$ROOT_DIR/install/setup.bash"
fi
if [[ -d "$ROOT_DIR/src/robot_sim_bringup" ]]; then
  export PYTHONPATH="$ROOT_DIR/src/robot_sim_bringup:${PYTHONPATH:-}"
fi
export GZ_VERSION="${GZ_VERSION:-harmonic}"

HELPER=(python3 -m robot_sim_bringup.sim_smoke_helper)
LINTER=(python3 -m robot_sim_bringup.profile_lint)
COMMON_ARGS=(--profile "$PROFILE" --mode "$MODE")
if [[ -n "$PROFILE_FILE" ]]; then
  COMMON_ARGS+=(--profile-file "$PROFILE_FILE")
fi
if [[ -n "$SENSOR_OVERRIDES" ]]; then
  COMMON_ARGS+=(--sensor-overrides "$SENSOR_OVERRIDES")
fi

preflight() {
  local missing=()
  for command in ros2 xacro check_urdf; do
    if ! command -v "$command" >/dev/null 2>&1; then
      missing+=("$command")
    fi
  done
  if [[ "$MODE" != "mock" ]] && ! command -v gz >/dev/null 2>&1; then
    missing+=("gz")
  fi
  if ((${#missing[@]} > 0)); then
    echo "Missing required commands: ${missing[*]}" >&2
    echo "Source ROS 2 and rebuild this workspace before running the smoke test." >&2
    exit 1
  fi
  if ! ros2 pkg prefix robot_sim_bringup >/dev/null 2>&1; then
    echo "robot_sim_bringup is not visible in the ROS environment." >&2
    echo "Run colcon build and source install/setup.bash first." >&2
    exit 1
  fi
  if ! "${HELPER[@]}" --help >/dev/null; then
    echo "Cannot import robot_sim_bringup.sim_smoke_helper." >&2
    echo "Run colcon build and source install/setup.bash first." >&2
    exit 1
  fi
  if ! "${LINTER[@]}" --help >/dev/null; then
    echo "Cannot import robot_sim_bringup.profile_lint." >&2
    echo "Run colcon build and source install/setup.bash first." >&2
    exit 1
  fi
}

LOG_DIR=""
URDF_FILE=""
SIM_PID=""
BAG_PID=""
FAILURE_SEEN=false

cleanup_process_group() {
  local pid="$1"
  local signal="${2:-TERM}"
  if [[ -n "$pid" ]] && kill -0 "$pid" >/dev/null 2>&1; then
    kill "-$signal" -- "-$pid" >/dev/null 2>&1 || true
    sleep 1
    kill -KILL -- "-$pid" >/dev/null 2>&1 || true
    wait "$pid" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  local exit_code=$?
  if [[ $exit_code -ne 0 ]]; then
    FAILURE_SEEN=true
  fi

  if [[ -n "$BAG_PID" ]]; then
    cleanup_process_group "$BAG_PID" INT
  fi
  if [[ "$KEEP_SIM" != true && -n "$SIM_PID" ]]; then
    cleanup_process_group "$SIM_PID" INT
  fi

  if [[ -n "$LOG_DIR" ]]; then
    if [[ "$KEEP_LOGS" == true || "$FAILURE_SEEN" == true || "$KEEP_SIM" == true ]]; then
      echo "Logs kept at: $LOG_DIR"
    else
      rm -rf "$LOG_DIR"
    fi
  fi
}
trap cleanup EXIT

run_step() {
  local label="$1"
  shift
  echo
  echo "==> $label"
  "$@"
}

wait_gazebo_model() {
  local model="$1"
  local deadline=$((SECONDS + TIMEOUT))
  while ((SECONDS < deadline)); do
    if ! kill -0 "$SIM_PID" >/dev/null 2>&1; then
      echo "Simulation process exited early. See $LOG_DIR/sim.launch.log" >&2
      return 1
    fi
    if gz model --list 2>/dev/null \
      | sed -E 's/^[[:space:]]*-[[:space:]]*//' \
      | grep -Fx "$model" >/dev/null; then
      echo "Gazebo model spawned: $model"
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for Gazebo model '$model'" >&2
  return 1
}

start_simulation() {
  local use_moveit="$1"
  local launch_args=(
    ros2 launch robot_sim_bringup sim.launch.py
    "sim_profile:=$PROFILE"
    "sim_mode:=$MODE"
    "headless:=true"
    "rviz:=false"
    "use_moveit:=$use_moveit"
  )
  if [[ -n "$PROFILE_FILE" ]]; then
    launch_args+=("sim_profile_file:=$PROFILE_FILE")
  fi
  if [[ -n "$SENSOR_OVERRIDES" ]]; then
    launch_args+=("sensor_overrides:=$SENSOR_OVERRIDES")
  fi

  echo "Launching simulation; log: $LOG_DIR/sim.launch.log"
  setsid "${launch_args[@]}" >"$LOG_DIR/sim.launch.log" 2>&1 &
  SIM_PID=$!
  sleep 2
  if ! kill -0 "$SIM_PID" >/dev/null 2>&1; then
    echo "Simulation failed to start. See $LOG_DIR/sim.launch.log" >&2
    return 1
  fi
}

rosbag_check() {
  local bag_dir="$LOG_DIR/rosbag"
  local bag_name="sim_smoke_bag"
  local bag_path="$bag_dir/$bag_name"
  mkdir -p "$bag_dir"

  echo "Recording short rosbag; output: $bag_path"
  setsid ros2 launch robot_sim_bringup record_bag.launch.py \
    topic_group:=all \
    output_dir:="$bag_dir" \
    bag_name:="$bag_name" \
    compression:=false \
    >"$LOG_DIR/rosbag.launch.log" 2>&1 &
  BAG_PID=$!
  sleep 8
  cleanup_process_group "$BAG_PID" INT
  BAG_PID=""

  if [[ ! -f "$bag_path/metadata.yaml" ]]; then
    echo "rosbag metadata was not created: $bag_path/metadata.yaml" >&2
    return 1
  fi
  ros2 bag info "$bag_path" >"$LOG_DIR/rosbag.info.log"
  local messages
  messages="$(awk -F: '/Messages:/ {gsub(/ /, "", $2); print $2; exit}' "$LOG_DIR/rosbag.info.log")"
  if [[ -z "$messages" || "$messages" == "0" ]]; then
    echo "rosbag contains no messages. See $LOG_DIR/rosbag.info.log" >&2
    return 1
  fi
  echo "rosbag OK: $messages messages"
}

preflight

LOG_DIR="$(mktemp -d "${TMPDIR:-/tmp}/robot_sim_smoke.XXXXXX")"
URDF_FILE="$LOG_DIR/robot.urdf"

echo "robot_sim smoke test"
echo "  profile: $PROFILE"
if [[ -n "$PROFILE_FILE" ]]; then
  echo "  profile_file: $PROFILE_FILE"
fi
echo "  mode: $MODE"
echo "  sensor_overrides: ${SENSOR_OVERRIDES:-<auto>}"
echo "  logs: $LOG_DIR"

SHELL_ENV_ARGS=("${COMMON_ARGS[@]}")
LINT_ARGS=(--profile "$PROFILE" --mode "$MODE")
if [[ -n "$PROFILE_FILE" ]]; then
  LINT_ARGS+=(--profile-file "$PROFILE_FILE")
fi
if [[ -n "$SENSOR_OVERRIDES" ]]; then
  LINT_ARGS+=(--sensor-overrides "$SENSOR_OVERRIDES")
fi
if [[ "$WITH_MOVEIT" == true ]]; then
  SHELL_ENV_ARGS+=(--with-moveit)
  LINT_ARGS+=(--require-moveit)
fi

run_step "Profile lint" "${LINTER[@]}" "${LINT_ARGS[@]}"
eval "$("${HELPER[@]}" shell-env "${SHELL_ENV_ARGS[@]}")"

run_step "Profile summary" "${HELPER[@]}" profile-json "${COMMON_ARGS[@]}"
run_step "Render URDF/xacro" "${HELPER[@]}" render-urdf "${COMMON_ARGS[@]}" --output "$URDF_FILE"
run_step "Validate URDF" check_urdf "$URDF_FILE"

start_simulation "$WITH_MOVEIT"

if [[ "$SMOKE_USE_GAZEBO" == "true" ]]; then
  run_step "Gazebo spawn" wait_gazebo_model "$SMOKE_SPAWN_NAME"
else
  echo
  echo "==> Gazebo spawn"
  echo "mock mode: skipping Gazebo spawn check"
fi

run_step "joint_states" "${HELPER[@]}" wait-joint-state "${COMMON_ARGS[@]}" --timeout "$TIMEOUT"
run_step "controllers active" "${HELPER[@]}" wait-controllers "${COMMON_ARGS[@]}" --timeout "$TIMEOUT"
run_step "trajectory action" "${HELPER[@]}" send-trajectory "${COMMON_ARGS[@]}" --timeout "$TIMEOUT"
run_step "sensor hz" "${HELPER[@]}" check-sensors "${COMMON_ARGS[@]}"
run_step "TF tree" "${HELPER[@]}" check-tf "${COMMON_ARGS[@]}" --urdf "$URDF_FILE"

if [[ "$WITH_MOVEIT" == true ]]; then
  run_step "MoveIt plan/execute" "${HELPER[@]}" moveit "${COMMON_ARGS[@]}" --timeout "$TIMEOUT"
else
  echo
  echo "==> MoveIt plan/execute"
  echo "optional check disabled; pass --with-moveit to enable"
fi

if [[ "$WITH_ROSBAG" == true ]]; then
  run_step "rosbag record" rosbag_check
else
  echo
  echo "==> rosbag record"
  echo "optional check disabled; pass --with-rosbag to enable"
fi

echo
echo "SIM SMOKE TEST PASSED"
if [[ "$KEEP_SIM" == true ]]; then
  echo "Simulation kept running with launch PID: $SIM_PID"
fi
