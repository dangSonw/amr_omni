#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
INSTALL_DIR="${ROOT_DIR}/install/ros2_jazzy"
MODE="gazebo"
WORLD=""
HEADLESS=false
USE_SIM_TIME=true
DURATION=""
RENODE_BOARD="stm32f4_discovery"
RENODE_SCRIPT=""
RENODE_FIRMWARE=""
RENODE_DURATION=""
INTERACTIVE=false
DRY_RUN=false
SKIP_BUILD_CHECK=false
LAUNCH_WEB=true
WEB_PORT=8000
ROS_LAUNCH_ARGS=()
GAZEBO_COMMAND=()
RENODE_COMMAND=()

usage() {
  cat <<'EOF'
Usage: scripts/run_sim.sh [options] [-- ros2-launch-arguments]

Run the Gazebo simulation, an STM32 Renode model, or both. The default mode is
Gazebo. The Gazebo launch includes the ROS 2 STM32 bridge and STM32 firmware
model; this script does not build implicitly or open a real USB device.

Options:
  --mode gazebo|renode|both  Simulation backend (default: gazebo)
  --world PATH               Gazebo SDF world (default: amr_lab.sdf)
  --headless                 Start Gazebo server with off-screen rendering
  --use-sim-time BOOL        ROS use_sim_time value (default: true)
  --duration SEC             Stop the selected process after SEC seconds
  --web                      Start the FastAPI web control/monitoring backend (default)
  --no-web, --noweb          Do not start the web interface
  --web-port PORT            Port for web interface (default: 8000)
  --board NAME               Renode board shortcut (default: stm32f4_discovery)
  --renode-script PATH       Renode .resc file; overrides --board
  --firmware PATH            ELF firmware passed to the Renode script
  --renode-duration VALUE    Renode virtual duration for emulation RunFor
  --interactive              Keep Renode monitor interactive (requires a TTY)
  --skip-build-check         Do not require the ROS install overlay
  --dry-run                  Print commands without running them
  --ros-arg ARG              Add one ros2 launch argument (repeatable)
  -h, --help                 Show this help

Renode board shortcuts:
  stm32f4_discovery, stm32f103, stm32f746, stm32l072

Examples:
  scripts/run_sim.sh                                    # Gazebo simulation + Web dashboard (:8000)
  scripts/run_sim.sh --no-web                           # Simulation only, no web backend
  scripts/run_sim.sh --web-port 8080                    # Serve web interface on custom port
  scripts/run_sim.sh --mode gazebo --headless --world src/omni_simulation/worlds/amr_lab.sdf
  scripts/run_sim.sh --mode renode --board stm32f4_discovery --renode-duration 5s
  scripts/run_sim.sh --mode renode --renode-script ./my_board.resc --firmware ./build/app.elf
  scripts/run_sim.sh --mode both --headless --duration 30
  scripts/run_sim.sh -- --show-args
EOF
}

die() {
  echo "run_sim.sh: $*" >&2
  exit 1
}

info() {
  echo "[INFO] $*"
}

cleanup_web() {
  if [[ -n "${WEB_PID:-}" ]]; then
    kill -INT "$WEB_PID" 2>/dev/null || true
    wait "$WEB_PID" 2>/dev/null || true
  fi
}

resolve_path() {
  local requested="$1"
  if [[ "$requested" = /* ]]; then
    printf '%s\n' "$requested"
  else
    printf '%s\n' "${ROOT_DIR}/${requested}"
  fi
}

validate_duration() {
  [[ -z "$1" || "$1" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "invalid duration: $1"
  [[ "$1" != 0 && "$1" != 0.0 ]] || die 'duration must be greater than zero'
}

source_ros2() {
  local setup_file="/opt/ros/${ROS_DISTRO}/setup.bash"
  [[ -f "$setup_file" ]] || die "ROS setup file not found: ${setup_file}"
  [[ "${ROS_VERSION:-}" != 1 ]] || die 'current shell contains ROS 1; open a clean shell.'
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
  [[ "${ROS_VERSION:-}" == 2 && "${ROS_DISTRO:-}" == "$ROS_DISTRO" ]] || \
    die "this script requires ROS 2 ${ROS_DISTRO}"
  [[ "$(uname -m)" == x86_64 ]] || die 'Gazebo simulation requires x86_64 WSL2.'
  command -v ros2 >/dev/null 2>&1 || die 'ros2 is not installed'
  if [[ -z "${ROS_LOG_DIR:-}" ]]; then
    if ! touch "${HOME}/.ros/log/.writable_test" 2>/dev/null; then
      export ROS_LOG_DIR="${ROOT_DIR}/.temp_ros_log"
      mkdir -p "$ROS_LOG_DIR"
    else
      rm -f "${HOME}/.ros/log/.writable_test"
    fi
  fi
}

renode_script_for_board() {
  case "$RENODE_BOARD" in
    stm32f4_discovery)
      printf '%s\n' /opt/renode/scripts/single-node/stm32f4_discovery.resc
      ;;
    stm32f103)
      printf '%s\n' /opt/renode/scripts/single-node/stm32f103.resc
      ;;
    stm32f746)
      printf '%s\n' /opt/renode/scripts/single-node/stm32f746.resc
      ;;
    stm32l072)
      printf '%s\n' /opt/renode/scripts/single-node/stm32l072.resc
      ;;
    *)
      die "unknown Renode board: ${RENODE_BOARD}"
      ;;
  esac
}

validate_renode() {
  command -v renode >/dev/null 2>&1 || die 'Renode is not installed'
  if [[ -z "$RENODE_SCRIPT" ]]; then
    RENODE_SCRIPT="$(renode_script_for_board)"
  else
    RENODE_SCRIPT="$(resolve_path "$RENODE_SCRIPT")"
  fi
  [[ -f "$RENODE_SCRIPT" ]] || die "Renode script not found: ${RENODE_SCRIPT}"
  if [[ -n "$RENODE_FIRMWARE" ]]; then
    RENODE_FIRMWARE="$(resolve_path "$RENODE_FIRMWARE")"
    [[ -f "$RENODE_FIRMWARE" ]] || die "firmware ELF not found: ${RENODE_FIRMWARE}"
  fi
  if [[ -n "$RENODE_DURATION" ]]; then
    [[ "$RENODE_DURATION" =~ ^[0-9]+([.][0-9]+)?(s|ms|us|ns)?$ ]] || \
      die "invalid Renode duration: ${RENODE_DURATION}"
  fi
}

run_command() {
  if [[ "$DRY_RUN" == true ]]; then
    printf '[DRY-RUN]'
    printf ' %q' "$@"
    printf '\n'
  else
    "$@"
  fi
}

run_for_real_seconds() {
  local duration="$1"
  shift
  set +e
  timeout --signal=INT --kill-after=10 "${duration}s" "$@"
  local status=$?
  set -e
  # timeout(1) returns 124 when the requested bounded run ended normally.
  # Treat that intentional stop as success; preserve all other failures.
  [[ "$status" == 124 ]] && return 0
  return "$status"
}

build_gazebo_command() {
  local world_file="$WORLD"
  if [[ -z "$world_file" ]]; then
    world_file="${ROOT_DIR}/src/omni_simulation/worlds/amr_lab.sdf"
  else
    world_file="$(resolve_path "$world_file")"
  fi
  [[ -f "$world_file" ]] || die "Gazebo world not found: ${world_file}"
  [[ -f "${INSTALL_DIR}/local_setup.bash" || "$SKIP_BUILD_CHECK" == true ]] || \
    die "ROS overlay is not built: ${INSTALL_DIR}/local_setup.bash (run scripts/build.sh)"

  GAZEBO_COMMAND=(ros2 launch omni_bringup simulation_bringup.launch.py
    "world:=${world_file}" "use_sim_time:=${USE_SIM_TIME}"
    "headless:=${HEADLESS}")
  if [[ "$LAUNCH_WEB" == true ]]; then
    local py_bin="${ROOT_DIR}/web/backend/.venv/bin/python3"
    [[ -f "$py_bin" ]] || py_bin="python3"
    GAZEBO_COMMAND+=(
      "web:=true"
      "web_port:=${WEB_PORT}"
      "web_script:=${ROOT_DIR}/web/backend/run_backend.py"
      "python_bin:=${py_bin}"
    )
  else
    GAZEBO_COMMAND+=("web:=false")
  fi
  GAZEBO_COMMAND+=("${ROS_LAUNCH_ARGS[@]}")
}

run_gazebo() {
  source_ros2
  if [[ -f "${INSTALL_DIR}/local_setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${INSTALL_DIR}/local_setup.bash"
    set -u
  elif [[ "$SKIP_BUILD_CHECK" != true ]]; then
    die "ROS overlay is not built: ${INSTALL_DIR}/local_setup.bash"
  fi
  build_gazebo_command
  info 'starting ROS 2 Gazebo simulation'
  if [[ "$LAUNCH_WEB" == true ]]; then
    info "Web interface enabled: http://localhost:${WEB_PORT}"
    info "Mẹo: Mở trình duyệt và nhấn Ctrl+Shift+R để tải giao diện mới nhất!"
  fi
  if [[ -n "$DURATION" ]]; then
    if [[ "$DRY_RUN" == true ]]; then
      run_command timeout --signal=INT --kill-after=10 "${DURATION}s" "${GAZEBO_COMMAND[@]}"
    else
      run_for_real_seconds "$DURATION" "${GAZEBO_COMMAND[@]}"
    fi
  else
    run_command "${GAZEBO_COMMAND[@]}"
  fi
}

build_renode_command() {
  RENODE_COMMAND=(renode --disable-gui)
  if [[ "$INTERACTIVE" == true ]]; then
    RENODE_COMMAND+=(--console)
  fi
  if [[ -n "$RENODE_FIRMWARE" ]]; then
    # Set the variable before including scripts that use the conventional $bin?
    # variable. This avoids creating a temporary .resc file for every ELF.
    RENODE_COMMAND+=(-e "set bin @${RENODE_FIRMWARE}")
  fi
  RENODE_COMMAND+=(-e "include @${RENODE_SCRIPT}")
  if [[ -n "$RENODE_DURATION" ]]; then
    RENODE_COMMAND+=(-e "emulation RunFor \"${RENODE_DURATION}\"" -e q)
  fi
}

run_renode() {
  validate_renode
  if [[ "$INTERACTIVE" == true && ! -t 0 ]]; then
    die '--interactive requires a terminal stdin (do not use it in a background/CI job)'
  fi
  build_renode_command
  info "starting Renode: ${RENODE_SCRIPT}"
  if [[ -n "$DURATION" ]]; then
    if [[ "$DRY_RUN" == true ]]; then
      run_command timeout --signal=INT --kill-after=10 "${DURATION}s" "${RENODE_COMMAND[@]}"
    else
      run_for_real_seconds "$DURATION" "${RENODE_COMMAND[@]}"
    fi
  else
    run_command "${RENODE_COMMAND[@]}"
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      [[ $# -ge 2 ]] || die '--mode requires gazebo, renode or both'
      MODE="$2"
      shift 2
      ;;
    --world)
      [[ $# -ge 2 ]] || die '--world requires a path'
      WORLD="$2"
      shift 2
      ;;
    --headless)
      HEADLESS=true
      shift
      ;;
    --use-sim-time)
      [[ $# -ge 2 ]] || die '--use-sim-time requires true or false'
      USE_SIM_TIME="$2"
      shift 2
      ;;
    --duration)
      [[ $# -ge 2 ]] || die '--duration requires seconds'
      DURATION="$2"
      shift 2
      ;;
    --board)
      [[ $# -ge 2 ]] || die '--board requires a name'
      RENODE_BOARD="$2"
      shift 2
      ;;
    --renode-script)
      [[ $# -ge 2 ]] || die '--renode-script requires a path'
      RENODE_SCRIPT="$2"
      shift 2
      ;;
    --firmware)
      [[ $# -ge 2 ]] || die '--firmware requires an ELF path'
      RENODE_FIRMWARE="$2"
      shift 2
      ;;
    --renode-duration)
      [[ $# -ge 2 ]] || die '--renode-duration requires a value'
      RENODE_DURATION="$2"
      shift 2
      ;;
    --interactive)
      INTERACTIVE=true
      shift
      ;;
    --skip-build-check)
      SKIP_BUILD_CHECK=true
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --web)
      LAUNCH_WEB=true
      shift
      ;;
    --no-web|--noweb)
      LAUNCH_WEB=false
      shift
      ;;
    --web-port)
      [[ $# -ge 2 ]] || die '--web-port requires a port number'
      WEB_PORT="$2"
      shift 2
      ;;
    --ros-arg)
      [[ $# -ge 2 ]] || die '--ros-arg requires NAME:=VALUE'
      ROS_LAUNCH_ARGS+=("$2")
      shift 2
      ;;
    --)
      shift
      ROS_LAUNCH_ARGS+=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1 (use --ros-arg NAME:=VALUE or -- ...)"
      ;;
  esac
done

case "$MODE" in
  gazebo|renode|both) ;;
  *) die "invalid --mode: ${MODE}" ;;
esac
[[ "$USE_SIM_TIME" == true || "$USE_SIM_TIME" == false ]] || die '--use-sim-time must be true or false'
validate_duration "$DURATION"

python_bin="${ROOT_DIR}/web/backend/.venv/bin/python3"
[[ -f "$python_bin" ]] || python_bin="python3"
web_command=("$python_bin" "${ROOT_DIR}/web/backend/run_backend.py" --port "$WEB_PORT" --mode ros2)

if [[ "$MODE" == gazebo ]]; then
  run_gazebo
elif [[ "$MODE" == renode ]]; then
  if [[ "$DRY_RUN" == true && "$LAUNCH_WEB" == true ]]; then
    run_command "${web_command[@]}"
  fi
  if [[ "$DRY_RUN" != true && "$LAUNCH_WEB" == true ]]; then
    source_ros2
    info "starting web backend on port ${WEB_PORT}..."
    "${web_command[@]}" &
    WEB_PID=$!
    trap cleanup_web EXIT INT TERM
  fi
  run_renode
else
  source_ros2
  if [[ -f "${INSTALL_DIR}/local_setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${INSTALL_DIR}/local_setup.bash"
    set -u
  elif [[ "$SKIP_BUILD_CHECK" != true ]]; then
    die "ROS overlay is not built: ${INSTALL_DIR}/local_setup.bash"
  fi
  validate_renode
  if [[ "$INTERACTIVE" == true && ! -t 0 ]]; then
    die '--interactive requires a terminal stdin (do not use it in a background/CI job)'
  fi
  [[ "$DRY_RUN" == true || -n "$DURATION" ]] || \
    die '--mode both requires --duration so both processes can be cleaned up safely'
  build_gazebo_command
  ros_command=("${GAZEBO_COMMAND[@]}")
  build_renode_command
  renode_command=("${RENODE_COMMAND[@]}")
  info 'starting Gazebo and Renode together'
  if [[ "$DRY_RUN" == true ]]; then
    run_command "${ros_command[@]}"
    run_command "${renode_command[@]}"
  else
    pids=()
    cleanup() {
      local pid
      for pid in "${pids[@]:-}"; do
        kill -INT "$pid" 2>/dev/null || true
      done
      wait 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM
    if [[ -n "$DURATION" ]]; then
      timeout --signal=INT --kill-after=10 "${DURATION}s" "${ros_command[@]}" &
      pids+=("$!")
      timeout --signal=INT --kill-after=10 "${DURATION}s" "${renode_command[@]}" &
      pids+=("$!")
    else
      "${ros_command[@]}" &
      pids+=("$!")
      "${renode_command[@]}" &
      pids+=("$!")
    fi
    set +e
    wait "${pids[0]}"
    status=$?
    wait "${pids[1]}" 2>/dev/null
    other_status=$?
    set -e
    if [[ -n "$DURATION" ]]; then
      [[ "$status" == 124 ]] && status=0
      [[ "$other_status" == 124 ]] && other_status=0
    fi
    if [[ "$status" != 0 ]]; then
      exit "$status"
    fi
    [[ "$other_status" == 0 ]] || exit "$other_status"
  fi
fi
