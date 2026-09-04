#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
ROS_DISTRO="melodic"
ROS1_WORKSPACE=""
BRINGUP_PACKAGE=""
BRINGUP_LAUNCH=""
DRY_RUN=false

usage() {
  cat <<'EOF'
Usage: scripts/run_robot.sh --workspace PATH --package NAME --launch FILE [options]

Run the ROS 1 hardware bringup on the Jetson Nano only. The repository does
not currently contain a ROS 1 bringup package, so this command fails clearly
until an external/real ROS 1 workspace and launch file are supplied.

Options:
  --workspace PATH       Catkin workspace containing devel/setup.bash
  --package NAME         ROS package that owns the hardware launch file
  --launch FILE          roslaunch file name
  --ros-distro DISTRO    ROS distribution (default: melodic)
  --dry-run              Validate and print commands without starting roscore
  --no-roscore           Do not start roscore (caller supplies ROS master)
  --web                  Start the FastAPI web control/monitoring backend
  --no-web               Do not start the web interface (default)
  --web-port PORT        Port for web interface (default: 8000)
  --ros-arg ARG          Add roslaunch argument (repeatable)
  -h, --help             Show this help

Safety:
  This script never runs as root and never enables actuators implicitly. Verify
  E-stop, watchdog, device, battery and command-timeout behavior before use.
EOF
}

die() {
  echo "run_robot.sh: $*" >&2
  exit 1
}

ROS1_WORKSPACE=""
START_ROSCORE=true
LAUNCH_WEB=false
WEB_PORT=8000
ROSLAUNCH_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      [[ $# -ge 2 ]] || die '--workspace requires a path'
      ROS1_WORKSPACE="$2"
      shift 2
      ;;
    --package)
      [[ $# -ge 2 ]] || die '--package requires a ROS package'
      BRINGUP_PACKAGE="$2"
      shift 2
      ;;
    --launch)
      [[ $# -ge 2 ]] || die '--launch requires a launch file'
      BRINGUP_LAUNCH="$2"
      shift 2
      ;;
    --ros-distro)
      [[ $# -ge 2 ]] || die '--ros-distro requires a value'
      ROS_DISTRO="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --no-roscore)
      START_ROSCORE=false
      shift
      ;;
    --web)
      LAUNCH_WEB=true
      shift
      ;;
    --no-web)
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
      ROSLAUNCH_ARGS+=("$2")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
done

[[ "$(uname -m)" == aarch64 ]] || die 'run_robot.sh must run on Jetson Nano (aarch64).'
[[ "$ROS_DISTRO" == melodic ]] || die 'run_robot.sh only supports ROS 1 Melodic on this baseline.'
[[ -n "$ROS1_WORKSPACE" ]] || die '--workspace is required'
[[ -n "$BRINGUP_PACKAGE" ]] || die '--package is required'
[[ -n "$BRINGUP_LAUNCH" ]] || die '--launch is required'
[[ -d "$ROS1_WORKSPACE" ]] || die "workspace not found: ${ROS1_WORKSPACE}"
[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || die "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "${ROS1_WORKSPACE}/devel/setup.bash" ]] || \
  die "catkin workspace is not built: ${ROS1_WORKSPACE}/devel/setup.bash"

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1090
source "${ROS1_WORKSPACE}/devel/setup.bash"
set -u
[[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == "$ROS_DISTRO" ]] || \
  die "expected ROS 1 ${ROS_DISTRO} after sourcing workspaces"
command -v roslaunch >/dev/null 2>&1 || die 'roslaunch is not installed'
package_path="$(rospack find "$BRINGUP_PACKAGE" 2>/dev/null)" || \
  die "ROS package not found: ${BRINGUP_PACKAGE}"
roslaunch_file="${package_path}/launch/${BRINGUP_LAUNCH}"
[[ -f "$roslaunch_file" ]] || die "launch file not found: ${BRINGUP_PACKAGE}/launch/${BRINGUP_LAUNCH}"

roslaunch_command=(roslaunch "$BRINGUP_PACKAGE" "$BRINGUP_LAUNCH" "${ROSLAUNCH_ARGS[@]}")
roscore_command=(roscore)
python_bin="${ROOT_DIR}/web/backend/.venv/bin/python3"
[[ -f "$python_bin" ]] || python_bin="python3"
web_command=("$python_bin" "${ROOT_DIR}/web/backend/run_backend.py" --port "$WEB_PORT" --mode ros1)

if [[ "$DRY_RUN" == true ]]; then
  printf '[DRY-RUN]'
  printf ' %q' "${roscore_command[@]}"
  printf '\n[DRY-RUN]'
  printf ' %q' "${roslaunch_command[@]}"
  if [[ "$LAUNCH_WEB" == true ]]; then
    printf '\n[DRY-RUN]'
    printf ' %q' "${web_command[@]}"
  fi
  printf '\n'
  exit 0
fi

ROSCORE_PID=""
WEB_PID=""

cleanup() {
  if [[ -n "$WEB_PID" ]]; then
    echo '[INFO] stopping web backend...'
    kill -INT "$WEB_PID" 2>/dev/null || true
    wait "$WEB_PID" 2>/dev/null || true
  fi
  if [[ -n "$ROSCORE_PID" ]]; then
    echo '[INFO] stopping roscore...'
    kill -INT "$ROSCORE_PID" 2>/dev/null || true
    wait "$ROSCORE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$START_ROSCORE" == true ]]; then
  echo '[INFO] starting roscore; press Ctrl-C to stop the robot bringup.'
  "${roscore_command[@]}" &
  ROSCORE_PID=$!
  sleep 2
fi

if [[ "$LAUNCH_WEB" == true ]]; then
  echo "[INFO] starting web backend on port ${WEB_PORT}..."
  "${web_command[@]}" &
  WEB_PID=$!
  sleep 1
fi

set +e
"${roslaunch_command[@]}"
launch_status=$?
set -e

cleanup
trap - EXIT INT TERM
exit "$launch_status"
