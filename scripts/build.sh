#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
ROOT_DIR="$(get_repo_root)"

COMPONENT="ros2"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
BUILD_DIR="${ROOT_DIR}/build/ros2_jazzy"
INSTALL_DIR="${ROOT_DIR}/install/ros2_jazzy"
LOG_DIR="${ROOT_DIR}/log/ros2_jazzy"
TEST_RESULT_DIR="${BUILD_DIR}/test_results"
TEST_RESULT_EXPLICIT=false
FIRMWARE_FILTERS=()
PIO_ENV=""
ROS_PACKAGES=()
DO_BUILD=true
DO_TEST=false
CLEAN=false
SKIP_EMPTY=false
SKIPPED=0

usage() {
  cat <<'EOF'
Usage: scripts/build.sh [options]

Build and optionally test ROS 2, PlatformIO firmware, or both. ROS 1 and ROS 2
artifacts are kept separate. Firmware projects without source are never
reported as successful; use --skip-empty only when a scaffold is intentionally
being skipped.

Options:
  --component ros2|firmware|web|all  What to build (default: ros2)
  --firmware NAME|PATH           PlatformIO project (repeatable; default: all)
  --environment ENV              PlatformIO environment, e.g. disco_f407vg
  --package NAME                 Select one ROS package (repeatable)
  --test                         Run tests after building
  --test-only                    Run tests without building first
  --clean                        Remove selected ROS build/install/log dirs
  --skip-empty                   Skip firmware projects with no source/tests
  --ros-distro DISTRO            ROS distribution (default: jazzy)
  --build-base PATH              ROS colcon build directory
  --install-base PATH            ROS colcon install directory
  --log-base PATH                ROS colcon log directory
  --test-result-base PATH        ROS test result directory
  -h, --help                     Show this help

Examples:
  scripts/build.sh --component ros2 --test
  scripts/build.sh --component firmware --firmware stm32_f407vg_arduino_sim
  scripts/build.sh --component all --test --skip-empty
  scripts/build.sh --component ros2 --test-only --package omni_control
EOF
}

resolve_firmware_project() {
  local requested="$1"
  if [[ -d "$requested" ]]; then
    printf '%s\n' "$(cd "$requested" && pwd)"
    return 0
  fi
  if [[ -d "${ROOT_DIR}/firmware/${requested}" ]]; then
    printf '%s\n' "${ROOT_DIR}/firmware/${requested}"
    return 0
  fi
  return 1
}

resolve_repo_path() {
  local requested="$1"
  if [[ "$requested" = /* ]]; then
    printf '%s\n' "$requested"
  else
    printf '%s\n' "${ROOT_DIR}/${requested}"
  fi
}

has_firmware_source() {
  find "$1/src" -type f \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.ino' \) \
    -print -quit 2>/dev/null | grep -q .
}

has_firmware_tests() {
  find "$1/test" -type f \
    ! -name 'README' ! -name 'README.md' ! -name '*.md' \
    -print -quit 2>/dev/null | grep -q .
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
  if [[ -x /usr/bin/colcon ]]; then
    COLCON_BIN=/usr/bin/colcon
  else
    command -v colcon >/dev/null 2>&1 || die 'colcon is not installed'
    COLCON_BIN="$(command -v colcon)"
  fi
  [[ "$(uname -m)" == x86_64 ]] || die 'ROS 2 simulation build requires x86_64 WSL2.'
  if [[ -z "${ROS_LOG_DIR:-}" ]]; then
    if ! touch "${HOME}/.ros/log/.writable_test" 2>/dev/null; then
      export ROS_LOG_DIR="${ROOT_DIR}/.temp_ros_log"
      mkdir -p "$ROS_LOG_DIR"
    else
      rm -f "${HOME}/.ros/log/.writable_test"
    fi
  fi
}

validate_inside_root() {
  local path="$1"
  case "$path" in
    "${ROOT_DIR}"/*) ;;
    *) die "path must be inside repository: ${path}" ;;
  esac
}

clean_ros_artifacts() {
  local path
  for path in "$BUILD_DIR" "$INSTALL_DIR" "$LOG_DIR"; do
    validate_inside_root "$path"
    rm -rf -- "$path"
  done
}

build_ros2() {
  source_ros2
  if [[ "$CLEAN" == true ]]; then
    info 'cleaning ROS 2 artifacts'
    clean_ros_artifacts
  fi

  local -a build_command=(
    "$COLCON_BIN" --log-base "$LOG_DIR" build
    --symlink-install
    --base-paths src
    --build-base "$BUILD_DIR"
    --install-base "$INSTALL_DIR"
    --event-handlers console_direct+
  )
  if [[ "${#ROS_PACKAGES[@]}" -gt 0 ]]; then
    build_command+=(--packages-up-to "${ROS_PACKAGES[@]}")
  fi
  if [[ "$DO_BUILD" == true ]]; then
    info 'building ROS 2 workspace'
    (cd "$ROOT_DIR" && "${build_command[@]}")
    info "ROS 2 build complete: ${INSTALL_DIR}"
  fi

  if [[ "$DO_TEST" == true ]]; then
    [[ -f "${INSTALL_DIR}/local_setup.bash" ]] || \
      die "ROS 2 overlay is not built: ${INSTALL_DIR}/local_setup.bash"
    set +u
    # shellcheck disable=SC1090
    source "${INSTALL_DIR}/local_setup.bash"
    set -u
    local -a test_command=(
      "$COLCON_BIN" --log-base "$LOG_DIR" test
      --base-paths src
      --build-base "$BUILD_DIR"
      --install-base "$INSTALL_DIR"
      --test-result-base "$TEST_RESULT_DIR"
      --python-testing pytest
      --event-handlers console_direct+
    )
    if [[ "${#ROS_PACKAGES[@]}" -gt 0 ]]; then
      test_command+=(--packages-select "${ROS_PACKAGES[@]}")
    fi
    info 'testing ROS 2 workspace'
    (cd "$ROOT_DIR" && "${test_command[@]}")
    (cd "$ROOT_DIR" && "$COLCON_BIN" test-result --test-result-base "$TEST_RESULT_DIR" --verbose)
    info 'ROS 2 tests complete'
  fi
}

build_firmware_project() {
  local project="$1"
  local name="$(basename "$project")"
  local pio_bin
  if command -v pio >/dev/null 2>&1; then
    pio_bin=pio
  elif command -v platformio >/dev/null 2>&1; then
    pio_bin=platformio
  else
    die 'PlatformIO is not installed (run scripts/setup.sh --install-platformio).'
  fi
  [[ -f "${project}/platformio.ini" ]] || die "platformio.ini not found: ${project}"

  if ! has_firmware_source "$project"; then
    if [[ "$SKIP_EMPTY" == true ]]; then
      warn "SKIP ${name}: no source files under ${project}/src"
      SKIPPED=$((SKIPPED + 1))
      return 0
    fi
    die "${name} has no firmware source; use --skip-empty only to skip the scaffold."
  fi

  local -a build_command=("$pio_bin" run --project-dir "$project")
  local test_env="${PIO_ENV:-native}"
  local -a test_command=("$pio_bin" test --project-dir "$project" --environment "$test_env")
  if [[ -n "$PIO_ENV" ]]; then
    build_command+=(--environment "$PIO_ENV")
  fi
  if [[ "$DO_BUILD" == true ]]; then
    info "building firmware: ${name}"
    "${build_command[@]}"
  fi
  if [[ "$DO_TEST" == true ]]; then
    if ! has_firmware_tests "$project"; then
      if [[ "$SKIP_EMPTY" == true ]]; then
        warn "SKIP ${name} tests: no PlatformIO test source"
        SKIPPED=$((SKIPPED + 1))
      else
        die "${name} has no PlatformIO test source; use --skip-empty to skip it."
      fi
    else
      info "testing firmware: ${name}"
      "${test_command[@]}"
    fi
  fi
}

build_firmware() {
  local requested project
  local -a projects=()
  if [[ "${#FIRMWARE_FILTERS[@]}" == 0 ]]; then
    mapfile -t projects < <(find "${ROOT_DIR}/firmware" -name platformio.ini -printf '%h\n' | sort)
  else
    for requested in "${FIRMWARE_FILTERS[@]}"; do
      project="$(resolve_firmware_project "$requested")" || die "firmware project not found: ${requested}"
      projects+=("$project")
    done
  fi
  [[ "${#projects[@]}" -gt 0 ]] || die 'no PlatformIO projects were found'
  for project in "${projects[@]}"; do
    build_firmware_project "$project"
  done
}

build_web() {
  local frontend_dir="${ROOT_DIR}/web/frontend"
  local static_dir="${ROOT_DIR}/web/backend/static"
  local backend_dir="${ROOT_DIR}/web/backend"

  if [[ "$CLEAN" == true ]]; then
    info 'cleaning web frontend build artifacts'
    rm -rf "${frontend_dir}/.next" "${frontend_dir}/out"
  fi

  if [[ "$DO_BUILD" == true ]]; then
    command -v npm >/dev/null 2>&1 || die 'npm is not installed (run scripts/setup.sh).'
    command -v node >/dev/null 2>&1 || die 'node is not installed (run scripts/setup.sh).'

    info 'building Next.js frontend static bundle'
    (cd "$frontend_dir" && npm run build)

    info "copying static export bundle to ${static_dir}"
    mkdir -p "$static_dir"
    rm -rf "${static_dir}/_next"
    cp -r "${frontend_dir}/out/"* "${static_dir}/"
    info 'web frontend build complete'
  fi

  if [[ "$DO_TEST" == true ]]; then
    info 'testing web backend APIs'
    local py_bin="${backend_dir}/.venv/bin/python3"
    if [[ ! -x "$py_bin" ]]; then
      py_bin="$(command -v python3)"
    fi
    (cd "$backend_dir" && "$py_bin" -m pytest tests/)
    info 'web tests complete'
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --component)
      [[ $# -ge 2 ]] || die '--component requires ros2, firmware, web or all'
      COMPONENT="$2"
      shift 2
      ;;
    --firmware)
      [[ $# -ge 2 ]] || die '--firmware requires NAME or PATH'
      FIRMWARE_FILTERS+=("$2")
      shift 2
      ;;
    --environment|--env)
      [[ $# -ge 2 ]] || die '--environment requires a PlatformIO environment'
      PIO_ENV="$2"
      shift 2
      ;;
    --package)
      [[ $# -ge 2 ]] || die '--package requires a ROS package name'
      ROS_PACKAGES+=("$2")
      shift 2
      ;;
    --test)
      DO_TEST=true
      shift
      ;;
    --test-only)
      DO_BUILD=false
      DO_TEST=true
      shift
      ;;
    --clean)
      CLEAN=true
      shift
      ;;
    --skip-empty)
      SKIP_EMPTY=true
      shift
      ;;
    --ros-distro)
      [[ $# -ge 2 ]] || die '--ros-distro requires a value'
      ROS_DISTRO="$2"
      shift 2
      ;;
    --build-base)
      [[ $# -ge 2 ]] || die '--build-base requires a path'
      BUILD_DIR="$(resolve_repo_path "$2")"
      [[ "$TEST_RESULT_EXPLICIT" == true ]] || TEST_RESULT_DIR="${BUILD_DIR}/test_results"
      shift 2
      ;;
    --install-base)
      [[ $# -ge 2 ]] || die '--install-base requires a path'
      INSTALL_DIR="$(resolve_repo_path "$2")"
      shift 2
      ;;
    --log-base)
      [[ $# -ge 2 ]] || die '--log-base requires a path'
      LOG_DIR="$(resolve_repo_path "$2")"
      shift 2
      ;;
    --test-result-base)
      [[ $# -ge 2 ]] || die '--test-result-base requires a path'
      TEST_RESULT_DIR="$(resolve_repo_path "$2")"
      TEST_RESULT_EXPLICIT=true
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

case "${COMPONENT:-ros2}" in
  ros2|firmware|web|all) ;;
  *) die "invalid --component: ${COMPONENT:-} (valid: ros2, firmware, web, all)" ;;
esac
COMPONENT="${COMPONENT:-ros2}"
if [[ "$COMPONENT" == firmware && "${#ROS_PACKAGES[@]}" -gt 0 ]]; then
  die '--package is only valid with --component ros2 or all'
fi
if [[ "$COMPONENT" == web && "${#ROS_PACKAGES[@]}" -gt 0 ]]; then
  die '--package is only valid with --component ros2 or all'
fi
if [[ "$COMPONENT" == ros2 && "${#FIRMWARE_FILTERS[@]}" -gt 0 ]]; then
  die '--firmware is only valid with --component firmware or all'
fi
if [[ "$CLEAN" == true && "$DO_BUILD" == false ]]; then
  die '--clean cannot be combined with --test-only'
fi

case "$COMPONENT" in
  ros2)
    build_ros2
    ;;
  firmware)
    build_firmware
    ;;
  web)
    build_web
    ;;
  all)
    build_ros2
    build_firmware
    build_web
    ;;
esac
info "build workflow completed (skipped components: ${SKIPPED})"
