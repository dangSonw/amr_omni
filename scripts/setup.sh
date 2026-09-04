#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
CHECK_FIRMWARE=false
INSTALL_FLASH_TOOLS=false
INSTALL_PLATFORMIO=false
STRICT=false
YES=false
FIRMWARE_FILTERS=()
FAILURES=0
WARNINGS=0

usage() {
  cat <<'EOF'
Usage: scripts/setup.sh [options]

Inspect the development host and optionally install tools. The default action
is read-only. This script never edits .bashrc, ROS setup files, udev rules or
other shell/system configuration.

Options:
  --check                    Check ROS, Renode and optional firmware tools
  --firmware NAME|PATH       Inspect one PlatformIO project (repeatable)
  --strict                   Treat missing optional tools/source as a failure
  --install-flash-tools      Install openocd, stlink-tools and dfu-util with apt
  --install-platformio       Install PlatformIO for the current user
  --yes                      Confirm apt installation
  --ros-distro DISTRO        ROS distribution to inspect (default: jazzy)
  -h, --help                 Show this help

Examples:
  scripts/setup.sh --check
  scripts/setup.sh --check --firmware stm32_f407vg_arduino_sim
  scripts/setup.sh --install-flash-tools --yes
EOF
}

die() {
  echo "setup.sh: $*" >&2
  exit 2
}

info() {
  echo "[INFO] $*"
}

warn() {
  WARNINGS=$((WARNINGS + 1))
  echo "[WARN] $*" >&2
}

fail() {
  FAILURES=$((FAILURES + 1))
  echo "[FAIL] $*" >&2
}

command_exists() {
  command -v "$1" >/dev/null 2>&1
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

check_command() {
  local required="$1"
  local name="$2"
  if command_exists "$name"; then
    info "found ${name}: $(command -v "$name")"
    return 0
  fi
  if [[ "$required" == true || "$STRICT" == true ]]; then
    fail "missing command: ${name}"
  else
    warn "optional command is not installed: ${name}"
  fi
  return 0
}

check_ros() {
  local setup_file="/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ ! -f "$setup_file" ]]; then
    fail "ROS setup file does not exist: ${setup_file}"
    return 0
  fi
  if [[ "${ROS_VERSION:-}" == 1 ]]; then
    fail 'current shell contains ROS 1; open a clean shell before checking ROS 2.'
    return 0
  fi

  # ROS setup files can reference intentionally unset variables.
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
  if [[ "${ROS_VERSION:-}" != 2 || "${ROS_DISTRO:-}" != "$ROS_DISTRO" ]]; then
    fail "expected ROS 2 ${ROS_DISTRO} after sourcing ${setup_file}"
    return 0
  fi

  check_command true colcon
  check_command true ros2
  check_command true xacro
  if command_exists gz; then
    info "found gz: $(gz sim --version 2>/dev/null | head -1 || true)"
  else
    warn 'Gazebo command "gz" is not installed; ROS package build may still work.'
  fi
}

check_renode() {
  check_command false renode
  if command_exists renode; then
    renode --version 2>/dev/null | head -1 || true
  fi
}

check_platformio() {
  if command_exists pio; then
    info "found pio: $(command -v pio)"
  elif command_exists platformio; then
    info "found platformio: $(command -v platformio)"
  elif [[ "$CHECK_FIRMWARE" == true || "$STRICT" == true ]]; then
    fail 'PlatformIO is missing; install it with --install-platformio.'
  else
    warn 'PlatformIO is not installed; firmware checks are disabled.'
  fi
}

check_firmware_project() {
  local project="$1"
  local source_count
  if [[ ! -f "${project}/platformio.ini" ]]; then
    fail "platformio.ini not found: ${project}"
    return 0
  fi
  info "PlatformIO project: ${project}"
  source_count="$(find "${project}/src" -type f \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.ino' \) 2>/dev/null | wc -l)"
  if [[ "$source_count" == 0 ]]; then
    if [[ "$STRICT" == true ]]; then
      fail "project has no firmware source under ${project}/src"
    else
      warn "configuration scaffold has no source: ${project}/src"
    fi
  else
    info "firmware source files: ${source_count}"
  fi
  if [[ -d "${project}/test" ]]; then
    local test_count
    test_count="$(find "${project}/test" -type f ! -name 'README' ! -name 'README.md' ! -name '*.md' | wc -l)"
    info "PlatformIO test files: ${test_count}"
  fi
}

install_flash_tools() {
  [[ "$YES" == true ]] || die '--install-flash-tools requires --yes'
  command_exists sudo || die 'sudo is required to install flash tools'
  command_exists apt-get || die 'apt-get is required to install flash tools'
  info 'Installing openocd, stlink-tools and dfu-util with apt...'
  sudo apt-get update
  sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y openocd stlink-tools dfu-util
}

install_platformio() {
  command_exists python3 || die 'python3 is required to install PlatformIO'
  info 'Installing PlatformIO for the current user (no sudo pip)...'
  python3 -m pip install --user platformio
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check)
      shift
      ;;
    --firmware)
      [[ $# -ge 2 ]] || die '--firmware requires NAME or PATH'
      CHECK_FIRMWARE=true
      FIRMWARE_FILTERS+=("$2")
      shift 2
      ;;
    --strict)
      STRICT=true
      shift
      ;;
    --install-flash-tools)
      INSTALL_FLASH_TOOLS=true
      shift
      ;;
    --install-platformio)
      INSTALL_PLATFORMIO=true
      shift
      ;;
    --yes)
      YES=true
      shift
      ;;
    --ros-distro)
      [[ $# -ge 2 ]] || die '--ros-distro requires a value'
      ROS_DISTRO="$2"
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

[[ "$(uname -m)" == x86_64 ]] || warn "host is $(uname -m); WSL2 baseline is x86_64"
info "repository: ${ROOT_DIR}"
info "host: $(uname -s) $(uname -m)"
info "ROS target: ${ROS_DISTRO}"

if [[ "$INSTALL_FLASH_TOOLS" == true ]]; then
  install_flash_tools
fi
if [[ "$INSTALL_PLATFORMIO" == true ]]; then
  install_platformio
fi

check_ros
check_renode
if [[ "$CHECK_FIRMWARE" == true ]]; then
  check_platformio
  if [[ "${#FIRMWARE_FILTERS[@]}" == 0 ]]; then
    mapfile -t FIRMWARE_FILTERS < <(find "${ROOT_DIR}/firmware" -name platformio.ini -printf '%h\n' | sort)
  fi
  for requested in "${FIRMWARE_FILTERS[@]}"; do
    if project="$(resolve_firmware_project "$requested")"; then
      check_firmware_project "$project"
    else
      fail "firmware project not found: ${requested}"
    fi
  done
fi

if [[ "$FAILURES" -ne 0 ]]; then
  echo "setup.sh: ${FAILURES} check(s) failed, ${WARNINGS} warning(s)." >&2
  exit 1
fi
info "environment checks completed with ${WARNINGS} warning(s)."
