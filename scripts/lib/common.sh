#!/usr/bin/env bash
# scripts/lib/common.sh
# Shared utilities for AMR Omni scripts (logging, root detection, environment sourcing, process traps)

set -Eeuo pipefail

# ANSI color codes (only when stdout is a terminal)
if [[ -t 1 ]]; then
  CLR_RESET='\033[0m'
  CLR_BOLD='\033[1m'
  CLR_RED='\033[31m'
  CLR_GREEN='\033[32m'
  CLR_YELLOW='\033[33m'
  CLR_CYAN='\033[36m'
else
  CLR_RESET=''
  CLR_BOLD=''
  CLR_RED=''
  CLR_GREEN=''
  CLR_YELLOW=''
  CLR_CYAN=''
fi

info() {
  printf '%b\n' "${CLR_GREEN}${CLR_BOLD}[INFO]${CLR_RESET} $*"
}

warn() {
  printf '%b\n' "${CLR_YELLOW}${CLR_BOLD}[WARN]${CLR_RESET} $*" >&2
}

fail() {
  printf '%b\n' "${CLR_RED}${CLR_BOLD}[FAIL]${CLR_RESET} $*" >&2
}

die() {
  fail "$*"
  exit 1
}

# Resolve repository root directory reliably
get_repo_root() {
  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
  git -C "$script_dir" rev-parse --show-toplevel 2>/dev/null || dirname "$(dirname "$script_dir")"
}

# Source ROS setup safely (handling unset variables in standard ROS setup files)
source_ros_setup() {
  local distro="${1:-jazzy}"
  local setup_file="/opt/ros/${distro}/setup.bash"

  if [[ ! -f "$setup_file" ]]; then
    die "ROS setup file not found: ${setup_file}"
  fi

  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

# Resolve Python binary (prefer web virtual environment, fallback to system python3)
resolve_python_executable() {
  local root_dir="$1"
  local venv_python="${root_dir}/web/backend/.venv/bin/python3"
  if [[ -x "$venv_python" ]]; then
    printf '%s' "$venv_python"
  else
    command -v python3 || printf '%s' "python3"
  fi
}

# Global array of registered background PIDs for safe cleanup on exit / SIGINT
REGISTERED_PIDS=()

register_pid() {
  REGISTERED_PIDS+=("$1")
}

cleanup_registered_pids() {
  local pid
  for pid in "${REGISTERED_PIDS[@]:-}"; do
    if [[ -n "$pid" ]]; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "${REGISTERED_PIDS[@]:-}"; do
    if [[ -n "$pid" ]]; then
      wait "$pid" 2>/dev/null || true
    fi
  done
  REGISTERED_PIDS=()
}

