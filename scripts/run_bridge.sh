#!/usr/bin/env bash
# scripts/run_bridge.sh
# Bridge giữa ROS 1 Melodic (Jetson Nano) và ROS 2 Jazzy (WSL2/Host)

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

usage() {
  cat <<'HELP_EOF'
Usage: scripts/run_bridge.sh [options]

Bridge communication between ROS 1 Melodic (Jetson Nano) and ROS 2 Jazzy (Host/WSL2).
Uses ros1_bridge dynamic_bridge if available, or falls back to Rosbridge WebSocket mode.

Options:
  --master-uri URI  Set ROS 1 master URI (default: http://localhost:11311)
  -h, --help        Show this help message

Examples:
  scripts/run_bridge.sh
  scripts/run_bridge.sh --master-uri http://192.168.1.100:11311
HELP_EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --master-uri)
      [[ $# -ge 2 ]] || die '--master-uri requires a URI'
      ROS_MASTER_URI="$2"
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

info "Khởi động cầu nối ROS 1 <-> ROS 2 Bridge..."

ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_MASTER_URI
info "ROS 1 Master: ${ROS_MASTER_URI}"

if command -v ros2 >/dev/null 2>&1; then
  if ros2 pkg list 2>/dev/null | grep -q "ros1_bridge"; then
    info "Phát hiện package ros1_bridge. Đang chạy dynamic_bridge..."
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
    exit 0
  fi
fi

warn "Package ros1_bridge nhị phân không có sẵn."
info "Sử dụng chế độ Rosbridge WebSocket bridge (cổng 9090) cho web và telemetry."
info "Hãy đảm bảo 'roslaunch rosbridge_server rosbridge_websocket.launch' đang chạy trên Jetson."

