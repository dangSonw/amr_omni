#!/usr/bin/env bash
# scripts/save_map.sh
# Lưu bản đồ OccupancyGrid hiện tại từ Nav2 ra file YAML + PGM

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

ROOT_DIR="$(get_repo_root)"
MAP_NAME="${1:-amr_lab_map}"
MAPS_DIR="${ROOT_DIR}/maps"

mkdir -p "$MAPS_DIR"
OUTPUT_PREFIX="${MAPS_DIR}/${MAP_NAME}"

info "Tiến hành lưu bản đồ Nav2 tới: ${OUTPUT_PREFIX}.yaml"

if command -v ros2 >/dev/null 2>&1; then
  source_ros_setup jazzy
  ros2 run nav2_map_server map_saver_cli -f "$OUTPUT_PREFIX" --occ 0.65 --free 0.25
  info "Đã lưu bản đồ thành công tại ${OUTPUT_PREFIX}.yaml và ${OUTPUT_PREFIX}.pgm"
else
  die "Lệnh 'ros2' không tồn tại trên môi trường hiện tại."
fi

