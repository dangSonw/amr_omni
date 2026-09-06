#!/usr/bin/env bash
# scripts/save_map.sh
# Lưu bản đồ OccupancyGrid hiện tại từ Nav2 ra file YAML + PGM

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

ROOT_DIR="$(get_repo_root)"
MAP_NAME="amr_lab_map"
MAPS_DIR="${ROOT_DIR}/maps"
OCC_THRESH="0.65"
FREE_THRESH="0.25"

usage() {
  cat <<'HELP_EOF'
Usage: scripts/save_map.sh [options] [map_name]

Save current Nav2 OccupancyGrid map to YAML and PGM files.

Arguments:
  map_name        Name prefix for output map files (default: amr_lab_map)

Options:
  --dir PATH      Custom output directory for map files (default: maps/)
  --occ THRESH    Occupied space threshold percentage (default: 0.65)
  --free THRESH   Free space threshold percentage (default: 0.25)
  -h, --help      Show this help message

Examples:
  scripts/save_map.sh
  scripts/save_map.sh warehouse_map
  scripts/save_map.sh my_map --dir /tmp/maps
HELP_EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dir)
      [[ $# -ge 2 ]] || die '--dir requires a directory path'
      MAPS_DIR="$2"
      shift 2
      ;;
    --occ)
      [[ $# -ge 2 ]] || die '--occ requires a threshold value'
      OCC_THRESH="$2"
      shift 2
      ;;
    --free)
      [[ $# -ge 2 ]] || die '--free requires a threshold value'
      FREE_THRESH="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if [[ "$1" =~ ^- ]]; then
        die "unknown option: $1 (run with -h for help)"
      else
        MAP_NAME="$1"
        shift
      fi
      ;;
  esac
done

mkdir -p "$MAPS_DIR"
OUTPUT_PREFIX="${MAPS_DIR}/${MAP_NAME}"

info "Tiến hành lưu bản đồ Nav2 tới: ${OUTPUT_PREFIX}.yaml"

if command -v ros2 >/dev/null 2>&1; then
  source_ros_setup jazzy
  ros2 run nav2_map_server map_saver_cli -f "$OUTPUT_PREFIX" --occ "$OCC_THRESH" --free "$FREE_THRESH"
  info "Đã lưu bản đồ thành công tại ${OUTPUT_PREFIX}.yaml và ${OUTPUT_PREFIX}.pgm"
else
  die "Lệnh 'ros2' không tồn tại trên môi trường hiện tại."
fi
