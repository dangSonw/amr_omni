#!/usr/bin/env bash
# scripts/bag.sh
# Unified rosbag record, play, and info utility for AMR Omni (ROS 2 Jazzy & ROS 1 Melodic)

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
ROOT_DIR="$(get_repo_root)"
BAGS_DIR="${ROOT_DIR}/bags"

SUBCOMMAND="${1:-help}"
shift || true

usage() {
  cat <<'HELP_EOF'
Usage: scripts/bag.sh <command> [options]

Unified rosbag management utility for AMR Omni.

Commands:
  record [name]   Record essential sensor, TF, and velocity topics to rosbag
  play <path>     Play back recorded rosbag file or directory
  info <path>     Display metadata and topic statistics for a bag file
  -h, --help      Show this help message

Options for 'record':
  --output NAME   Custom bag output name (default: amr_session_<timestamp>)
  --duration SEC  Stop recording automatically after SEC seconds
  --all-topics    Record all topics instead of filtered essential topics

Options for 'play':
  --rate FACTOR   Playback rate multiplier (e.g. 0.5, 1.0, 2.0; default: 1.0)
  --loop          Loop playback continuously
  --clock         Publish simulation clock (default: true)

Examples:
  scripts/bag.sh record
  scripts/bag.sh record test_run_01 --duration 30
  scripts/bag.sh play bags/test_run_01 --rate 1.5
  scripts/bag.sh info bags/test_run_01
HELP_EOF
}

bag_record() {
  local bag_name=""
  local duration=""
  local all_topics=false

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --output)
        [[ $# -ge 2 ]] || die '--output requires a bag name'
        bag_name="$2"
        shift 2
        ;;
      --duration)
        [[ $# -ge 2 ]] || die '--duration requires seconds'
        duration="$2"
        shift 2
        ;;
      --all-topics)
        all_topics=true
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        if [[ -z "$bag_name" && ! "$1" =~ ^- ]]; then
          bag_name="$1"
          shift
        else
          die "unknown record option: $1"
        fi
        ;;
    esac
  done

  mkdir -p "$BAGS_DIR"
  local timestamp
  timestamp="$(date +'%Y-%m-%d_%H-%M-%S')"
  bag_name="${bag_name:-amr_session_${timestamp}}"
  local out_path="${BAGS_DIR}/${bag_name}"

  info "Bắt đầu ghi rosbag tại: ${out_path}"

  local -a topics=(
    /cmd_vel /safe_cmd_vel /odom /imu/data_raw /scan
    /tf /tf_static /wheel_state /diagnostics
  )

  if command -v ros2 >/dev/null 2>&1; then
    source_ros_setup jazzy
    local -a cmd=(ros2 bag record -o "$out_path")
    if [[ "$all_topics" == true ]]; then
      cmd+=(-a)
    else
      cmd+=("${topics[@]}")
    fi
    if [[ -n "$duration" ]]; then
      timeout "${duration}s" "${cmd[@]}" || true
    else
      "${cmd[@]}"
    fi
  elif command -v rosbag >/dev/null 2>&1; then
    source_ros_setup melodic
    local -a cmd=(rosbag record -O "${out_path}.bag")
    if [[ "$all_topics" == true ]]; then
      cmd+=(-a)
    else
      cmd+=("${topics[@]}")
    fi
    if [[ -n "$duration" ]]; then
      timeout "${duration}s" "${cmd[@]}" || true
    else
      "${cmd[@]}"
    fi
  else
    die "Không tìm thấy công cụ ros2 bag hoặc rosbag."
  fi
  info "Đã lưu rosbag tại: ${out_path}"
}

bag_play() {
  local target_bag=""
  local rate="1.0"
  local loop=false
  local clock=true

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --rate)
        [[ $# -ge 2 ]] || die '--rate requires a float multiplier'
        rate="$2"
        shift 2
        ;;
      --loop)
        loop=true
        shift
        ;;
      --clock)
        clock=true
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        if [[ -z "$target_bag" && ! "$1" =~ ^- ]]; then
          target_bag="$1"
          shift
        else
          die "unknown play option: $1"
        fi
        ;;
    esac
  done

  [[ -n "$target_bag" ]] || die "Chưa chỉ định đường dẫn file bag để phát (Usage: scripts/bag.sh play <path>)"
  [[ -e "$target_bag" ]] || die "File hoặc thư mục bag không tồn tại: ${target_bag}"

  info "Đang phát lại rosbag: ${target_bag} (Rate: ${rate}x)"

  if command -v ros2 >/dev/null 2>&1; then
    source_ros_setup jazzy
    local -a cmd=(ros2 bag play "$target_bag" -r "$rate")
    [[ "$clock" == true ]] && cmd+=(--clock)
    [[ "$loop" == true ]] && cmd+=(--loop)
    "${cmd[@]}"
  elif command -v rosbag >/dev/null 2>&1; then
    source_ros_setup melodic
    local -a cmd=(rosbag play "$target_bag" -r "$rate")
    [[ "$clock" == true ]] && cmd+=(--clock)
    [[ "$loop" == true ]] && cmd+=(-l)
    "${cmd[@]}"
  else
    die "Không tìm thấy công cụ ros2 bag hoặc rosbag."
  fi
}

bag_info() {
  local target_bag="${1:-}"
  [[ -n "$target_bag" ]] || die "Chưa chỉ định đường dẫn file bag (Usage: scripts/bag.sh info <path>)"
  [[ -e "$target_bag" ]] || die "File hoặc thư mục bag không tồn tại: ${target_bag}"

  if command -v ros2 >/dev/null 2>&1; then
    source_ros_setup jazzy
    ros2 bag info "$target_bag"
  elif command -v rosbag >/dev/null 2>&1; then
    source_ros_setup melodic
    rosbag info "$target_bag"
  else
    die "Không tìm thấy công cụ ros2 bag hoặc rosbag."
  fi
}

case "$SUBCOMMAND" in
  record)
    bag_record "$@"
    ;;
  play)
    bag_play "$@"
    ;;
  info)
    bag_info "$@"
    ;;
  -h|--help|help)
    usage
    exit 0
    ;;
  *)
    die "Lệnh không hợp lệ: ${SUBCOMMAND}. Chạy 'scripts/bag.sh --help' để xem hướng dẫn."
    ;;
esac
