#!/usr/bin/env bash
# scripts/install.sh
# Unified installation utility for AMR Omni (udev rules & systemd autostart service)

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
ROOT_DIR="$(get_repo_root)"

TARGET="all"
DRY_RUN=false
UNINSTALL=false
RUN_USER="${USER:-sonev}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_WORKSPACE="${ROOT_DIR}"
AGENT_ENABLED=true
WEB_ENABLED=true

usage() {
  cat <<'HELP_EOF'
Usage: scripts/install.sh [options] [target]

Unified installer for AMR Omni robot hardware rules and systemd daemons.

Targets:
  all         Install persistent udev rules and systemd service (default)
  udev        Install udev rules (/etc/udev/rules.d/99-amr-*.rules)
  service     Install systemd service (/etc/systemd/system/amr-bringup.service)
  status      Check current status of udev symlinks and systemd service

Options:
  --user USER          System user for systemd service (default: current user)
  --ros-distro DISTRO  ROS 2 distribution (default: jazzy)
  --ros-workspace PATH Path to AMR Omni workspace root (default: repo root)
  --no-agent           Disable auto-starting micro-ROS agent in systemd service
  --no-web             Disable auto-starting web backend in systemd service
  --dry-run            Print generated configurations without writing to /etc
  --uninstall          Remove installed udev rules and systemd service
  -h, --help           Show this help message

Examples:
  scripts/install.sh udev
  scripts/install.sh service --user robot
  scripts/install.sh all
  scripts/install.sh status
  scripts/install.sh --uninstall
HELP_EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-workspace)
      [[ $# -ge 2 ]] || die '--ros-workspace requires a path'
      ROS_WORKSPACE="$2"
      shift 2
      ;;
    --ros-distro)
      [[ $# -ge 2 ]] || die '--ros-distro requires a distribution name'
      ROS_DISTRO="$2"
      shift 2
      ;;
    --no-agent)
      AGENT_ENABLED=false
      shift
      ;;
    --no-web)
      WEB_ENABLED=false
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --uninstall)
      UNINSTALL=true
      shift
      ;;
    --user)
      [[ $# -ge 2 ]] || die '--user requires a username'
      RUN_USER="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    all|udev|service|status)
      TARGET="$1"
      shift
      ;;
    *)
      die "unknown option or target: $1 (run with -h for help)"
      ;;
  esac
done

install_udev() {
  local stm32_rule_src="${ROOT_DIR}/udev/99-amr-stm32.rules"
  local lidar_rule_src="${ROOT_DIR}/udev/99-amr-lidar.rules"
  local dest_dir="/etc/udev/rules.d"

  if [[ "$UNINSTALL" == true ]]; then
    info "Đang gỡ bỏ udev rules trong ${dest_dir}..."
    if [[ "$DRY_RUN" == true ]]; then
      info "[DRY-RUN] Sẽ xóa ${dest_dir}/99-amr-stm32.rules và 99-amr-lidar.rules"
    else
      sudo rm -f "${dest_dir}/99-amr-stm32.rules" "${dest_dir}/99-amr-lidar.rules"
      sudo udevadm control --reload-rules && sudo udevadm trigger || true
      info "Đã gỡ bỏ udev rules."
    fi
    return 0
  fi

  info "Cài đặt udev rules cho STM32 (/dev/stm32) và LiDAR (/dev/lidar)..."
  if [[ "$DRY_RUN" == true ]]; then
    info "[DRY-RUN] Copy ${stm32_rule_src} -> ${dest_dir}/99-amr-stm32.rules"
    info "[DRY-RUN] Copy ${lidar_rule_src} -> ${dest_dir}/99-amr-lidar.rules"
    return 0
  fi

  sudo cp "${stm32_rule_src}" "${dest_dir}/99-amr-stm32.rules"
  sudo cp "${lidar_rule_src}" "${dest_dir}/99-amr-lidar.rules"
  sudo chmod 644 "${dest_dir}/99-amr-stm32.rules" "${dest_dir}/99-amr-lidar.rules"
  sudo udevadm control --reload-rules && sudo udevadm trigger || true
  info "Đã cài đặt udev rules thành công: /dev/stm32, /dev/lidar"
}

install_service() {
  local target_file="/etc/systemd/system/amr-bringup.service"
  info "Cấu hình systemd daemon tự khởi động robot (amr-bringup.service)..."

  local extra_args=""
  if [[ "$AGENT_ENABLED" == false ]]; then
    extra_args="${extra_args} agent:=false"
  fi
  if [[ "$WEB_ENABLED" == false ]]; then
    extra_args="${extra_args} web:=false"
  fi

  local service_content
  service_content=$(cat <<SERVICE
[Unit]
Description=AMR Omni Autonomous Mobile Robot Production Bringup
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${RUN_USER}
Group=${RUN_USER}
WorkingDirectory=${ROS_WORKSPACE}
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
Environment="ROS_LOG_DIR=/home/${RUN_USER}/.ros/log"
ExecStart=/bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && source ${ROS_WORKSPACE}/install/setup.bash && ros2 launch omni_bringup real_robot_bringup.launch.py${extra_args}"
Restart=on-failure
RestartSec=5
StartLimitIntervalSec=60
StartLimitBurst=5
LimitNOFILE=65536
KillMode=mixed
TimeoutStopSec=10s
StandardOutput=journal
StandardError=journal
SyslogIdentifier=amr-omni

[Install]
WantedBy=multi-user.target
SERVICE
)

  if [[ "$UNINSTALL" == true ]]; then
    info "Đang dừng và gỡ bỏ amr-bringup.service..."
    if [[ "$DRY_RUN" == true ]]; then
      info "[DRY-RUN] Sẽ dừng, disable và xóa ${target_file}"
    else
      sudo systemctl stop amr-bringup.service 2>/dev/null || true
      sudo systemctl disable amr-bringup.service 2>/dev/null || true
      sudo rm -f "$target_file"
      sudo systemctl daemon-reload || true
      info "Đã gỡ bỏ systemd service."
    fi
    return 0
  fi

  if [[ "$DRY_RUN" == true ]]; then
    info "[DRY-RUN] Nội dung $target_file:"
    printf '%s\n' "$service_content"
    return 0
  fi

  printf '%s\n' "$service_content" | sudo tee "$target_file" >/dev/null
  sudo systemctl daemon-reload
  sudo systemctl enable amr-bringup.service
  info "Đã cài đặt và kích hoạt dịch vụ tự khởi động: amr-bringup.service"
}

check_status() {
  info "=== KIỂM TRA TRẠNG THÁI THIẾT BỊ VÀ DỊCH VỤ ==="
  echo ""
  info "1. Cổng thiết bị USB cố định (udev symlinks):"
  if [[ -e /dev/stm32 ]]; then
    echo "  [OK] /dev/stm32 -> $(readlink -f /dev/stm32)"
  else
    echo "  [FAIL] /dev/stm32 chưa được nhận diện"
  fi
  if [[ -e /dev/lidar ]]; then
    echo "  [OK] /dev/lidar -> $(readlink -f /dev/lidar)"
  else
    echo "  [FAIL] /dev/lidar chưa được nhận diện"
  fi

  echo ""
  info "2. Dịch vụ systemd autostart (amr-bringup.service):"
  if systemctl is-active --quiet amr-bringup.service 2>/dev/null; then
    echo "  [ACTIVE] amr-bringup.service đang chạy"
  elif systemctl is-enabled --quiet amr-bringup.service 2>/dev/null; then
    echo "  [ENABLED] amr-bringup.service đã kích hoạt (chờ khởi động)"
  else
    echo "  [INACTIVE] amr-bringup.service chưa cài đặt hoặc đang tắt"
  fi
}

case "$TARGET" in
  udev)
    install_udev
    ;;
  service)
    install_service
    ;;
  all)
    install_udev
    install_service
    ;;
  status)
    check_status
    ;;
esac

info "Thao tác ${TARGET} hoàn tất."
