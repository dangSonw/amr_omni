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
ROS_WORKSPACE="/opt/ros1_ws"
ROS_PACKAGE="amr_bringup"
ROS_LAUNCH="robot.launch"

usage() {
  cat <<'HELP_EOF'
Usage: scripts/install.sh [options] [target]

Unified installer for AMR Omni robot hardware rules and systemd daemons.

Targets:
  all         Install both persistent udev rules and systemd service (default)
  udev        Install /etc/udev/rules.d/99-amr-omni.rules (symlinks: /dev/amr_*)
  service     Install /etc/systemd/system/amr_robot.service for autostart

Options:
  --user USER     System user for systemd service execution (default: current user)
  --dry-run       Print generated configurations without writing to /etc
  --uninstall     Remove installed udev rules and systemd service
  -h, --help      Show this help message

Examples:
  scripts/install.sh udev
  scripts/install.sh service --user robot
  scripts/install.sh all
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
    --ros-package)
      [[ $# -ge 2 ]] || die '--ros-package requires a package name'
      ROS_PACKAGE="$2"
      shift 2
      ;;
    --ros-launch)
      [[ $# -ge 2 ]] || die '--ros-launch requires a launch file'
      ROS_LAUNCH="$2"
      shift 2
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
    all|udev|service)
      TARGET="$1"
      shift
      ;;
    *)
      die "unknown option or target: $1 (run with -h for help)"
      ;;
  esac
done

install_udev() {
  local target_file="/etc/udev/rules.d/99-amr-omni.rules"
  info "Cấu hình udev rules cho cổng thiết bị USB cố định (/dev/amr_*)..."

  local rules_content
  rules_content=$(cat <<'RULES'
# AMR Omni persistent USB device symlinks
# STM32 F407 Virtual COM Port
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", SYMLINK+="amr_mcu"
# Silicon Labs CP2102 USB-to-UART (RPLiDAR A1/A2)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="amr_lidar"
# QinHeng Electronics CH340 / FTDI (IMU USB adapter)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", SYMLINK+="amr_imu"
RULES
)

  if [[ "$UNINSTALL" == true ]]; then
    if [[ -f "$target_file" ]]; then
      info "Đang gỡ bỏ $target_file..."
      sudo rm -f "$target_file"
      sudo udevadm control --reload-rules && sudo udevadm trigger || true
      info "Đã gỡ bỏ udev rules."
    fi
    return 0
  fi

  if [[ "$DRY_RUN" == true ]]; then
    info "[DRY-RUN] Nội dung $target_file:"
    printf '%s\n' "$rules_content"
    return 0
  fi

  printf '%s\n' "$rules_content" | sudo tee "$target_file" >/dev/null
  sudo udevadm control --reload-rules && sudo udevadm trigger || true
  info "Đã cài đặt udev rules thành công: ${target_file}"
}

install_service() {
  local target_file="/etc/systemd/system/amr_robot.service"
  info "Cấu hình systemd daemon tự khởi động robot (amr_robot.service)..."

  local service_content
  service_content=$(cat <<SERVICE
[Unit]
Description=AMR Omni Autonomous Mobile Robot Autostart Service
After=network.target roscore.service
Wants=network.target

[Service]
Type=simple
User=${RUN_USER}
WorkingDirectory=${ROOT_DIR}
Environment=HOME=/home/${RUN_USER}
EnvironmentFile=-/etc/amr_omni/robot.conf
ExecStart=${ROOT_DIR}/scripts/run_robot.sh --workspace ${ROS_WORKSPACE} --package ${ROS_PACKAGE} --launch ${ROS_LAUNCH}
Restart=on-failure
RestartSec=5s
KillMode=mixed
TimeoutStopSec=10s

[Install]
WantedBy=multi-user.target
SERVICE
)

  if [[ "$UNINSTALL" == true ]]; then
    info "Đang dừng và gỡ bỏ amr_robot.service..."
    sudo systemctl stop amr_robot.service 2>/dev/null || true
    sudo systemctl disable amr_robot.service 2>/dev/null || true
    sudo rm -f "$target_file"
    sudo systemctl daemon-reload || true
    info "Đã gỡ bỏ systemd service."
    return 0
  fi

  if [[ "$DRY_RUN" == true ]]; then
    info "[DRY-RUN] Nội dung $target_file:"
    printf '%s\n' "$service_content"
    return 0
  fi

  printf '%s\n' "$service_content" | sudo tee "$target_file" >/dev/null
  sudo systemctl daemon-reload
  sudo systemctl enable amr_robot.service
  info "Đã cài đặt và kích hoạt dịch vụ tự khởi động: amr_robot.service"
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
esac

info "Quy trình cài đặt hoàn tất."
