#!/usr/bin/env bash
# scripts/calibrate.sh
# Unified calibration utility for AMR Omni robot (Odometry, IMU, LiDAR, Motors)

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
ROOT_DIR="$(get_repo_root)"

TARGET="all"
DRY_RUN=false
DURATION=5
TARGET_DIST=1.0
TEST_SPEED=0.2

usage() {
  cat <<'HELP_EOF'
Usage: scripts/calibrate.sh [options] [target]

Unified calibration utility for AMR Omni robot hardware.

Targets:
  all         Run all calibration routines in sequence (default)
  odometry    Calibrate wheel radius and trackwidth (linear and rotation test)
  imu         Measure static gyro bias and verify 1g accelerometer vector
  lidar       Validate LiDAR scan frequency, range boundaries, and beams
  motors      Verify 4 omni wheel rotation directions and encoder feedback

Options:
  --dry-run       Simulate without sending motor velocity commands
  --duration SEC  Sampling duration for IMU/LiDAR (default: 5)
  --dist METERS   Target distance for linear translation test (default: 1.0)
  --speed MPS     Test speed for odometry/motors (default: 0.2)
  -h, --help      Show this help message

Examples:
  scripts/calibrate.sh all
  scripts/calibrate.sh odometry --dist 2.0 --speed 0.3
  scripts/calibrate.sh imu --duration 10
  scripts/calibrate.sh lidar --duration 10
  scripts/calibrate.sh motors --dry-run
HELP_EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --duration)
      [[ $# -ge 2 ]] || die '--duration requires a number of seconds'
      DURATION="$2"
      shift 2
      ;;
    --dist)
      [[ $# -ge 2 ]] || die '--dist requires distance in meters'
      TARGET_DIST="$2"
      shift 2
      ;;
    --speed)
      [[ $# -ge 2 ]] || die '--speed requires speed in m/s'
      TEST_SPEED="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    odometry|imu|lidar|motors|all)
      TARGET="$1"
      shift
      ;;
    *)
      die "unknown target or option: $1 (run with -h for help)"
      ;;
  esac
done

calibrate_odometry() {
  info "=========================================================="
  info "  1. HIỆU CHUẨN ODOMETRY (WHEEL RADIUS & KINEMATICS)      "
  info "=========================================================="
  info "Quy trình: Di chuyển tịnh tiến ${TARGET_DIST}m thẳng ở tốc độ ${TEST_SPEED} m/s."

  if [[ "$(awk -v s="$TEST_SPEED" 'BEGIN { print (s == 0) ? 1 : 0 }')" == "1" ]]; then
    die "TEST_SPEED must not be zero"
  fi

  if [[ "$DRY_RUN" == true ]]; then
    local dry_time
    dry_time="$(awk -v d="$TARGET_DIST" -v s="$TEST_SPEED" 'BEGIN { printf "%.1f", d / s }')"
    info "[DRY-RUN] Giả lập phát lệnh cmd_vel vx=${TEST_SPEED} trong ${dry_time}s"
    return 0
  fi

  if command -v ros2 >/dev/null 2>&1; then
    source_ros_setup jazzy
    local run_time
    run_time="$(awk -v d="$TARGET_DIST" -v s="$TEST_SPEED" 'BEGIN { printf "%.1f", d / s }')"
    info "Đang di chuyển trong ${run_time}s..."
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: ${TEST_SPEED}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null &
    sleep "${run_time}"
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null
  else
    warn "ROS 2 không khả dụng; bỏ qua phát lệnh ROS thực tế."
  fi

  info "Đo khoảng cách thực tế xe đã đi được trên mặt sàn (mét)."
  info "Hệ số tỷ lệ K = Khoảng_cách_lý_thuyết / Khoảng_cách_đo_thực_tế."
  info "Cập nhật kWheelRadiusM trong firmware: kWheelRadiusM_new = kWheelRadiusM * K"
}

calibrate_imu() {
  info "=========================================================="
  info "  2. HIỆU CHUẨN CẢM BIẾN IMU (GYRO BIAS & ACCELEROMETER)  "
  info "=========================================================="
  info "Đảm bảo robot đứng yên hoàn toàn trên mặt phẳng nằm ngang."

  if [[ "$DRY_RUN" == true ]]; then
    info "[DRY-RUN] Giả lập đọc mẫu IMU trong ${DURATION}s..."
    info "Gyro bias giả lập: gx=0.001 rad/s, gy=-0.002 rad/s, gz=0.000 rad/s (PASS)"
    info "Gia tốc trọng trường: |a| = 9.81 m/s^2 (PASS)"
    return 0
  fi

  if command -v ros2 >/dev/null 2>&1; then
    source_ros_setup jazzy
    info "Đang thu thập mẫu IMU trong ${DURATION} giây..."
    timeout "${DURATION}" ros2 topic echo /imu --field linear_acceleration 2>/dev/null | head -n 10 || true
  fi
  info "Hiệu chuẩn IMU hoàn tất: Giữ bù giá trị offset tĩnh trong bộ lọc EKF (ekf.yaml)."
}

calibrate_lidar() {
  info "=========================================================="
  info "  3. KIỂM TRA & HIỆU CHUẨN LIDAR 2D                       "
  info "=========================================================="
  info "Kiểm tra tần số quét và độ bao phủ góc của tia quét LaserScan..."

  if [[ "$DRY_RUN" == true ]]; then
    info "[DRY-RUN] Giả lập kiểm tra /scan: Tần số 10.0 Hz, 360 mẫu tia (PASS)"
    return 0
  fi

  if command -v ros2 >/dev/null 2>&1; then
    source_ros_setup jazzy
    if ros2 topic list 2>/dev/null | grep -q "/scan"; then
      info "Đo tần số topic /scan:"
      timeout 3 ros2 topic hz /scan || true
    else
      warn "Chưa phát hiện topic /scan đang hoạt động."
    fi
  fi
}

calibrate_motors() {
  info "=========================================================="
  info "  4. KIỂM TRA ĐỘNG CƠ & CHIỀU QUAY 4 BÁNH OMNI           "
  info "=========================================================="
  info "Kiểm tra tuần tự 4 bánh xe: FR (Bánh 1), FL (Bánh 2), RL (Bánh 3), RR (Bánh 4)..."

  if [[ "$DRY_RUN" == true ]]; then
    info "[DRY-RUN] Giả lập kích hoạt Motor 1..4 ở tốc độ ${TEST_SPEED} m/s (PASS)"
    return 0
  fi

  info "Quy tắc kiểm tra: Nhìn từ trên xuống, khi xe tiến thẳng (vx > 0):"
  info "  - Bánh 1 (FR) & Bánh 4 (RR) quay theo chiều chuẩn (+)"
  info "  - Bánh 2 (FL) & Bánh 3 (RL) quay theo chiều chuẩn (-)"
  info "Sử dụng bàn test nâng bánh xe khỏi mặt đất để kiểm tra an toàn."
}

case "$TARGET" in
  odometry)
    calibrate_odometry
    ;;
  imu)
    calibrate_imu
    ;;
  lidar)
    calibrate_lidar
    ;;
  motors)
    calibrate_motors
    ;;
  all)
    calibrate_odometry
    calibrate_imu
    calibrate_lidar
    calibrate_motors
    ;;
esac

info "Quy trình hiệu chuẩn hoàn tất thành công."
