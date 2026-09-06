#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
MCU=""
DEVICE=""
FIRMWARE=""
PROJECT=""
ENVIRONMENT=""
BACKEND="auto"
CONFIRMED=false
DRY_RUN=false
RESET=false

usage() {
  cat <<'EOF'
Usage: scripts/flash_mcu.sh --mcu stm32f1|stm32f4 --device DEVICE --firmware FILE [options]

Flash one explicitly selected STM32 target. This command is intentionally
conservative: it validates all inputs, refuses to run without --yes, and never
uses sudo. PlatformIO is the preferred backend; openocd, st-flash and dfu-util
are available when explicitly selected and installed.

Options:
  --mcu stm32f1|stm32f4   MCU family confirmation (required)
  --device SELECTOR        Upload port, probe serial, or DFU VID:PID (required)
  --firmware FILE          .elf, .hex or .bin firmware (required)
  --project NAME|PATH      PlatformIO project (enables pio backend)
  --environment ENV        PlatformIO environment (default: project default)
  --backend auto|pio|openocd|st-flash|dfu-util
  --reset                  Request reset after a successful flash when supported
  --yes                    Confirm the destructive operation
  --dry-run                Validate and print command without flashing
  -h, --help               Show this help

Examples:
  scripts/flash_mcu.sh --mcu stm32f4 --device /dev/serial/by-id/... \
    --firmware firmware.elf --backend pio --project stm32_f407vg_arduino_sim --dry-run
  scripts/flash_mcu.sh --mcu stm32f4 --device /dev/serial/by-id/... \
    --firmware firmware.elf --backend pio --project stm32_f407vg_arduino_sim --yes
  scripts/flash_mcu.sh --mcu stm32f4 --device default --firmware app.elf \
    --backend openocd --yes
  scripts/flash_mcu.sh --mcu stm32f4 --device STLINK_SERIAL --firmware app.bin \
    --backend st-flash --yes
  scripts/flash_mcu.sh --mcu stm32f4 --device 0483:df11 --firmware app.bin \
    --backend dfu-util --yes
EOF
}

die() {
  echo "flash_mcu.sh: $*" >&2
  exit 1
}

info() {
  echo "[INFO] $*"
}

resolve_path() {
  local requested="$1"
  if [[ "$requested" = /* ]]; then
    printf '%s\n' "$requested"
  else
    printf '%s\n' "${ROOT_DIR}/${requested}"
  fi
}

command_exists() {
  command -v "$1" >/dev/null 2>&1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mcu)
      [[ $# -ge 2 ]] || die '--mcu requires stm32f1 or stm32f4'
      MCU="$2"
      shift 2
      ;;
    --device)
      [[ $# -ge 2 ]] || die '--device requires a path'
      DEVICE="$2"
      shift 2
      ;;
    --firmware)
      [[ $# -ge 2 ]] || die '--firmware requires a file'
      FIRMWARE="$2"
      shift 2
      ;;
    --project)
      [[ $# -ge 2 ]] || die '--project requires NAME or PATH'
      PROJECT="$2"
      shift 2
      ;;
    --environment|--env)
      [[ $# -ge 2 ]] || die '--environment requires a value'
      ENVIRONMENT="$2"
      shift 2
      ;;
    --backend)
      [[ $# -ge 2 ]] || die '--backend requires a value'
      BACKEND="$2"
      shift 2
      ;;
    --reset)
      RESET=true
      shift
      ;;
    --yes)
      CONFIRMED=true
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
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

[[ "$MCU" == stm32f1 || "$MCU" == stm32f4 ]] || die '--mcu must be stm32f1 or stm32f4'
[[ -n "$DEVICE" ]] || die '--device is required'
[[ -n "$FIRMWARE" ]] || die '--firmware is required'
FIRMWARE="$(resolve_path "$FIRMWARE")"
[[ -f "$FIRMWARE" ]] || die "firmware file does not exist: ${FIRMWARE}"
[[ "$FIRMWARE" =~ \.(elf|hex|bin)$ ]] || die 'firmware must have .elf, .hex or .bin extension'
[[ "$CONFIRMED" == true || "$DRY_RUN" == true ]] || \
  die 'refusing to flash without --yes; verify MCU, device and firmware first'

if [[ -n "$PROJECT" ]]; then
  if [[ -d "$PROJECT" ]]; then
    PROJECT="$(cd "$PROJECT" && pwd)"
  else
    PROJECT="${ROOT_DIR}/firmware/${PROJECT}"
  fi
  [[ -f "${PROJECT}/platformio.ini" ]] || die "PlatformIO project not found: ${PROJECT}"
fi

case "$BACKEND" in
  auto)
    if [[ -n "$PROJECT" ]]; then
      BACKEND=pio
    else
      die 'select --backend and --project for PlatformIO, or choose openocd, st-flash or dfu-util'
    fi
    ;;
  pio|openocd|st-flash|dfu-util) ;;
  *) die "invalid backend: ${BACKEND}" ;;
esac

command=()
case "$BACKEND" in
  pio)
    command_exists pio || command_exists platformio || die 'PlatformIO is not installed'
    [[ -n "$PROJECT" ]] || die '--project is required with --backend pio'
    pio_bin=pio
    command_exists pio || pio_bin=platformio
    command=("$pio_bin" run --project-dir "$PROJECT" --target upload --upload-port "$DEVICE")
    [[ -z "$ENVIRONMENT" ]] || command+=(--environment "$ENVIRONMENT")
    ;;
  openocd)
    command_exists openocd || die 'openocd is not installed (run setup.sh --install-flash-tools --yes)'
    [[ "$FIRMWARE" == *.elf ]] || die 'openocd backend requires an ELF firmware file'
    openocd_target=target/stm32f4x.cfg
    [[ "$MCU" == stm32f1 ]] && openocd_target=target/stm32f1x.cfg
    command=(openocd -f interface/stlink.cfg -f "$openocd_target")
    [[ "$DEVICE" == default ]] || command+=(-c "hla_serial ${DEVICE}")
    command+=(-c "program ${FIRMWARE} verify reset exit")
    ;;
  st-flash)
    command_exists st-flash || die 'st-flash is not installed (run setup.sh --install-flash-tools --yes)'
    [[ "$FIRMWARE" == *.bin ]] || die 'st-flash backend requires a .bin firmware file'
    command=(st-flash --serial "$DEVICE" --connect-under-reset write "$FIRMWARE" 0x08000000)
    ;;
  dfu-util)
    command_exists dfu-util || die 'dfu-util is not installed (run setup.sh --install-flash-tools --yes)'
    [[ "$FIRMWARE" == *.bin ]] || die 'dfu-util backend requires a .bin firmware file'
    command=(dfu-util --device "$DEVICE" --download "$FIRMWARE")
    ;;
esac

case "$BACKEND" in
  pio|st-flash|openocd)
    [[ -e "$DEVICE" || "$DEVICE" != *:* ]] || die "invalid device selector: ${DEVICE}"
    ;;
  dfu-util)
    [[ "$DEVICE" =~ ^[0-9a-fA-F]{4}:[0-9a-fA-F]{4}([/][0-9a-fA-F]+)?$ || -e "$DEVICE" ]] || \
      die 'dfu-util --device must be VID:PID[/serial] or an existing device path'
    ;;
esac

info "MCU=${MCU} device=${DEVICE} firmware=${FIRMWARE} backend=${BACKEND}"
if [[ "$DRY_RUN" == true ]]; then
  printf '[DRY-RUN]'
  printf ' %q' "${command[@]}"
  printf '\n'
  exit 0
fi
info 'flashing target; keep E-stop available and do not disconnect power.'
"${command[@]}"
if [[ "$RESET" == true ]]; then
  case "$BACKEND" in
    openocd) info 'reset is included in the openocd command.' ;;
    st-flash|dfu-util|pio) info 'reset request depends on the selected programmer/board.' ;;
  esac
fi
info 'flash command completed.'
