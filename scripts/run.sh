#!/usr/bin/env bash
# =============================================================================
# scripts/run.sh - Unified runner for AMR Omni
# Targets: sim (default), robot, web, bridge
# =============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
ROOT_DIR="$(get_repo_root)"

# -----------------------------------------------------------------------------
# MASTER USAGE / HELP
# -----------------------------------------------------------------------------
usage_master() {
  cat <<'EOF'
Usage: scripts/run.sh [target] [options...]

Unified runner for AMR Omni: Simulation, Real Robot Hardware, Web Dashboard, and Bridge.

Targets:
  sim         Launch Gazebo Harmonic / Renode simulation + Web Dashboard (default)
  robot       Launch real robot hardware bringup (micro-ROS agent + bridge + Web)
  web         Launch standalone Web interface (FastAPI backend + Next.js frontend)
  bridge      Launch ROS 1 <-> ROS 2 communication bridge

Quick Examples:
  scripts/run.sh sim                                    # Start Gazebo simulation + Web UI (:8000)
  scripts/run.sh sim --headless --no-web                # Headless Gazebo, no web UI
  scripts/run.sh robot                                  # Real robot bringup with STM32 agent & Web UI
  scripts/run.sh robot --agent-port /dev/ttyACM0        # Hardware bringup with custom serial port
  scripts/run.sh robot --slam --nav                     # Hardware bringup with SLAM & Nav2
  scripts/run.sh web --dev                              # Start Next.js hot-reload dev server (:3000)
  scripts/run.sh web --port 8080                        # Start production web on custom port
  scripts/run.sh bridge --master-uri http://nano:11311  # Bridge ROS 1 and ROS 2 topics

To view target-specific help and full options:
  scripts/run.sh sim --help
  scripts/run.sh robot --help
  scripts/run.sh web --help
  scripts/run.sh bridge --help
EOF
}

# =============================================================================
# TARGET: SIMULATION (Gazebo / Renode)
# =============================================================================
usage_sim() {
  cat <<'EOF'
Usage: scripts/run.sh sim [options] [-- ros2-launch-arguments]

Run the Gazebo simulation, an STM32 Renode model, or both. The default mode is
Gazebo. The Gazebo launch includes the ROS 2 STM32 bridge and STM32 firmware
model; this script does not build implicitly or open a real USB device.

Options:
  --mode gazebo|renode|both  Simulation backend (default: gazebo)
  --world PATH               Gazebo SDF world (default: amr_lab.sdf)
  --headless                 Start Gazebo server with off-screen rendering
  --use-sim-time BOOL        ROS use_sim_time value (default: true)
  --duration SEC             Stop the selected process after SEC seconds
  --web                      Start the FastAPI web control/monitoring backend (default)
  --no-web, --noweb          Do not start the web interface
  --web-port PORT            Port for web interface (default: 8000)
  --board NAME               Renode board shortcut (default: stm32f4_discovery)
  --renode-script PATH       Renode .resc file; overrides --board
  --firmware PATH            ELF firmware passed to the Renode script
  --renode-duration VALUE    Renode virtual duration for emulation RunFor
  --interactive              Keep Renode monitor interactive (requires a TTY)
  --skip-build-check         Do not require the ROS install overlay
  --dry-run                  Print commands without running them
  --ros-arg ARG              Add one ros2 launch argument (repeatable)
  -h, --help                 Show this help

Renode board shortcuts:
  stm32f4_discovery, stm32f103, stm32f746, stm32l072

Examples:
  scripts/run.sh sim                                    # Gazebo simulation + Web dashboard (:8000)
  scripts/run.sh sim --no-web                           # Simulation only, no web backend
  scripts/run.sh sim --web-port 8080                    # Serve web interface on custom port
  scripts/run.sh sim --mode gazebo --headless --world src/omni_simulation/worlds/amr_lab.sdf
  scripts/run.sh sim --mode renode --board stm32f4_discovery --renode-duration 5s
  scripts/run.sh sim --mode renode --renode-script ./my_board.resc --firmware ./build/app.elf
  scripts/run.sh sim --mode both --headless --duration 30
  scripts/run.sh sim -- --show-args
EOF
}

run_sim() {
  local ros_distro="${ROS_DISTRO:-jazzy}"
  local install_dir="${ROOT_DIR}/install/ros2_jazzy"
  local mode="gazebo"
  local world=""
  local headless=false
  local use_sim_time=true
  local duration=""
  local renode_board="stm32f4_discovery"
  local renode_script=""
  local renode_firmware=""
  local renode_duration=""
  local interactive=false
  local dry_run=false
  local skip_build_check=false
  local launch_web=true
  local web_port=8000
  local ros_launch_args=()
  local gazebo_command=()
  local renode_command=()
  local web_pid=""

  cleanup_sim_web() {
    if [[ -n "$web_pid" ]]; then
      kill -INT "$web_pid" 2>/dev/null || true
      wait "$web_pid" 2>/dev/null || true
    fi
  }

  resolve_path() {
    local requested="$1"
    if [[ "$requested" = /* ]]; then
      printf '%s\n' "$requested"
    else
      printf '%s\n' "${ROOT_DIR}/${requested}"
    fi
  }

  validate_duration() {
    [[ -z "$1" || "$1" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "invalid duration: $1"
    [[ "$1" != 0 && "$1" != 0.0 ]] || die 'duration must be greater than zero'
  }

  source_sim_ros2() {
    local setup_file="/opt/ros/${ros_distro}/setup.bash"
    [[ -f "$setup_file" ]] || die "ROS setup file not found: ${setup_file}"
    [[ "${ROS_VERSION:-}" != 1 ]] || die 'current shell contains ROS 1; open a clean shell.'
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
    [[ "${ROS_VERSION:-}" == 2 && "${ROS_DISTRO:-}" == "$ros_distro" ]] || \
      die "this script requires ROS 2 ${ros_distro}"
    [[ "$(uname -m)" == x86_64 ]] || die 'Gazebo simulation requires x86_64 WSL2.'
    command -v ros2 >/dev/null 2>&1 || die 'ros2 is not installed'
    if [[ -z "${ROS_LOG_DIR:-}" ]]; then
      if ! touch "${HOME}/.ros/log/.writable_test" 2>/dev/null; then
        export ROS_LOG_DIR="${ROOT_DIR}/.temp_ros_log"
        mkdir -p "$ROS_LOG_DIR"
      else
        rm -f "${HOME}/.ros/log/.writable_test"
      fi
    fi
  }

  renode_script_for_board() {
    case "$renode_board" in
      stm32f4_discovery)
        printf '%s\n' /opt/renode/scripts/single-node/stm32f4_discovery.resc
        ;;
      stm32f103)
        printf '%s\n' /opt/renode/scripts/single-node/stm32f103.resc
        ;;
      stm32f746)
        printf '%s\n' /opt/renode/scripts/single-node/stm32f746.resc
        ;;
      stm32l072)
        printf '%s\n' /opt/renode/scripts/single-node/stm32l072.resc
        ;;
      *)
        die "unknown Renode board: ${renode_board}"
        ;;
    esac
  }

  validate_renode() {
    command -v renode >/dev/null 2>&1 || die 'Renode is not installed'
    if [[ -z "$renode_script" ]]; then
      renode_script="$(renode_script_for_board)"
    else
      renode_script="$(resolve_path "$renode_script")"
    fi
    [[ -f "$renode_script" ]] || die "Renode script not found: ${renode_script}"
    if [[ -n "$renode_firmware" ]]; then
      renode_firmware="$(resolve_path "$renode_firmware")"
      [[ -f "$renode_firmware" ]] || die "firmware ELF not found: ${renode_firmware}"
    fi
    if [[ -n "$renode_duration" ]]; then
      [[ "$renode_duration" =~ ^[0-9]+([.][0-9]+)?(s|ms|us|ns)?$ ]] || \
        die "invalid Renode duration: ${renode_duration}"
    fi
  }

  run_sim_cmd() {
    if [[ "$dry_run" == true ]]; then
      printf '[DRY-RUN]'
      printf ' %q' "$@"
      printf '\n'
    else
      "$@"
    fi
  }

  run_for_real_seconds() {
    local dur="$1"
    shift
    set +e
    timeout --signal=INT --kill-after=10 "${dur}s" "$@"
    local status=$?
    set -e
    [[ "$status" == 124 ]] && return 0
    return "$status"
  }

  build_gazebo_command() {
    local world_file="$world"
    if [[ -z "$world_file" ]]; then
      world_file="${ROOT_DIR}/src/omni_simulation/worlds/amr_lab.sdf"
    else
      world_file="$(resolve_path "$world_file")"
    fi
    [[ -f "$world_file" ]] || die "Gazebo world not found: ${world_file}"
    [[ -f "${install_dir}/local_setup.bash" || "$skip_build_check" == true ]] || \
      die "ROS overlay is not built: ${install_dir}/local_setup.bash (run scripts/build.sh)"

    gazebo_command=(ros2 launch omni_bringup simulation_bringup.launch.py
      "world:=${world_file}" "use_sim_time:=${use_sim_time}"
      "headless:=${headless}")
    if [[ "$launch_web" == true ]]; then
      local py_bin
      py_bin="$(resolve_python_executable "$ROOT_DIR")"
      gazebo_command+=(
        "web:=true"
        "web_port:=${web_port}"
        "web_script:=${ROOT_DIR}/web/backend/run_backend.py"
        "python_bin:=${py_bin}"
      )
    else
      gazebo_command+=("web:=false")
    fi
    gazebo_command+=("${ros_launch_args[@]}")
  }

  exec_gazebo() {
    source_sim_ros2
    if [[ -f "${install_dir}/local_setup.bash" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "${install_dir}/local_setup.bash"
      set -u
    elif [[ "$skip_build_check" != true ]]; then
      die "ROS overlay is not built: ${install_dir}/local_setup.bash"
    fi
    build_gazebo_command
    info 'starting ROS 2 Gazebo simulation'
    if [[ "$launch_web" == true ]]; then
      info "Web interface enabled: http://localhost:${web_port}"
      info "Mẹo: Mở trình duyệt và nhấn Ctrl+Shift+R để tải giao diện mới nhất!"
    fi
    if [[ -n "$duration" ]]; then
      if [[ "$dry_run" == true ]]; then
        run_sim_cmd timeout --signal=INT --kill-after=10 "${duration}s" "${gazebo_command[@]}"
      else
        run_for_real_seconds "$duration" "${gazebo_command[@]}"
      fi
    else
      run_sim_cmd "${gazebo_command[@]}"
    fi
  }

  build_renode_command() {
    renode_command=(renode --disable-gui)
    if [[ "$interactive" == true ]]; then
      renode_command+=(--console)
    fi
    if [[ -n "$renode_firmware" ]]; then
      renode_command+=(-e "set bin @${renode_firmware}")
    fi
    renode_command+=(-e "include @${renode_script}")
    if [[ -n "$renode_duration" ]]; then
      renode_command+=(-e "emulation RunFor \"${renode_duration}\"" -e q)
    fi
  }

  exec_renode() {
    validate_renode
    if [[ "$interactive" == true && ! -t 0 ]]; then
      die '--interactive requires a terminal stdin (do not use it in a background/CI job)'
    fi
    build_renode_command
    info "starting Renode: ${renode_script}"
    if [[ -n "$duration" ]]; then
      if [[ "$dry_run" == true ]]; then
        run_sim_cmd timeout --signal=INT --kill-after=10 "${duration}s" "${renode_command[@]}"
      else
        run_for_real_seconds "$duration" "${renode_command[@]}"
      fi
    else
      run_sim_cmd "${renode_command[@]}"
    fi
  }

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --mode)
        [[ $# -ge 2 ]] || die '--mode requires gazebo, renode or both'
        mode="$2"
        shift 2
        ;;
      --world)
        [[ $# -ge 2 ]] || die '--world requires a path'
        world="$2"
        shift 2
        ;;
      --headless)
        headless=true
        shift
        ;;
      --use-sim-time)
        [[ $# -ge 2 ]] || die '--use-sim-time requires true or false'
        use_sim_time="$2"
        shift 2
        ;;
      --duration)
        [[ $# -ge 2 ]] || die '--duration requires seconds'
        duration="$2"
        shift 2
        ;;
      --board)
        [[ $# -ge 2 ]] || die '--board requires a name'
        renode_board="$2"
        shift 2
        ;;
      --renode-script)
        [[ $# -ge 2 ]] || die '--renode-script requires a path'
        renode_script="$2"
        shift 2
        ;;
      --firmware)
        [[ $# -ge 2 ]] || die '--firmware requires an ELF path'
        renode_firmware="$2"
        shift 2
        ;;
      --renode-duration)
        [[ $# -ge 2 ]] || die '--renode-duration requires a value'
        renode_duration="$2"
        shift 2
        ;;
      --interactive)
        interactive=true
        shift
        ;;
      --skip-build-check)
        skip_build_check=true
        shift
        ;;
      --dry-run)
        dry_run=true
        shift
        ;;
      --web)
        launch_web=true
        shift
        ;;
      --no-web|--noweb)
        launch_web=false
        shift
        ;;
      --web-port)
        [[ $# -ge 2 ]] || die '--web-port requires a port number'
        web_port="$2"
        shift 2
        ;;
      --ros-arg)
        [[ $# -ge 2 ]] || die '--ros-arg requires NAME:=VALUE'
        ros_launch_args+=("$2")
        shift 2
        ;;
      --)
        shift
        ros_launch_args+=("$@")
        break
        ;;
      -h|--help)
        usage_sim
        return 0
        ;;
      *)
        die "unknown sim option: $1 (use --ros-arg NAME:=VALUE or -- ...)"
        ;;
    esac
  done

  case "$mode" in
    gazebo|renode|both) ;;
    *) die "invalid --mode: ${mode}" ;;
  esac
  [[ "$use_sim_time" == true || "$use_sim_time" == false ]] || die '--use-sim-time must be true or false'
  validate_duration "$duration"

  local py_bin
  py_bin="$(resolve_python_executable "$ROOT_DIR")"
  local web_cmd=("$py_bin" "${ROOT_DIR}/web/backend/run_backend.py" --port "$web_port" --mode ros2)

  if [[ "$mode" == gazebo ]]; then
    exec_gazebo
  elif [[ "$mode" == renode ]]; then
    if [[ "$dry_run" == true && "$launch_web" == true ]]; then
      run_sim_cmd "${web_cmd[@]}"
    fi
    if [[ "$dry_run" != true && "$launch_web" == true ]]; then
      source_sim_ros2
      info "starting web backend on port ${web_port}..."
      "${web_cmd[@]}" &
      web_pid=$!
      trap cleanup_sim_web EXIT INT TERM
    fi
    exec_renode
  else
    source_sim_ros2
    if [[ -f "${install_dir}/local_setup.bash" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "${install_dir}/local_setup.bash"
      set -u
    elif [[ "$skip_build_check" != true ]]; then
      die "ROS overlay is not built: ${install_dir}/local_setup.bash"
    fi
    validate_renode
    if [[ "$interactive" == true && ! -t 0 ]]; then
      die '--interactive requires a terminal stdin (do not use it in a background/CI job)'
    fi
    [[ "$dry_run" == true || -n "$duration" ]] || \
      die '--mode both requires --duration so both processes can be cleaned up safely'
    build_gazebo_command
    local ros_cmd=("${gazebo_command[@]}")
    build_renode_command
    local ren_cmd=("${renode_command[@]}")
    info 'starting Gazebo and Renode together'
    if [[ "$dry_run" == true ]]; then
      run_sim_cmd "${ros_cmd[@]}"
      run_sim_cmd "${ren_cmd[@]}"
    else
      local pids=()
      cleanup_both() {
        local p
        for p in "${pids[@]:-}"; do
          kill -INT "$p" 2>/dev/null || true
        done
        wait 2>/dev/null || true
      }
      trap cleanup_both EXIT INT TERM
      if [[ -n "$duration" ]]; then
        timeout --signal=INT --kill-after=10 "${duration}s" "${ros_cmd[@]}" &
        pids+=("$!")
        timeout --signal=INT --kill-after=10 "${duration}s" "${ren_cmd[@]}" &
        pids+=("$!")
      else
        "${ros_cmd[@]}" &
        pids+=("$!")
        "${ren_cmd[@]}" &
        pids+=("$!")
      fi
      set +e
      wait "${pids[0]}"
      local status=$?
      wait "${pids[1]}" 2>/dev/null
      local other_status=$?
      set -e
      if [[ -n "$duration" ]]; then
        [[ "$status" == 124 ]] && status=0
        [[ "$other_status" == 124 ]] && other_status=0
      fi
      if [[ "$status" != 0 ]]; then
        exit "$status"
      fi
      [[ "$other_status" == 0 ]] || exit "$other_status"
    fi
  fi
}

# =============================================================================
# TARGET: REAL ROBOT HARDWARE BRINGUP
# =============================================================================
usage_robot() {
  cat <<'EOF'
Usage: scripts/run.sh robot [options]

Unified robot hardware bringup for AMR Omni.
Supports modern ROS 2 (default) on any Linux SBC/PC, as well as legacy ROS 1.

Options:
  --mode MODE            Operating mode: ros2 (default) or ros1
  --ros-distro DISTRO    ROS distribution (default: jazzy, or humble/melodic)
  --agent                Launch micro-ROS serial agent (default: true for ros2)
  --no-agent             Do not launch micro-ROS agent
  --agent-port PORT      Serial port for micro-ROS agent (default: /dev/stm32)
  --web                  Launch web backend dashboard (default: true for ros2)
  --no-web               Do not launch web backend
  --web-port PORT        Port for web interface (default: 8000)
  --slam                 Enable SLAM Toolbox online mapping
  --nav                  Enable Nav2 navigation stack
  --perception           Enable perception pipeline (camera/lidar processing)
  --dry-run              Print commands without starting processes
  --arg NAME:=VALUE      Pass additional ROS arguments (repeatable)
  -h, --help             Show this help

ROS 1 Legacy Options:
  --workspace PATH       Catkin workspace containing devel/setup.bash
  --package NAME         ROS 1 package name
  --launch FILE          ROS 1 launch file name
  --no-roscore           Do not start roscore (ROS 1 only)

Examples:
  scripts/run.sh robot                          # Start full ROS 2 hardware bringup + agent + web
  scripts/run.sh robot --agent-port /dev/ttyACM0
  scripts/run.sh robot --slam --nav             # Bringup with autonomous SLAM and Navigation
  scripts/run.sh robot --dry-run
  scripts/run.sh robot --mode ros1 --workspace /opt/ros1_ws --package omni_bringup_ros1 --launch robot.launch
EOF
}

run_robot() {
  local mode="ros2"
  local ros_distro="${ROS_DISTRO:-jazzy}"
  local dry_run=false
  local launch_web=true
  local web_port=8000
  local launch_agent=true
  local agent_port="/dev/stm32"
  local enable_slam=false
  local enable_nav=false
  local enable_perception=false
  local extra_args=()

  # ROS 1 Legacy variables
  local ros1_workspace=""
  local bringup_package=""
  local bringup_launch=""
  local start_roscore=true

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --mode)
        [[ $# -ge 2 ]] || die '--mode requires a mode (ros2 or ros1)'
        mode="$2"
        shift 2
        ;;
      --ros-distro)
        [[ $# -ge 2 ]] || die '--ros-distro requires a value'
        ros_distro="$2"
        shift 2
        ;;
      --agent)
        launch_agent=true
        shift
        ;;
      --no-agent)
        launch_agent=false
        shift
        ;;
      --agent-port)
        [[ $# -ge 2 ]] || die '--agent-port requires a port path'
        agent_port="$2"
        shift 2
        ;;
      --web)
        launch_web=true
        shift
        ;;
      --no-web)
        launch_web=false
        shift
        ;;
      --web-port)
        [[ $# -ge 2 ]] || die '--web-port requires a port number'
        web_port="$2"
        shift 2
        ;;
      --slam)
        enable_slam=true
        shift
        ;;
      --nav)
        enable_nav=true
        shift
        ;;
      --perception)
        enable_perception=true
        shift
        ;;
      --dry-run)
        dry_run=true
        shift
        ;;
      --arg|--ros-arg)
        [[ $# -ge 2 ]] || die '--arg requires NAME:=VALUE'
        extra_args+=("$2")
        shift 2
        ;;
      --workspace)
        [[ $# -ge 2 ]] || die '--workspace requires a path'
        ros1_workspace="$2"
        mode="ros1"
        shift 2
        ;;
      --package)
        [[ $# -ge 2 ]] || die '--package requires a ROS package'
        bringup_package="$2"
        mode="ros1"
        shift 2
        ;;
      --launch)
        [[ $# -ge 2 ]] || die '--launch requires a launch file'
        bringup_launch="$2"
        mode="ros1"
        shift 2
        ;;
      --no-roscore)
        start_roscore=false
        shift
        ;;
      -h|--help)
        usage_robot
        return 0
        ;;
      *)
        die "unknown robot option: $1 (run with -h for help)"
        ;;
    esac
  done

  # ==================== ROS 2 BRINGUP (DEFAULT) ====================
  if [[ "$mode" == "ros2" ]]; then
    local ros_setup="/opt/ros/${ros_distro}/setup.bash"
    local ws_setup="${ROOT_DIR}/install/setup.bash"

    [[ -f "$ros_setup" ]] || die "ROS 2 setup not found: ${ros_setup}"
    [[ -f "$ws_setup" ]] || die "Workspace install setup not found: ${ws_setup}. Run 'scripts/build.sh' first."

    local launch_cmd=(
      ros2 launch omni_bringup real_robot_bringup.launch.py
      "agent:=${launch_agent}"
      "agent_port:=${agent_port}"
      "web:=${launch_web}"
      "web_port:=${web_port}"
      "slam:=${enable_slam}"
      "nav:=${enable_nav}"
      "perception:=${enable_perception}"
    )

    if [[ ${#extra_args[@]} -gt 0 ]]; then
      launch_cmd+=("${extra_args[@]}")
    fi

    if [[ "$dry_run" == true ]]; then
      echo "[DRY-RUN] Environment: source ${ros_setup} && source ${ws_setup}"
      printf '[DRY-RUN] Command:'
      printf ' %q' "${launch_cmd[@]}"
      printf '\n'
      return 0
    fi

    set +u
    # shellcheck disable=SC1090
    source "$ros_setup"
    # shellcheck disable=SC1090
    source "$ws_setup"
    set -u

    info "Khởi động robot AMR Omni (ROS 2 ${ros_distro})..."
    info "micro-ROS agent: ${launch_agent} (port: ${agent_port})"
    info "Web backend: ${launch_web} (port: ${web_port})"
    exec "${launch_cmd[@]}"
  fi

  # ==================== ROS 1 LEGACY BRINGUP ====================
  if [[ "$mode" == "ros1" ]]; then
    [[ -n "$ros1_workspace" ]] || die '--workspace is required for ROS 1 mode'
    [[ -n "$bringup_package" ]] || die '--package is required for ROS 1 mode'
    [[ -n "$bringup_launch" ]] || die '--launch is required for ROS 1 mode'
    [[ -d "$ros1_workspace" ]] || die "workspace not found: ${ros1_workspace}"
    [[ -f "/opt/ros/${ros_distro}/setup.bash" ]] || die "ROS setup not found: /opt/ros/${ros_distro}/setup.bash"
    [[ -f "${ros1_workspace}/devel/setup.bash" ]] || die "catkin workspace is not built: ${ros1_workspace}/devel/setup.bash"

    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ros_distro}/setup.bash"
    # shellcheck disable=SC1090
    source "${ros1_workspace}/devel/setup.bash"
    set -u

    local roslaunch_cmd=(roslaunch "$bringup_package" "$bringup_launch" "${extra_args[@]}")
    local roscore_cmd=(roscore)
    local py_bin
    py_bin="$(resolve_python_executable "$ROOT_DIR")"
    local web_cmd=("$py_bin" "${ROOT_DIR}/web/backend/run_backend.py" --port "$web_port" --mode ros1)

    if [[ "$dry_run" == true ]]; then
      printf '[DRY-RUN]'
      printf ' %q' "${roscore_cmd[@]}"
      printf '\n[DRY-RUN]'
      printf ' %q' "${roslaunch_cmd[@]}"
      if [[ "$launch_web" == true ]]; then
        printf '\n[DRY-RUN]'
        printf ' %q' "${web_cmd[@]}"
      fi
      printf '\n'
      return 0
    fi

    local roscore_pid=""
    local web_robot_pid=""

    cleanup_ros1() {
      if [[ -n "$web_robot_pid" ]]; then
        info 'stopping web backend...'
        kill -INT "$web_robot_pid" 2>/dev/null || true
        wait "$web_robot_pid" 2>/dev/null || true
      fi
      if [[ -n "$roscore_pid" ]]; then
        info 'stopping roscore...'
        kill -INT "$roscore_pid" 2>/dev/null || true
        wait "$roscore_pid" 2>/dev/null || true
      fi
    }
    trap cleanup_ros1 EXIT INT TERM

    if [[ "$start_roscore" == true ]]; then
      info 'starting roscore; press Ctrl-C to stop the robot bringup.'
      "${roscore_cmd[@]}" &
      roscore_pid=$!
      sleep 2
    fi

    if [[ "$launch_web" == true ]]; then
      info "starting web backend on port ${web_port}..."
      "${web_cmd[@]}" &
      web_robot_pid=$!
      sleep 1
    fi

    set +e
    "${roslaunch_cmd[@]}"
    local launch_status=$?
    set -e

    cleanup_ros1
    trap - EXIT INT TERM
    return "$launch_status"
  fi
}

# =============================================================================
# TARGET: WEB COCKPIT (FastAPI + Next.js)
# =============================================================================
usage_web() {
  cat <<'EOF'
Usage: scripts/run.sh web [options]

Start the AMR Omni web interface (FastAPI backend + Next.js frontend).

Options:
  --port PORT        Port for backend web server (default: 8000)
  --host HOST        Host address to bind to (default: 0.0.0.0)
  --mode MODE        Bridge mode: auto, ros2, ros1 (default: auto)
  --dev              Run Next.js dev server on port 3000 alongside backend
  --build            Rebuild frontend static bundle before starting
  -h, --help         Show this help message

Examples:
  scripts/run.sh web                     # Start production backend + static UI (:8000)
  scripts/run.sh web --mode ros2         # Start with native ROS 2 bridge
  scripts/run.sh web --port 8080         # Bind backend to custom port 8080
  scripts/run.sh web --dev               # Start with Next.js hot-reload on :3000
  scripts/run.sh web --build             # Rebuild frontend bundle and serve
EOF
}

run_web() {
  local host="0.0.0.0"
  local port="8000"
  local mode="auto"
  local dev_mode=false
  local build_frontend=false

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --port)
        [[ $# -ge 2 ]] || die '--port requires a port number'
        port="$2"
        shift 2
        ;;
      --host)
        [[ $# -ge 2 ]] || die '--host requires a host address'
        host="$2"
        shift 2
        ;;
      --mode)
        [[ $# -ge 2 ]] || die '--mode requires a mode (auto, ros2, ros1)'
        mode="$2"
        shift 2
        ;;
      --dev)
        dev_mode=true
        shift
        ;;
      --build)
        build_frontend=true
        shift
        ;;
      -h|--help)
        usage_web
        return 0
        ;;
      *)
        die "unknown web option: $1"
        ;;
    esac
  done

  local py_bin="${ROOT_DIR}/web/backend/.venv/bin/python3"
  if [[ ! -f "$py_bin" ]]; then
    info "Virtual environment not found, creating at web/backend/.venv..."
    python3 -m venv "${ROOT_DIR}/web/backend/.venv"
    "${ROOT_DIR}/web/backend/.venv/bin/pip" install -r "${ROOT_DIR}/web/backend/requirements.txt"
  fi

  local static_index="${ROOT_DIR}/web/backend/static/index.html"
  if [[ "$build_frontend" == true || (! -f "$static_index" && "$dev_mode" != true) ]]; then
    info "Building Next.js frontend static bundle..."
    (
      cd "${ROOT_DIR}/web/frontend"
      npm run build
    )
    mkdir -p "${ROOT_DIR}/web/backend/static"
    cp -r "${ROOT_DIR}/web/frontend/out/"* "${ROOT_DIR}/web/backend/static/"
    info "Frontend static bundle copied to web/backend/static/"
  fi

  local pids=()
  cleanup_web_all() {
    info "Stopping web servers..."
    local p
    for p in "${pids[@]:-}"; do
      kill -INT "$p" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    info "All web services stopped."
  }
  trap cleanup_web_all EXIT INT TERM

  if [[ "$dev_mode" == true ]]; then
    info "Starting Next.js frontend in development mode on http://localhost:3000..."
    (
      cd "${ROOT_DIR}/web/frontend"
      npm run dev
    ) &
    pids+=("$!")
  fi

  info "Starting FastAPI backend on http://${host}:${port} (mode: ${mode})..."
  "$py_bin" "${ROOT_DIR}/web/backend/run_backend.py" --host "$host" --port "$port" --mode "$mode" &
  pids+=("$!")

  info "Web services are running. Press Ctrl-C to terminate."
  set +e
  wait "${pids[-1]}"
  set -e
}

# =============================================================================
# TARGET: COMMUNICATION BRIDGE (ROS 1 <-> ROS 2)
# =============================================================================
usage_bridge() {
  cat <<'EOF'
Usage: scripts/run.sh bridge [options]

Bridge communication between ROS 1 Melodic (Jetson Nano) and ROS 2 Jazzy (Host/WSL2).
Uses ros1_bridge dynamic_bridge if available, or falls back to Rosbridge WebSocket mode.

Options:
  --master-uri URI  Set ROS 1 master URI (default: http://localhost:11311)
  -h, --help        Show this help message

Examples:
  scripts/run.sh bridge
  scripts/run.sh bridge --master-uri http://192.168.1.100:11311
EOF
}

run_bridge() {
  local ros_master_uri="http://localhost:11311"

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --master-uri)
        [[ $# -ge 2 ]] || die '--master-uri requires a URI'
        ros_master_uri="$2"
        shift 2
        ;;
      -h|--help)
        usage_bridge
        return 0
        ;;
      *)
        die "unknown bridge option: $1"
        ;;
    esac
  done

  export ROS_MASTER_URI="$ros_master_uri"
  info "Khởi động cầu nối ROS 1 <-> ROS 2 Bridge (Master: ${ROS_MASTER_URI})..."

  if command -v ros2 >/dev/null 2>&1; then
    if ros2 pkg list 2>/dev/null | grep -q "ros1_bridge"; then
      info "Phát hiện package ros1_bridge. Đang chạy dynamic_bridge..."
      exec ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
    fi
  fi

  warn "Package ros1_bridge nhị phân không có sẵn."
  info "Sử dụng chế độ Rosbridge WebSocket bridge (cổng 9090) cho web và telemetry."
}

# -----------------------------------------------------------------------------
# MAIN DISPATCHER
# -----------------------------------------------------------------------------
if [[ $# -eq 0 ]]; then
  run_sim
  exit $?
fi

case "$1" in
  -h|--help)
    usage_master
    exit 0
    ;;
  sim)
    shift
    run_sim "$@"
    ;;
  robot)
    shift
    run_robot "$@"
    ;;
  web)
    shift
    run_web "$@"
    ;;
  bridge)
    shift
    run_bridge "$@"
    ;;
  --sim)
    shift
    run_sim "$@"
    ;;
  --robot)
    shift
    run_robot "$@"
    ;;
  --web)
    shift
    run_web "$@"
    ;;
  --bridge)
    shift
    run_bridge "$@"
    ;;
  *)
    if [[ "$1" == -* ]]; then
      run_sim "$@"
    else
      echo "Error: Unknown target '$1'. Run 'scripts/run.sh --help' for available targets." >&2
      exit 1
    fi
    ;;
esac

