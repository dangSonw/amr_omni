#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
ROOT_DIR="$(get_repo_root)"
HOST="0.0.0.0"
PORT="8000"
MODE="auto"
DEV_MODE=false
BUILD_FRONTEND=false

usage() {
  cat <<'EOF'
Usage: scripts/run_web.sh [options]

Start the AMR Omni web interface (FastAPI backend + Next.js frontend).

Options:
  --port PORT        Port for backend web server (default: 8000)
  --host HOST        Host address to bind to (default: 0.0.0.0)
  --mode MODE        Bridge mode: auto, ros2, ros1 (default: auto)
  --dev              Run Next.js dev server on port 3000 alongside backend
  --build            Rebuild frontend static bundle before starting
  -h, --help         Show this help message

Examples:
  scripts/run_web.sh                     # Start production backend + static UI
  scripts/run_web.sh --mode ros2         # Start with native ROS 2 Jazzy bridge
  scripts/run_web.sh --dev               # Start with Next.js hot-reload on :3000
  scripts/run_web.sh --build             # Rebuild frontend and serve
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      [[ $# -ge 2 ]] || die '--port requires a port number'
      PORT="$2"
      shift 2
      ;;
    --host)
      [[ $# -ge 2 ]] || die '--host requires a host address'
      HOST="$2"
      shift 2
      ;;
    --mode)
      [[ $# -ge 2 ]] || die '--mode requires a mode (auto, ros2, ros1)'
      MODE="$2"
      shift 2
      ;;
    --dev)
      DEV_MODE=true
      shift
      ;;
    --build)
      BUILD_FRONTEND=true
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

# Check Python virtual environment
PYTHON_BIN="${ROOT_DIR}/web/backend/.venv/bin/python3"
if [[ ! -f "$PYTHON_BIN" ]]; then
  info "Virtual environment not found, creating at web/backend/.venv..."
  python3 -m venv "${ROOT_DIR}/web/backend/.venv"
  "${ROOT_DIR}/web/backend/.venv/bin/pip" install -r "${ROOT_DIR}/web/backend/requirements.txt"
fi

# Rebuild frontend if requested or if static bundle is missing
STATIC_INDEX="${ROOT_DIR}/web/backend/static/index.html"
if [[ "$BUILD_FRONTEND" == true || (! -f "$STATIC_INDEX" && "$DEV_MODE" != true) ]]; then
  info "Building Next.js frontend static bundle..."
  (
    cd "${ROOT_DIR}/web/frontend"
    npm run build
  )
  mkdir -p "${ROOT_DIR}/web/backend/static"
  cp -r "${ROOT_DIR}/web/frontend/out/"* "${ROOT_DIR}/web/backend/static/"
  info "Frontend static bundle copied to web/backend/static/"
fi

# Cleanup handler
PIDS=()
cleanup() {
  info "Stopping web servers..."
  local pid
  for pid in "${PIDS[@]:-}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  info "All web services stopped."
}
trap cleanup EXIT INT TERM

if [[ "$DEV_MODE" == true ]]; then
  info "Starting Next.js frontend in development mode on http://localhost:3000..."
  (
    cd "${ROOT_DIR}/web/frontend"
    npm run dev
  ) &
  PIDS+=("$!")
fi

info "Starting FastAPI backend on http://${HOST}:${PORT} (mode: ${MODE})..."
"$PYTHON_BIN" "${ROOT_DIR}/web/backend/run_backend.py" --host "$HOST" --port "$PORT" --mode "$MODE" &
PIDS+=("$!")

info "Web services are running. Press Ctrl-C to terminate."
set +e
wait "${PIDS[-1]}"
set -e

