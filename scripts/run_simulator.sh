#!/usr/bin/env bash
set -euo pipefail

# Launch the Marsy PyQt simulator together with the web dashboard in SIM mode.
# The dashboard provides Manual control, Explore area, AI Agent, generated plan,
# live mission telemetry, and the latest map viewer.

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

export MARSY_MODE="sim"
export MARSY_SIM_HOST="${MARSY_SIM_HOST:-127.0.0.1}"
export MARSY_SIM_PORT="${MARSY_SIM_PORT:-8523}"
export MARSY_WEB_HOST="${MARSY_WEB_HOST:-127.0.0.1}"
export MARSY_WEB_PORT="${MARSY_WEB_PORT:-8080}"
export MARSY_CAMERA_ROTATION="${MARSY_CAMERA_ROTATION:-0}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

if [[ -n "${MARSY_PYTHON:-}" ]]; then
  PYTHON_BIN="$MARSY_PYTHON"
elif [[ -n "${CONDA_PREFIX:-}" && -x "$CONDA_PREFIX/bin/python" ]]; then
  PYTHON_BIN="$CONDA_PREFIX/bin/python"
elif [[ -x "$PROJECT_ROOT/.venv/bin/python" ]]; then
  PYTHON_BIN="$PROJECT_ROOT/.venv/bin/python"
elif command -v python3 >/dev/null 2>&1; then
  PYTHON_BIN="$(command -v python3)"
elif command -v python >/dev/null 2>&1; then
  PYTHON_BIN="$(command -v python)"
else
  echo "Marsy simulator: Python was not found." >&2
  exit 1
fi

GROQ_ENV="${MARSY_GROQ_ENV:-$HOME/.config/marsy/groq.env}"
if [[ -z "${GROQ_API_KEY:-}" && -f "$GROQ_ENV" ]]; then
  # shellcheck disable=SC1090
  source "$GROQ_ENV"
fi

LOG_DIR="$PROJECT_ROOT/artifacts/logs"
mkdir -p "$LOG_DIR"
SIM_LOG="$LOG_DIR/simulator_latest.log"
DASHBOARD_LOG="$LOG_DIR/simulator_dashboard_latest.log"
: > "$SIM_LOG"
: > "$DASHBOARD_LOG"

SIM_PID=""
DASHBOARD_PID=""
CLEANUP_DONE=0

cleanup() {
  local exit_code=$?
  if [[ "$CLEANUP_DONE" -eq 1 ]]; then
    return
  fi
  CLEANUP_DONE=1
  trap - EXIT INT TERM HUP

  echo
  echo "Stopping Marsy simulator stack..."

  if [[ -n "$DASHBOARD_PID" ]] && kill -0 "$DASHBOARD_PID" 2>/dev/null; then
    kill -TERM "$DASHBOARD_PID" 2>/dev/null || true
    wait "$DASHBOARD_PID" 2>/dev/null || true
  fi

  if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
    kill -TERM "$SIM_PID" 2>/dev/null || true
    wait "$SIM_PID" 2>/dev/null || true
  fi

  echo "Marsy simulator stack stopped."
  exit "$exit_code"
}
trap cleanup EXIT INT TERM HUP

check_port_free() {
  local host="$1"
  local port="$2"
  "$PYTHON_BIN" - "$host" "$port" <<'PY'
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((host, port))
    except OSError as exc:
        print(f"Port {host}:{port} is already in use: {exc}", file=sys.stderr)
        raise SystemExit(1)
PY
}

wait_for_url() {
  local url="$1"
  local label="$2"
  local timeout_s="${3:-25}"
  "$PYTHON_BIN" - "$url" "$label" "$timeout_s" <<'PY'
import sys
import time
import urllib.error
import urllib.request

url, label, timeout_raw = sys.argv[1:4]
deadline = time.monotonic() + float(timeout_raw)
last_error = None
while time.monotonic() < deadline:
    try:
        with urllib.request.urlopen(url, timeout=0.6) as response:
            if 200 <= response.status < 500:
                print(f"{label} ready: {url}")
                raise SystemExit(0)
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        last_error = exc
    time.sleep(0.25)
print(f"Timed out waiting for {label}: {last_error}", file=sys.stderr)
raise SystemExit(1)
PY
}

check_port_free "$MARSY_SIM_HOST" "$MARSY_SIM_PORT"
check_port_free "$MARSY_WEB_HOST" "$MARSY_WEB_PORT"

if [[ ! -f "$PROJECT_ROOT/simulator/marsy_sim_ui.py" ]]; then
  echo "Missing simulator/marsy_sim_ui.py" >&2
  exit 1
fi

if [[ ! -f "$PROJECT_ROOT/marsy_web/server.py" ]]; then
  echo "Missing marsy_web/server.py" >&2
  exit 1
fi

if ! "$PYTHON_BIN" -c "import PyQt6" >/dev/null 2>&1; then
  echo "PyQt6 is not available in: $PYTHON_BIN" >&2
  echo "Activate the Marsy conda environment or set MARSY_PYTHON explicitly." >&2
  exit 1
fi

if [[ -z "${GROQ_API_KEY:-}" ]]; then
  echo "Warning: GROQ_API_KEY is not loaded. Cached plans can run, but new plans and replanning will fail." >&2
fi

echo "Starting Marsy simulator with: $PYTHON_BIN"
"$PYTHON_BIN" simulator/marsy_sim_ui.py >"$SIM_LOG" 2>&1 &
SIM_PID=$!

if ! wait_for_url "http://${MARSY_SIM_HOST}:${MARSY_SIM_PORT}/state" "Simulator" 30; then
  echo "Simulator log:" >&2
  tail -n 60 "$SIM_LOG" >&2 || true
  exit 1
fi

echo "Starting Marsy dashboard in SIM mode..."
"$PYTHON_BIN" -m marsy_web.server \
  --host "$MARSY_WEB_HOST" \
  --port "$MARSY_WEB_PORT" \
  --camera-rotation "$MARSY_CAMERA_ROTATION" \
  >"$DASHBOARD_LOG" 2>&1 &
DASHBOARD_PID=$!

if ! wait_for_url "http://${MARSY_WEB_HOST}:${MARSY_WEB_PORT}/api/state" "Dashboard" 30; then
  echo "Dashboard log:" >&2
  tail -n 80 "$DASHBOARD_LOG" >&2 || true
  exit 1
fi

DASHBOARD_URL="http://${MARSY_WEB_HOST}:${MARSY_WEB_PORT}/"
MAPS_URL="http://${MARSY_WEB_HOST}:${MARSY_WEB_PORT}/maps"

echo
echo "Marsy simulator is running."
echo "Dashboard: $DASHBOARD_URL"
echo "Maps:     $MAPS_URL"
echo "Missions: Explore area and AI Agent"
echo "Logs:"
echo "  $SIM_LOG"
echo "  $DASHBOARD_LOG"
echo "Close the simulator window or press Ctrl+C here to stop both processes."

if [[ "${MARSY_OPEN_BROWSER:-1}" != "0" ]]; then
  if command -v open >/dev/null 2>&1; then
    open "$DASHBOARD_URL" >/dev/null 2>&1 || true
  elif command -v xdg-open >/dev/null 2>&1; then
    xdg-open "$DASHBOARD_URL" >/dev/null 2>&1 || true
  fi
fi

# Portable replacement for `wait -n` (not available in macOS Bash 3.2).
while true; do
  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    wait "$SIM_PID" 2>/dev/null || true
    echo "Simulator window closed."
    break
  fi
  if ! kill -0 "$DASHBOARD_PID" 2>/dev/null; then
    wait "$DASHBOARD_PID" 2>/dev/null || true
    echo "Dashboard process stopped unexpectedly." >&2
    tail -n 80 "$DASHBOARD_LOG" >&2 || true
    break
  fi
  sleep 0.5
done
