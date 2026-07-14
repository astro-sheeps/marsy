#!/usr/bin/env bash
# Stop the Marsy real web dashboard safely and aggressively if needed.
# Run on the Raspberry Pi from the marsy project folder:
#   ./scripts/kill_dashboard.sh

set -u

PORT="${MARSY_DASHBOARD_PORT:-8080}"
HOST="127.0.0.1"
URL="http://${HOST}:${PORT}"
PATTERN='marsy_web.server|python3 -m marsy_web.server|python -m marsy_web.server|run_real_dashboard.sh'

log() {
  printf '[%(%H:%M:%S)T] %s\n' -1 "$*"
}

matching_pids() {
  pgrep -f "$PATTERN" 2>/dev/null | grep -v "^$$$" || true
}

log "Stopping Marsy dashboard on port ${PORT}..."

# 1) Ask the dashboard to stop itself. This gives it a chance to stop motors,
# release the camera, and run rover cleanup normally.
if command -v curl >/dev/null 2>&1; then
  log "Requesting dashboard shutdown API..."
  curl -fsS -m 2 -X POST "${URL}/api/shutdown" >/dev/null 2>&1 \
    || curl -fsS -m 2 "${URL}/api/shutdown" >/dev/null 2>&1 \
    || true
  sleep 2
fi

# 2) Gracefully terminate remaining dashboard processes.
PIDS="$(matching_pids)"
if [ -n "$PIDS" ]; then
  log "Sending TERM to dashboard processes: $(echo "$PIDS" | tr '\n' ' ')"
  kill -TERM $PIDS 2>/dev/null || true
  sleep 3
fi

# 3) Force-kill anything still alive.
PIDS="$(matching_pids)"
if [ -n "$PIDS" ]; then
  log "Sending KILL to stuck dashboard processes: $(echo "$PIDS" | tr '\n' ' ')"
  kill -KILL $PIDS 2>/dev/null || true
  sleep 1
fi

# 4) Free the HTTP port if some leftover process is still holding it.
if command -v fuser >/dev/null 2>&1; then
  if fuser "${PORT}/tcp" >/dev/null 2>&1; then
    log "Freeing TCP port ${PORT} with fuser..."
    fuser -k "${PORT}/tcp" >/dev/null 2>&1 || true
    sudo -n fuser -k "${PORT}/tcp" >/dev/null 2>&1 || true
    sleep 1
  fi
fi

# 5) Report final state.
PIDS="$(matching_pids)"
if [ -n "$PIDS" ]; then
  log "WARNING: dashboard process still exists:"
  ps -fp $PIDS || true
  log "If the camera is still stuck, run: sudo reboot"
  exit 1
fi

if command -v fuser >/dev/null 2>&1 && fuser "${PORT}/tcp" >/dev/null 2>&1; then
  log "WARNING: port ${PORT} is still in use. Try: sudo fuser -k ${PORT}/tcp"
  exit 1
fi

log "Dashboard stopped."
