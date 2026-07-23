#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

export MARSY_MODE="${MARSY_MODE:-real}"
export MARSY_WEB_HOST="${MARSY_WEB_HOST:-0.0.0.0}"
export MARSY_CAMERA_ROTATION="${MARSY_CAMERA_ROTATION:-90}"
export MARSY_WEB_PORT="${MARSY_WEB_PORT:-8080}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

GROQ_ENV="${MARSY_GROQ_ENV:-$HOME/.config/marsy/groq.env}"
if [[ -f "$GROQ_ENV" ]]; then
  # shellcheck disable=SC1090
  source "$GROQ_ENV"
fi

if [[ -n "${MARSY_PYTHON:-}" ]]; then
  PYTHON_BIN="$MARSY_PYTHON"
elif [[ -x ".venv/bin/python" ]]; then
  PYTHON_BIN=".venv/bin/python"
else
  PYTHON_BIN="python"
fi

exec "$PYTHON_BIN" -m marsy_web.server --host "$MARSY_WEB_HOST" --port "$MARSY_WEB_PORT"
