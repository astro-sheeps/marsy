#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

export MARSY_MODE="${MARSY_MODE:-real}"
export MARSY_WEB_HOST="${MARSY_WEB_HOST:-0.0.0.0}"
export MARSY_CAMERA_ROTATION="${MARSY_CAMERA_ROTATION:-90}"
export MARSY_WEB_PORT="${MARSY_WEB_PORT:-8080}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

python -m marsy_web.server --host "$MARSY_WEB_HOST" --port "$MARSY_WEB_PORT"
