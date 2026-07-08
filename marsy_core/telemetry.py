"""
Lightweight telemetry/log bridge for Marsy missions.

Project location:
    marsy_core/telemetry.py

Use this instead of plain print() in mission/behavior code when you want the
same message to appear both in the terminal and in the simulator window.

In simulator mode it POSTs log events to:
    http://127.0.0.1:8523/telemetry

On the real rover it still prints to the terminal, but by default it does not
try to contact the simulator UI. Override with:
    MARSY_TELEMETRY_TO_SIM=1

Disable simulator telemetry completely with:
    MARSY_TELEMETRY_TO_SIM=0
"""

from __future__ import annotations

import json
import os
import socket
import time
from typing import Any, Optional


TELEMETRY_BUILD_STAMP = "telemetry_bridge_control_v4_2026_07_08"

SIM_HOST = os.getenv("MARSY_SIM_HOST", "127.0.0.1")
SIM_PORT = int(os.getenv("MARSY_SIM_PORT", "8523"))

# Keep this long enough for the simulator Flask thread to accept messages while
# it is also serving /state requests from sim_backend. Still short enough not to
# make the real rover wait noticeably if the UI is gone.
SIM_TIMEOUT_S = float(os.getenv("MARSY_TELEMETRY_TIMEOUT_S", "0.25"))
SIM_RETRIES = int(os.getenv("MARSY_TELEMETRY_RETRIES", "1"))


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on", "y"}


def _should_send_to_simulator() -> bool:
    """Return True when telemetry should be mirrored into marsy_sim_ui.py."""
    if os.getenv("MARSY_TELEMETRY_TO_SIM") is not None:
        return _env_bool("MARSY_TELEMETRY_TO_SIM", default=True)

    # Default: mirror only in simulator mode, so real-rover runs do not waste
    # time trying to connect to a UI that probably is not running.
    return os.getenv("MARSY_MODE", "sim").strip().lower() == "sim"


def send_telemetry(
    message: str,
    *,
    source: str = "mission",
    level: str = "info",
    event: Optional[str] = None,
    **fields: Any,
) -> bool:
    """
    Send one telemetry/log event to the simulator UI.

    This function intentionally uses raw sockets instead of requests so the
    rover project does not need an extra dependency. Failures are silent by
    default: logs must never stop the robot.
    """
    if not _should_send_to_simulator():
        return False

    payload = {
        "time": round(time.time(), 3),
        "source": source,
        "level": level,
        "message": str(message),
    }

    if event is not None:
        payload["event"] = event

    payload.update(fields)

    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    request = (
        b"POST /telemetry HTTP/1.1\r\n"
        + f"Host: {SIM_HOST}:{SIM_PORT}\r\n".encode("utf-8")
        + b"Content-Type: application/json\r\n"
        + f"Content-Length: {len(body)}\r\n".encode("utf-8")
        + b"Connection: close\r\n"
        + b"\r\n"
        + body
    )

    attempts = max(1, SIM_RETRIES + 1)
    last_error = None

    for _ in range(attempts):
        try:
            with socket.create_connection((SIM_HOST, SIM_PORT), timeout=SIM_TIMEOUT_S) as sock:
                sock.settimeout(SIM_TIMEOUT_S)
                sock.sendall(request)
            return True
        except OSError as exc:
            last_error = exc
            time.sleep(0.02)

    if _env_bool("MARSY_TELEMETRY_DEBUG", default=False):
        print(f"[telemetry] send failed: {last_error}", flush=True)

    return False


def get_simulator_state() -> dict[str, Any]:
    """
    Read simulator /state.

    Used by autonomous missions in simulator mode so UI buttons can request a
    graceful stop. On real rover runs this returns an empty dict by default.
    """
    if not _should_send_to_simulator():
        return {}

    request = (
        b"GET /state HTTP/1.1\r\n"
        + f"Host: {SIM_HOST}:{SIM_PORT}\r\n".encode("utf-8")
        + b"Connection: close\r\n"
        + b"\r\n"
    )

    try:
        with socket.create_connection((SIM_HOST, SIM_PORT), timeout=SIM_TIMEOUT_S) as sock:
            sock.settimeout(SIM_TIMEOUT_S)
            sock.sendall(request)
            chunks = []
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                chunks.append(chunk)

        raw = b"".join(chunks)
        _, _, body = raw.partition(b"\r\n\r\n")
        if not body:
            return {}
        return json.loads(body.decode("utf-8", errors="replace"))
    except (OSError, json.JSONDecodeError, UnicodeDecodeError) as exc:
        if _env_bool("MARSY_TELEMETRY_DEBUG", default=False):
            print(f"[telemetry] /state read failed: {exc}", flush=True)
        return {}


def simulator_stop_requested() -> bool:
    """
    Return True when the simulator UI top STOP button requested mission stop.

    This is intentionally simulator-scoped. On the real rover it returns False
    unless MARSY_TELEMETRY_TO_SIM is explicitly enabled and a simulator is
    reachable.
    """
    state = get_simulator_state()
    control = state.get("control") or {}
    return bool(
        state.get("mission_stop_requested", False)
        or control.get("mission_stop_requested", False)
    )


def log(
    message: str,
    *,
    source: str = "mission",
    level: str = "info",
    event: Optional[str] = None,
    **fields: Any,
) -> None:
    """Print a message and mirror it to the simulator telemetry panel."""
    print(message, flush=True)
    send_telemetry(
        message,
        source=source,
        level=level,
        event=event,
        **fields,
    )
