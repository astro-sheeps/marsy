"""
Marsy real-rover web dashboard.

Run from the repository root:

    MARSY_MODE=real python -m marsy_web.server --host 0.0.0.0 --port 8080

The server intentionally uses only the Python standard library for HTTP so the
first version is easy to run on a Raspberry Pi. Camera support uses Picamera2
when it is installed, otherwise the UI still loads with a placeholder stream.
"""

from __future__ import annotations

import argparse
import atexit
import json
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import quote, unquote, urlparse

from behaviors.explore_navigation import (
    ExploreMotionState,
    choose_turn_direction,
    movement_made_progress,
)
from marsy_web.camera import CameraStream

STATIC_DIR = Path(__file__).resolve().parent / "static"
PROJECT_ROOT = Path(__file__).resolve().parents[1]
MAPS_DIR = Path(
    os.getenv("MARSY_MAP_DIR", str(PROJECT_ROOT / "artifacts" / "maps"))
).expanduser().resolve()
MAP_LIST_LIMIT = max(1, int(os.getenv("MARSY_MAP_LIST_LIMIT", "250")))
MAP_MAX_BYTES = max(1024, int(os.getenv("MARSY_MAP_MAX_BYTES", str(10 * 1024 * 1024))))
LATEST_MAP_STEM = "explore_area_latest"
MAP_OUTPUT_SUFFIXES = {".json", ".svg", ".tmp"}
DEFAULT_MODE = os.getenv("MARSY_MODE", "real").strip().lower()
HEARTBEAT_TIMEOUT_S = float(os.getenv("MARSY_WEB_HEARTBEAT_TIMEOUT", "1.5"))
DISTANCE_POLL_INTERVAL_S = float(os.getenv("MARSY_WEB_DISTANCE_POLL_INTERVAL", "0.75"))
LIDAR_SETTLE_S = float(os.getenv("MARSY_WEB_LIDAR_SETTLE", "0.35"))
MANUAL_GUARD_INTERVAL_S = float(os.getenv("MARSY_WEB_MANUAL_GUARD_INTERVAL", "0.25"))
DEFAULT_SAFE_DISTANCE_CM = float(os.getenv("MARSY_WEB_SAFE_DISTANCE_CM", "55"))
DEFAULT_DANGER_DISTANCE_CM = float(os.getenv("MARSY_WEB_DANGER_DISTANCE_CM", "35"))
DEFAULT_MAX_VALID_DISTANCE_CM = float(os.getenv("MARSY_WEB_MAX_VALID_DISTANCE_CM", "250"))
MAST_MAX_ANGLE_DEG = int(float(os.getenv("MARSY_WEB_MAST_MAX_ANGLE", "81")))
NO_ECHO_MEANS_CLEAR = os.getenv("MARSY_WEB_NO_ECHO_BLOCKS", "0").strip().lower() not in {"1", "true", "yes", "on", "y"}
FORWARD_COMMANDS = {"forward", "forward_left", "forward_right"}
STEERING_COMMANDS = {"steer_left", "steer_right"}
MOVING_COMMANDS = FORWARD_COMMANDS | {"reverse", "reverse_left", "reverse_right", "spin_left", "spin_right"}

MISSION_CATALOG: list[dict[str, Any]] = [
    {
        "id": "avoid_obstacle",
        "name": "Obstacle avoidance",
        "description": "Drive continuously, scan with the mast sensor, and steer around nearby obstacles.",
        "map_enabled": False,
        "parameters": [
            {"name": "run_seconds", "label": "Run seconds", "type": "number", "min": 1, "step": 1, "optional": True},
            {"name": "forward_speed", "label": "Forward speed", "type": "number", "min": 10, "max": 70, "step": 1, "default": 25},
            {"name": "safe_distance", "label": "Safe distance, cm", "type": "number", "min": 20, "max": 120, "step": 1, "default": 55},
            {"name": "danger_distance", "label": "Danger distance, cm", "type": "number", "min": 10, "max": 90, "step": 1, "default": 35},
        ],
    },
    {
        "id": "explore_area",
        "name": "Explore area",
        "description": "Explore unknown space, avoid obstacles, and continuously save a SLAM-lite occupancy map.",
        "map_enabled": True,
        "parameters": [
            {"name": "steps", "label": "Maximum steps", "type": "number", "min": 1, "max": 500, "step": 1, "default": 20},
            {"name": "run_seconds", "label": "Run seconds", "type": "number", "min": 1, "step": 1, "optional": True},
            {"name": "step_distance", "label": "Step distance, cm", "type": "number", "min": 2, "max": 50, "step": 1, "default": 12},
            {"name": "speed", "label": "Forward speed", "type": "number", "min": 10, "max": 60, "step": 1, "default": 25},
            {"name": "safe_distance", "label": "Safe distance, cm", "type": "number", "min": 20, "max": 120, "step": 1, "default": 45},
            {"name": "danger_distance", "label": "Danger distance, cm", "type": "number", "min": 10, "max": 90, "step": 1, "default": 28},
            {"name": "resolution", "label": "Map cell, cm", "type": "number", "min": 2, "max": 20, "step": 1, "default": 5},
            {"name": "return_home", "label": "Return home after exploration", "type": "checkbox", "default": False},
        ],
    },
]



def _clear_map_artifacts() -> int:
    """Delete dashboard map exports and temporary files.

    Maps are session data: the dashboard keeps only the current JSON/SVG pair
    while it is running and removes it during normal shutdown. Clearing at
    startup also removes leftovers after an unclean power loss.
    """
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    removed = 0
    for path in MAPS_DIR.iterdir():
        if not path.is_file():
            continue
        if path.suffix.lower() not in MAP_OUTPUT_SUFFIXES and not path.name.endswith(".tmp"):
            continue
        try:
            path.unlink(missing_ok=True)
            removed += 1
        except OSError:
            continue
    return removed

def _safe_map_path(filename: str, allowed_suffixes: set[str]) -> Optional[Path]:
    """Resolve a map filename without allowing directory traversal."""
    decoded = unquote(filename).strip()
    if not decoded or Path(decoded).name != decoded:
        return None
    candidate = (MAPS_DIR / decoded).resolve()
    if candidate.parent != MAPS_DIR or candidate.suffix.lower() not in allowed_suffixes:
        return None
    return candidate


def _map_run_id(stem: str) -> str:
    if stem.endswith("_final"):
        return stem[:-6]
    marker = stem.rfind("_step_")
    if marker >= 0:
        return stem[:marker]
    return stem


def _map_summary(path: Path) -> Dict[str, Any]:
    stat = path.stat()
    summary: Dict[str, Any] = {
        "name": path.name,
        "run_id": _map_run_id(path.stem),
        "modified_at": stat.st_mtime,
        "size_bytes": stat.st_size,
        "is_final": path.stem.endswith("_final"),
        "json_url": f"/maps/file/{quote(path.name)}",
        "svg_url": None,
        "format": None,
        "mapping_mode": None,
        "resolution_cm": None,
        "path_points": 0,
        "cells": 0,
        "free_cells": 0,
        "occupied_cells": 0,
        "visited_cells": 0,
        "metadata": {},
        "error": None,
    }

    svg_path = path.with_suffix(".svg")
    if svg_path.exists() and svg_path.is_file():
        summary["svg_url"] = f"/maps/file/{quote(svg_path.name)}"

    if stat.st_size > MAP_MAX_BYTES:
        summary["error"] = f"map file is larger than {MAP_MAX_BYTES} bytes"
        return summary

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("map root must be an object")
        cells = data.get("cells") if isinstance(data.get("cells"), list) else []
        path_points = data.get("path") if isinstance(data.get("path"), list) else []
        states = [cell.get("state") for cell in cells if isinstance(cell, dict)]
        metadata = data.get("metadata") if isinstance(data.get("metadata"), dict) else {}
        summary.update(
            {
                "format": data.get("format"),
                "mapping_mode": data.get("mapping_mode"),
                "resolution_cm": data.get("resolution_cm"),
                "path_points": len(path_points),
                "cells": len(cells),
                "free_cells": sum(1 for state in states if state == "free"),
                "occupied_cells": sum(1 for state in states if state == "occupied"),
                "visited_cells": sum(
                    1
                    for cell in cells
                    if isinstance(cell, dict) and int(cell.get("visits", 0) or 0) > 0
                ),
                "metadata": metadata,
                "is_final": bool(metadata.get("final", summary["is_final"])),
            }
        )
    except Exception as exc:
        summary["error"] = f"{type(exc).__name__}: {exc}"
    return summary


def _list_maps() -> Dict[str, Any]:
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    paths = sorted(
        (path for path in MAPS_DIR.glob("*.json") if path.is_file()),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )[:MAP_LIST_LIMIT]
    return {
        "maps": [_map_summary(path) for path in paths],
        "count": len(paths),
        "limit": MAP_LIST_LIMIT,
    }


def _read_map(filename: str) -> tuple[Optional[Dict[str, Any]], Optional[str]]:
    path = _safe_map_path(filename, {".json"})
    if path is None:
        return None, "invalid map filename"
    if not path.exists() or not path.is_file():
        return None, "map not found"
    if path.stat().st_size > MAP_MAX_BYTES:
        return None, f"map file is larger than {MAP_MAX_BYTES} bytes"
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return None, f"cannot read map: {type(exc).__name__}: {exc}"
    if not isinstance(data, dict):
        return None, "map root must be an object"
    return data, None


class MissionStopRequested(Exception):
    """Internal stop signal for the dashboard-owned avoidance thread."""



@dataclass
class MissionState:
    running: bool = False
    name: Optional[str] = None
    started_at: Optional[float] = None
    returncode: Optional[int] = None
    last_error: Optional[str] = None
    command: list[str] = field(default_factory=list)
    progress: Dict[str, Any] = field(default_factory=dict)


@dataclass
class DashboardState:
    mode: str = "manual"
    backend: str = DEFAULT_MODE
    speed: int = 30
    steer_angle: int = 24
    mast_angle: int = 45
    lidar_angle_deg: int = 0
    safe_distance_cm: float = DEFAULT_SAFE_DISTANCE_CM
    danger_distance_cm: float = DEFAULT_DANGER_DISTANCE_CM
    lidar_scan: Optional[Dict[str, Any]] = None
    last_command: str = "idle"
    last_error: Optional[str] = None
    last_heartbeat_age_s: Optional[float] = None
    emergency_stop: bool = False
    shutdown_requested: bool = False
    distance_cm: Optional[float] = None
    battery_v: Optional[float] = None
    camera: Dict[str, Any] = field(default_factory=dict)
    mission: MissionState = field(default_factory=MissionState)
    logs: list[str] = field(default_factory=list)


def _clamp_int(value: Any, low: int, high: int, default: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        parsed = default
    return max(low, min(high, parsed))


def _fmt_distance(value: Optional[float]) -> str:
    if value is None:
        return "None"
    try:
        return f"{float(value):.1f} cm"
    except (TypeError, ValueError):
        return str(value)


def _clamp_float(value: Any, low: float, high: float, default: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        parsed = default
    return max(low, min(high, parsed))


def _optional_positive_float(value: Any) -> Optional[float]:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if parsed > 0 else None


class RoverController:
    """Coordinates rover access, heartbeat safety, telemetry, and missions."""

    def __init__(self, camera: CameraStream):
        self.camera = camera
        self.state = DashboardState(camera=asdict(camera.status))
        self._lock = threading.RLock()
        # Serialise all physical rover I/O. The 4tronix ultrasonic
        # getDistance() function flips the same GPIO pin between OUT and IN,
        # so concurrent dashboard state/lidar requests can otherwise race and
        # produce "GPIO channel has not been set up as an OUTPUT".
        self._hardware_lock = threading.RLock()
        self._cleanup_lock = threading.Lock()
        self._cleanup_done = False
        self._rover = None
        self._motion = None
        self._sensors = None
        self._initialized = False
        self._last_heartbeat = time.time()
        self._last_distance_poll = 0.0
        self._last_manual_guard = 0.0
        self._manual_active_command: Optional[str] = None
        self._mission_process: Optional[subprocess.Popen[str]] = None  # legacy; no longer used for new missions
        self._mission_thread: Optional[threading.Thread] = None
        self._mission_stop_event = threading.Event()
        self._stop_threads = threading.Event()

        threading.Thread(target=self._watchdog_loop, daemon=True).start()
        threading.Thread(target=self._mission_monitor_loop, daemon=True).start()

    def log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {message}"
        print(line, flush=True)
        with self._lock:
            self.state.logs.append(line)
            self.state.logs = self.state.logs[-120:]

    def _ensure_rover(self) -> None:
        if self._rover is not None:
            return
        from marsy_backends.loader import load_rover
        from marsy_core.motion import MarsyMotion
        from marsy_core.sensors import MarsySensors

        rover = load_rover()
        self._rover = rover
        self._motion = MarsyMotion(rover)
        self._sensors = MarsySensors(rover)
        self.log(f"Loaded Marsy backend: {DEFAULT_MODE}")

    def _ensure_initialized(self) -> None:
        self._ensure_rover()
        if self._initialized:
            return
        assert self._rover is not None
        self._rover.init(0)
        self._initialized = True
        self.log("Rover initialized")

    def cleanup(self) -> None:
        """Idempotent cleanup used by Ctrl+C, SIGTERM, and UI shutdown."""
        with self._cleanup_lock:
            if self._cleanup_done:
                return
            self._cleanup_done = True

        self._stop_threads.set()

        # Stop MJPEG capture first. Otherwise an already-open /video.mjpg
        # handler can call capture_jpeg() again and re-open Picamera2 after
        # the server has started cleaning up.
        try:
            self.camera.stop()
        except Exception as exc:
            self.log(f"camera cleanup failed: {exc}")

        self.stop_mission(reason="server shutdown")

        try:
            with self._hardware_lock:
                if self._motion is not None:
                    self._motion.brake_and_reset()
                elif self._rover is not None:
                    self._rover.brake()
        except Exception as exc:
            self.log(f"cleanup brake failed: {exc}")

        try:
            with self._hardware_lock:
                if self._rover is not None:
                    self._rover.cleanup()
        except Exception as exc:
            self.log(f"rover cleanup failed: {exc}")

        removed_maps = _clear_map_artifacts()
        if removed_maps:
            self.log(f"Removed {removed_maps} dashboard map files")
        self.log("Dashboard cleanup complete")

    def request_shutdown_state(self, reason: str) -> Dict[str, Any]:
        self.log(f"Shutdown requested: {reason}")
        with self._lock:
            self.state.shutdown_requested = True
            self.state.last_command = f"shutdown requested: {reason}"
        return self.snapshot_state()

    def heartbeat(self) -> Dict[str, Any]:
        with self._lock:
            self._last_heartbeat = time.time()
            self.state.emergency_stop = False
        return self.get_state()

    def _watchdog_loop(self) -> None:
        while not self._stop_threads.is_set():
            time.sleep(0.25)
            with self._lock:
                age = time.time() - self._last_heartbeat
                self.state.last_heartbeat_age_s = round(age, 2)
                should_stop = (
                    self.state.mode == "manual"
                    and age > HEARTBEAT_TIMEOUT_S
                    and not self.state.emergency_stop
                    and not self.state.shutdown_requested
                )
            if should_stop:
                try:
                    with self._hardware_lock:
                        self._ensure_initialized()
                        assert self._motion is not None
                        self._motion.brake_and_reset()
                    with self._lock:
                        self._manual_active_command = None
                        self.state.last_command = "heartbeat timeout → brake"
                        self.state.emergency_stop = True
                except Exception as exc:
                    with self._lock:
                        self.state.last_error = f"watchdog failed: {exc}"
            self._manual_safety_guard_once()

    def _mission_monitor_loop(self) -> None:
        while not self._stop_threads.is_set():
            time.sleep(0.4)
            proc = self._mission_process
            if proc is None:
                continue
            returncode = proc.poll()
            if returncode is not None:
                with self._lock:
                    self.state.mission.running = False
                    self.state.mission.returncode = returncode
                    self.state.mode = "manual"
                    self.state.last_command = f"mission finished: {returncode}"
                self._mission_process = None
                self.log(f"Mission process exited with code {returncode}")

    # ------------------------------------------------------------------
    # Shared distance / obstacle helpers
    # ------------------------------------------------------------------
    def _is_valid_distance(self, distance: Optional[float]) -> bool:
        if distance is None:
            return False
        try:
            value = float(distance)
        except (TypeError, ValueError):
            return False
        return 0.0 < value <= DEFAULT_MAX_VALID_DISTANCE_CM

    def _distance_score(self, distance: Optional[float]) -> float:
        if self._is_valid_distance(distance):
            return float(distance)  # type: ignore[arg-type]
        if NO_ECHO_MEANS_CLEAR:
            return DEFAULT_MAX_VALID_DISTANCE_CM
        return 0.0

    def _is_close_distance(self, distance: Optional[float], safe_distance: Optional[float] = None) -> bool:
        if safe_distance is None:
            safe_distance = self.state.safe_distance_cm
        return self._is_valid_distance(distance) and float(distance) < float(safe_distance)

    def _is_dangerous_distance(self, distance: Optional[float], danger_distance: Optional[float] = None) -> bool:
        if danger_distance is None:
            danger_distance = self.state.danger_distance_cm
        return self._is_valid_distance(distance) and float(distance) < float(danger_distance)

    def _read_front_distance_locked(self, samples: int = 1, sample_delay_s: float = 0.04) -> Optional[float]:
        """Read front distance while _hardware_lock is already held."""
        assert self._sensors is not None
        values: list[float] = []
        for _ in range(max(1, samples)):
            value = self._sensors.distance_cm()
            if self._is_valid_distance(value):
                values.append(float(value))
            if samples > 1:
                time.sleep(sample_delay_s)
        if not values:
            return None
        values.sort()
        return round(values[len(values) // 2], 1)

    def _record_distance(self, distance: Optional[float]) -> None:
        with self._lock:
            self.state.distance_cm = None if distance is None else round(float(distance), 1)
            self._last_distance_poll = time.time()

    def _manual_safety_guard_once(self) -> None:
        """Brake manual forward motion if the front range sensor sees an obstacle."""
        now = time.time()
        with self._lock:
            active = self._manual_active_command
            mode = self.state.mode
            shutdown = self.state.shutdown_requested
            safe_distance = self.state.safe_distance_cm
            if now - self._last_manual_guard < MANUAL_GUARD_INTERVAL_S:
                return
            self._last_manual_guard = now

        if shutdown or mode != "manual" or active not in FORWARD_COMMANDS:
            return
        if not self._hardware_lock.acquire(blocking=False):
            return
        try:
            self._ensure_initialized()
            assert self._motion is not None
            assert self._sensors is not None
            # Always guard using the forward-facing mast position.
            self._motion.mast_center()
            distance = self._read_front_distance_locked(samples=1)
            self._record_distance(distance)
            if self._is_close_distance(distance, safe_distance=safe_distance):
                self._motion.brake_and_reset()
                with self._lock:
                    self._manual_active_command = None
                    self.state.last_command = "manual guard: obstacle → brake"
                    self.state.last_error = f"manual guard: obstacle at {_fmt_distance(distance)}"
                    self.state.emergency_stop = True
                self.log(f"Manual guard stopped forward drive: obstacle at {_fmt_distance(distance)}")
        except Exception as exc:
            with self._lock:
                self.state.last_error = f"manual guard failed: {type(exc).__name__}: {exc}"
        finally:
            self._hardware_lock.release()

    # ------------------------------------------------------------------
    # Dashboard-owned obstacle avoidance. This intentionally runs inside this
    # process and reuses the same rover object. The earlier subprocess mission
    # fought the dashboard for GPIO/PWM ownership and could leave the 4tronix
    # library in a state where re-init raised "PWM object already exists".
    # ------------------------------------------------------------------
    def _mission_check_stop_locked(self) -> None:
        if self._mission_stop_event.is_set() or self._stop_threads.is_set():
            assert self._motion is not None
            self._motion.stop()
            raise MissionStopRequested()

    def _mission_drive_for_locked(self, duration_s: float, *, check_front: bool = True, danger_distance: float) -> Optional[float]:
        end_time = time.time() + max(0.0, duration_s)
        while time.time() < end_time:
            self._mission_check_stop_locked()
            sleep_time = min(0.12, end_time - time.time())
            if sleep_time > 0:
                time.sleep(sleep_time)
            if not check_front:
                continue
            distance = self._read_front_distance_locked(samples=1)
            self._record_distance(distance)
            if self._is_dangerous_distance(distance, danger_distance=danger_distance):
                assert self._motion is not None
                self._motion.stop()
                self.log(f"Mission interrupted manoeuvre: obstacle at {_fmt_distance(distance)}")
                return distance
        return None

    def _mission_reverse_locked(self, reverse_speed: int, reverse_time_s: float) -> None:
        assert self._motion is not None
        self.log("Mission action: reverse briefly")
        self._motion.reverse(reverse_speed, straighten=True)
        self._mission_drive_for_locked(reverse_time_s, check_front=False, danger_distance=self.state.danger_distance_cm)
        self._motion.stop()

    def _mission_scan_locked(self, scan_angle: int, settle_s: float) -> Dict[str, Optional[float]]:
        assert self._motion is not None
        self.log("Mission scan: left / center / right")
        self._mission_check_stop_locked()
        self._motion.mast_left(scan_angle)
        time.sleep(settle_s)
        left = self._read_front_distance_locked(samples=3)
        self._mission_check_stop_locked()
        self._motion.mast_center()
        time.sleep(settle_s)
        center = self._read_front_distance_locked(samples=3)
        self._mission_check_stop_locked()
        self._motion.mast_right(scan_angle)
        time.sleep(settle_s)
        right = self._read_front_distance_locked(samples=3)
        self._motion.mast_center()
        scan = {"left": left, "center": center, "right": right}
        with self._lock:
            self.state.lidar_scan = {"kind": "triad", "angle_deg": 0, "distances": scan}
        self.log(f"Mission scan result: left={left}, center={center}, right={right}")
        return scan

    def _mission_choose_direction(self, scan: Dict[str, Optional[float]], default_turn: str = "right") -> str:
        left_score = self._distance_score(scan.get("left"))
        right_score = self._distance_score(scan.get("right"))
        if left_score > right_score:
            return "left"
        if right_score > left_score:
            return "right"
        return default_turn if default_turn in {"left", "right"} else "right"

    def _mission_step_locked(self, cfg: Dict[str, Any]) -> None:
        assert self._motion is not None
        self._mission_check_stop_locked()
        self._motion.mast_center()
        distance = self._read_front_distance_locked(samples=3)
        self._record_distance(distance)
        safe_distance = float(cfg["safe_distance"])
        danger_distance = float(cfg["danger_distance"])

        if self._is_close_distance(distance, safe_distance=safe_distance):
            self.log(f"Mission obstacle detected: {_fmt_distance(distance)}")
            self._motion.stop()
            time.sleep(0.15)
            if self._is_dangerous_distance(distance, danger_distance=danger_distance):
                self._mission_reverse_locked(int(cfg["reverse_speed"]), float(cfg["reverse_time"]))
            scan = self._mission_scan_locked(int(cfg["scan_angle"]), float(cfg["scan_settle"]))
            direction = self._mission_choose_direction(scan, str(cfg["default_turn"]))
            self.log(f"Mission chosen direction: {direction}")
            if self._is_dangerous_distance(scan.get("center"), danger_distance=danger_distance):
                self._mission_reverse_locked(int(cfg["reverse_speed"]), float(cfg["reverse_time"]))
            if direction == "left":
                self._motion.forward_left(int(cfg["turn_speed"]), int(cfg["steer_angle"]))
            else:
                self._motion.forward_right(int(cfg["turn_speed"]), int(cfg["steer_angle"]))
            self._mission_drive_for_locked(float(cfg["turn_time"]), check_front=True, danger_distance=danger_distance)
            self._motion.stop()
            self._motion.wheels_straight()
            return

        # No echo is treated as clear by default, matching the existing behavior.
        self._motion.forward(int(cfg["forward_speed"]), straighten=True)
        with self._lock:
            self.state.last_command = "mission forward"

    def _read_mission_output(self, proc: subprocess.Popen[str]) -> None:
        if proc.stdout is None:
            return
        for line in proc.stdout:
            self.log(line.rstrip())

    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            self.state.camera = asdict(self.camera.status)
            now = time.time()
            need_distance = (
                self._sensors is not None
                and not self.state.mission.running
                and not self.state.shutdown_requested
                and (now - self._last_distance_poll) > DISTANCE_POLL_INTERVAL_S
            )

        if need_distance:
            try:
                with self._hardware_lock:
                    assert self._sensors is not None
                    distance = self._sensors.distance_cm()
                    battery = self._sensors.battery_v()
                with self._lock:
                    self.state.distance_cm = None if distance is None else round(float(distance), 1)
                    self.state.battery_v = None if battery is None else round(float(battery), 2)
                    self._last_distance_poll = now
            except Exception as exc:
                with self._lock:
                    self.state.last_error = f"telemetry failed: {exc}"
                    self._last_distance_poll = now

        with self._lock:
            self.state.last_heartbeat_age_s = round(time.time() - self._last_heartbeat, 2)
            return asdict(self.state)

    def snapshot_state(self) -> Dict[str, Any]:
        """Return state without polling hardware. Safe during shutdown."""
        with self._lock:
            self.state.camera = asdict(self.camera.status)
            self.state.last_heartbeat_age_s = round(time.time() - self._last_heartbeat, 2)
            return asdict(self.state)

    def command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        command = str(payload.get("command", "")).strip().lower()
        speed = _clamp_int(payload.get("speed", self.state.speed), 0, 100, self.state.speed)
        steer_angle = _clamp_int(payload.get("steer_angle", self.state.steer_angle), 0, 60, self.state.steer_angle)
        mast_angle = _clamp_int(payload.get("mast_angle", self.state.mast_angle), 0, MAST_MAX_ANGLE_DEG, self.state.mast_angle)
        try:
            safe_distance = float(payload.get("safe_distance", self.state.safe_distance_cm))
        except (TypeError, ValueError):
            safe_distance = self.state.safe_distance_cm
        try:
            danger_distance = float(payload.get("danger_distance", self.state.danger_distance_cm))
        except (TypeError, ValueError):
            danger_distance = self.state.danger_distance_cm

        with self._lock:
            if self.state.mission.running and command not in {"stop", "brake"}:
                self.state.last_error = "mission is running; stop mission before manual driving"
                return self.get_state()
            self.state.speed = speed
            self.state.steer_angle = steer_angle
            self.state.mast_angle = mast_angle
            self.state.safe_distance_cm = safe_distance
            self.state.danger_distance_cm = danger_distance
            self.state.mode = "manual"
            self._last_heartbeat = time.time()

        try:
            with self._hardware_lock:
                self._ensure_initialized()
                assert self._motion is not None
                motion = self._motion

                if command in FORWARD_COMMANDS:
                    motion.mast_center()
                    distance = self._read_front_distance_locked(samples=2)
                    self._record_distance(distance)
                    if self._is_close_distance(distance, safe_distance=safe_distance):
                        motion.brake_and_reset()
                        with self._lock:
                            self._manual_active_command = None
                            self.state.last_command = "manual guard: blocked"
                            self.state.last_error = f"blocked: obstacle at {_fmt_distance(distance)}"
                            self.state.emergency_stop = True
                        return self.snapshot_state()

                if command == "forward":
                    motion.forward(speed)
                elif command == "reverse":
                    motion.reverse(speed)
                elif command == "forward_left":
                    motion.forward_left(speed, steer_angle)
                elif command == "forward_right":
                    motion.forward_right(speed, steer_angle)
                elif command == "reverse_left":
                    motion.reverse_left(speed, steer_angle)
                elif command == "reverse_right":
                    motion.reverse_right(speed, steer_angle)
                elif command == "spin_left":
                    motion.spin_left(speed)
                elif command == "spin_right":
                    motion.spin_right(speed)
                elif command == "steer_left":
                    motion.steer_left(steer_angle)
                elif command == "steer_right":
                    motion.steer_right(steer_angle)
                elif command == "center_wheels":
                    motion.wheels_straight()
                elif command == "mast_left":
                    motion.mast_left(mast_angle)
                    with self._lock:
                        self.state.lidar_angle_deg = -mast_angle
                elif command == "mast_right":
                    motion.mast_right(mast_angle)
                    with self._lock:
                        self.state.lidar_angle_deg = mast_angle
                elif command == "mast_center":
                    motion.mast_center()
                    with self._lock:
                        self.state.lidar_angle_deg = 0
                elif command == "reset_pose":
                    motion.reset_pose()
                    with self._lock:
                        self.state.lidar_angle_deg = 0
                elif command == "stop":
                    motion.stop_and_reset()
                    with self._lock:
                        self.state.lidar_angle_deg = 0
                elif command == "brake":
                    motion.brake_and_reset()
                    with self._lock:
                        self.state.lidar_angle_deg = 0
                else:
                    raise ValueError(f"unknown command: {command}")

            with self._lock:
                self.state.last_command = command
                self.state.last_error = None
                self._manual_active_command = command if command in MOVING_COMMANDS else None
                if command in {"stop", "brake", "center_wheels", "reset_pose", "mast_left", "mast_right", "mast_center", "steer_left", "steer_right"}:
                    self._manual_active_command = None
                if command in {"stop", "brake"}:
                    self.state.emergency_stop = command == "brake"
            return self.get_state()
        except Exception as exc:
            with self._lock:
                self.state.last_error = f"{type(exc).__name__}: {exc}"
            return self.get_state()

    def lidar(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Move/scan the mast-mounted range sensor.

        The UI says LiDAR because this is the future dashboard role, but this
        currently uses Marsy's mast range sensor API, which is ultrasonic on the
        4tronix rover.
        """
        action = str(payload.get("action", "")).strip().lower()
        mast_angle = _clamp_int(payload.get("mast_angle", self.state.mast_angle), 0, MAST_MAX_ANGLE_DEG, self.state.mast_angle)
        try:
            safe_distance = float(payload.get("safe_distance", self.state.safe_distance_cm))
        except (TypeError, ValueError):
            safe_distance = self.state.safe_distance_cm
        try:
            danger_distance = float(payload.get("danger_distance", self.state.danger_distance_cm))
        except (TypeError, ValueError):
            danger_distance = self.state.danger_distance_cm
        angle_deg = _clamp_int(payload.get("angle_deg", 0), -MAST_MAX_ANGLE_DEG, MAST_MAX_ANGLE_DEG, 0)

        with self._lock:
            if self.state.mission.running:
                self.state.last_error = "mission is running; stop mission before manual mast/LiDAR control"
                return self.snapshot_state()
            self.state.mast_angle = mast_angle
            self._last_heartbeat = time.time()

        try:
            with self._hardware_lock:
                self._ensure_initialized()
                assert self._motion is not None
                assert self._sensors is not None
                motion = self._motion
                sensors = self._sensors

                if action == "left":
                    motion.mast_left(mast_angle)
                    time.sleep(LIDAR_SETTLE_S)
                    distance = sensors.distance_cm()
                    scan = {"kind": "point", "angle_deg": -mast_angle, "distance_cm": distance}
                    logical_angle = -mast_angle
                elif action == "right":
                    motion.mast_right(mast_angle)
                    time.sleep(LIDAR_SETTLE_S)
                    distance = sensors.distance_cm()
                    scan = {"kind": "point", "angle_deg": mast_angle, "distance_cm": distance}
                    logical_angle = mast_angle
                elif action == "center":
                    motion.mast_center()
                    time.sleep(LIDAR_SETTLE_S)
                    distance = sensors.distance_cm()
                    scan = {"kind": "point", "angle_deg": 0, "distance_cm": distance}
                    logical_angle = 0
                elif action == "to":
                    motion.mast_to(angle_deg)
                    time.sleep(LIDAR_SETTLE_S)
                    distance = sensors.distance_cm()
                    scan = {"kind": "point", "angle_deg": angle_deg, "distance_cm": distance}
                    logical_angle = angle_deg
                elif action == "scan":
                    if hasattr(sensors, "scan_mast"):
                        distances = sensors.scan_mast(motion, left_angle=mast_angle, right_angle=mast_angle, settle_s=LIDAR_SETTLE_S)
                    else:
                        motion.mast_left(mast_angle)
                        time.sleep(LIDAR_SETTLE_S)
                        left = sensors.distance_cm()
                        motion.mast_center()
                        time.sleep(LIDAR_SETTLE_S)
                        center = sensors.distance_cm()
                        motion.mast_right(mast_angle)
                        time.sleep(LIDAR_SETTLE_S)
                        right = sensors.distance_cm()
                        motion.mast_center()
                        distances = {"left": left, "center": center, "right": right}
                    scan = {"kind": "triad", "angle_deg": 0, "distances": distances}
                    logical_angle = 0
                elif action == "sweep":
                    raw_angles = payload.get("angles")
                    if isinstance(raw_angles, list) and raw_angles:
                        angles = [_clamp_int(a, -MAST_MAX_ANGLE_DEG, MAST_MAX_ANGLE_DEG, 0) for a in raw_angles]
                    else:
                        angles = [-60, -30, 0, 30, 60]
                    samples = []
                    for angle in angles:
                        motion.mast_to(angle)
                        time.sleep(LIDAR_SETTLE_S)
                        distance = sensors.distance_cm()
                        samples.append({"angle_deg": angle, "distance_cm": distance})
                    motion.mast_center()
                    scan = {"kind": "sweep", "samples": samples}
                    logical_angle = 0
                else:
                    raise ValueError(f"unknown lidar action: {action}")

            with self._lock:
                self.state.lidar_angle_deg = logical_angle
                self.state.lidar_scan = scan
                self.state.last_command = f"lidar {action}"
                self.state.last_error = None
                if scan.get("kind") == "point":
                    self.state.distance_cm = None if scan.get("distance_cm") is None else round(float(scan["distance_cm"]), 1)
                self.log(f"LiDAR/range {action}: {json.dumps(scan, ensure_ascii=False, default=str)}")
            return self.get_state()
        except Exception as exc:
            with self._lock:
                self.state.last_error = f"lidar failed: {type(exc).__name__}: {exc}"
            return self.get_state()

    def hard_stop(self) -> Dict[str, Any]:
        self.stop_mission(reason="dashboard stop")
        return self.command({"command": "brake"})

    def list_missions(self) -> Dict[str, Any]:
        with self._lock:
            active = asdict(self.state.mission)
        return {"missions": MISSION_CATALOG, "active": active}

    def _begin_mission(self, mission: str, safe_distance: float, danger_distance: float) -> bool:
        with self._lock:
            if self.state.mission.running:
                self.state.last_error = "mission is already running"
                return False
            self._mission_stop_event.clear()
            self._manual_active_command = None
            self.state.mode = "mission"
            self.state.safe_distance_cm = safe_distance
            self.state.danger_distance_cm = danger_distance
            self.state.mission = MissionState(
                running=True,
                name=mission,
                started_at=time.time(),
                command=["dashboard-thread", mission],
                progress={},
            )
            self.state.last_command = f"start mission: {mission}"
            self.state.last_error = None
        return True

    def _finish_mission(self, mission: str, returncode: int) -> None:
        with self._lock:
            self.state.mode = "manual"
            self.state.mission.running = False
            self.state.mission.returncode = returncode
            self.state.last_command = f"mission finished: {mission} ({returncode})"

    def start_mission(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Start a dashboard-owned mission using the dashboard's rover instance.

        Missions stay in this process so the dashboard and a child process do
        not compete for GPIO, PWM, the mast servo, or the ultrasonic sensor.
        """
        mission = str(payload.get("mission", "avoid_obstacle")).strip().lower()
        known = {item["id"] for item in MISSION_CATALOG}
        if mission not in known:
            with self._lock:
                self.state.last_error = f"unknown mission: {mission}"
            return self.snapshot_state()
        if mission == "avoid_obstacle":
            return self._start_avoid_obstacle(payload)
        if mission == "explore_area":
            return self._start_explore_area(payload)
        with self._lock:
            self.state.last_error = f"mission is not dashboard-compatible: {mission}"
        return self.snapshot_state()

    def _start_avoid_obstacle(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        safe_distance = _clamp_float(payload.get("safe_distance"), 10.0, 200.0, DEFAULT_SAFE_DISTANCE_CM)
        danger_distance = _clamp_float(payload.get("danger_distance"), 5.0, safe_distance, DEFAULT_DANGER_DISTANCE_CM)
        run_seconds = _optional_positive_float(payload.get("run_seconds"))
        cfg = {
            "safe_distance": safe_distance,
            "danger_distance": danger_distance,
            "forward_speed": _clamp_int(payload.get("forward_speed", 25), 0, 100, 25),
            "turn_speed": _clamp_int(payload.get("turn_speed", payload.get("forward_speed", 25)), 0, 100, 25),
            "reverse_speed": _clamp_int(payload.get("reverse_speed", 20), 0, 100, 20),
            "scan_angle": _clamp_int(payload.get("scan_angle", 55), 0, MAST_MAX_ANGLE_DEG, 55),
            "steer_angle": _clamp_int(payload.get("steer_angle", 28), 0, 60, 28),
            "reverse_time": _clamp_float(payload.get("reverse_time", 0.55), 0.1, 5.0, 0.55),
            "turn_time": _clamp_float(payload.get("turn_time", 1.05), 0.1, 8.0, 1.05),
            "scan_settle": _clamp_float(payload.get("scan_settle", LIDAR_SETTLE_S), 0.05, 2.0, LIDAR_SETTLE_S),
            "loop_delay": 0.15,
            "default_turn": "right",
            "run_seconds": run_seconds,
        }
        if not self._begin_mission("avoid_obstacle", safe_distance, danger_distance):
            return self.snapshot_state()

        def _run() -> None:
            start_time = time.time()
            returncode = 0
            try:
                with self._hardware_lock:
                    self._ensure_initialized()
                    assert self._motion is not None
                    self._motion.stop()
                    self._motion.wheels_straight()
                    self._motion.mast_center()
                self.log(
                    "Dashboard obstacle avoidance started: "
                    f"safe={safe_distance} cm, danger={danger_distance} cm"
                )
                while not self._mission_stop_event.is_set() and not self._stop_threads.is_set():
                    if run_seconds is not None and time.time() - start_time >= run_seconds:
                        self.log("Dashboard obstacle avoidance run time finished")
                        break
                    with self._hardware_lock:
                        self._mission_step_locked(cfg)
                    with self._lock:
                        self.state.mission.progress = {
                            "elapsed_s": round(time.time() - start_time, 1),
                            "run_seconds": run_seconds,
                        }
                    time.sleep(float(cfg["loop_delay"]))
            except MissionStopRequested:
                self.log("Dashboard obstacle avoidance stopped")
            except Exception as exc:
                returncode = 1
                self.log(f"Dashboard obstacle avoidance failed: {type(exc).__name__}: {exc}")
                with self._lock:
                    self.state.last_error = f"mission failed: {type(exc).__name__}: {exc}"
            finally:
                try:
                    with self._hardware_lock:
                        if self._motion is not None:
                            self._motion.brake_and_reset()
                except Exception as exc:
                    self.log(f"mission shutdown brake warning: {exc}")
                self._finish_mission("avoid_obstacle", returncode)
                self.log(f"Dashboard obstacle avoidance finished with code {returncode}")

        thread = threading.Thread(target=_run, name="marsy-obstacle-avoidance", daemon=True)
        self._mission_thread = thread
        thread.start()
        return self.snapshot_state()

    def _start_explore_area(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        steps = _clamp_int(payload.get("steps", 20), 1, 500, 20)
        run_seconds = _optional_positive_float(payload.get("run_seconds"))
        step_distance = _clamp_float(payload.get("step_distance", 12.0), 2.0, 50.0, 12.0)
        speed = _clamp_int(payload.get("speed", 25), 10, 60, 25)
        rotate_speed = _clamp_int(payload.get("rotate_speed", speed), 10, 60, speed)
        safe_distance = _clamp_float(payload.get("safe_distance", 45.0), 20.0, 150.0, 45.0)
        danger_distance = _clamp_float(payload.get("danger_distance", 28.0), 10.0, safe_distance, 28.0)
        max_range = _clamp_float(payload.get("max_range", 200.0), 50.0, 400.0, 200.0)
        resolution = _clamp_float(payload.get("resolution", 5.0), 2.0, 20.0, 5.0)
        return_home = bool(payload.get("return_home", False))
        turn_step = _clamp_float(payload.get("turn_step", 30.0), 15.0, 45.0, 30.0)
        max_turns_without_progress = _clamp_int(
            payload.get("max_turns_without_progress", 4), 2, 8, 4
        )
        # Scan from the centre outwards, alternating sides. This makes the
        # camera HUD update symmetrically and ensures right-side samples are
        # published early even if a scan is interrupted.
        scan_angles = [0.0, -25.0, 25.0, -50.0, 50.0, -75.0, 75.0]
        if not self._begin_mission("explore_area", safe_distance, danger_distance):
            return self.snapshot_state()

        def _save(skills: Any, step: int, final: bool = False) -> tuple[Path, Path]:
            # The map viewer needs one coherent, continually updated map rather
            # than hundreds of step snapshots. Atomic exporters in
            # skills.mapping keep readers from seeing a half-written file.
            json_path = MAPS_DIR / f"{LATEST_MAP_STEM}.json"
            svg_path = MAPS_DIR / f"{LATEST_MAP_STEM}.svg"
            metadata = {
                "mission": "explore_area",
                "step": step,
                "final": final,
                "pose_source": skills.pose.source,
                "note": "SLAM-lite occupancy grid; no loop closure",
            }
            skills.map.save_json(json_path, current_pose=skills.pose, metadata=metadata)
            skills.map.save_svg(svg_path, current_pose=skills.pose)
            return json_path, svg_path

        def _publish_sweep(samples: list[Any], *, scanning: bool, scan_id: str) -> None:
            """Expose the same range samples used by mapping to the camera HUD.

            Every sample is marked as measured even when the ultrasonic sensor
            returns no echo. The browser can therefore distinguish "not
            measured yet" from "measured and clear beyond sensor range".
            """
            serialised: list[Dict[str, Any]] = []
            center_distance: Optional[float] = None
            for sample in samples:
                try:
                    angle = float(sample.mast_angle_deg)
                    raw_distance = sample.distance_cm
                    distance = None if raw_distance is None else round(float(raw_distance), 1)
                    no_echo = bool(getattr(sample, "no_echo", raw_distance is None))
                    quality = str(getattr(sample, "quality", "measured" if distance is not None else "unknown"))
                    spread_cm = getattr(sample, "spread_cm", None)
                except (AttributeError, TypeError, ValueError):
                    continue
                serialised.append(
                    {
                        "angle_deg": angle,
                        "distance_cm": distance,
                        "measured": True,
                        "no_echo": no_echo,
                        "quality": quality,
                        "spread_cm": spread_cm,
                    }
                )
                if abs(angle) < 0.5 and distance is not None:
                    center_distance = distance

            with self._lock:
                self.state.lidar_scan = {
                    "kind": "sweep",
                    "source": "explore_area",
                    "scan_id": scan_id,
                    "scanning": scanning,
                    "max_range_cm": max_range,
                    "samples": serialised,
                    "updated_at": time.time(),
                }
                if center_distance is not None:
                    self.state.distance_cm = center_distance

        def _publish_avoidance_scan(scan: Any) -> None:
            if not isinstance(scan, dict):
                return
            distances = {
                "left": scan.get("left"),
                "center": scan.get("center"),
                "right": scan.get("right"),
            }
            with self._lock:
                self.state.lidar_scan = {
                    "kind": "triad",
                    "source": "explore_area_avoidance",
                    "scanning": False,
                    "distances": distances,
                    "updated_at": time.time(),
                }
                center = distances.get("center")
                if center is not None:
                    try:
                        self.state.distance_cm = round(float(center), 1)
                    except (TypeError, ValueError):
                        pass

        def _scan_with_retry(skills: Any, *, scan_id: str) -> tuple[list[Any], Dict[str, Any]]:
            result = skills.scan_arc(
                angles=scan_angles,
                samples=3,
                on_sample=lambda _sample, collected: _publish_sweep(
                    collected, scanning=True, scan_id=scan_id
                ),
            )
            samples = list(result.data.get("samples", []))
            _publish_sweep(samples, scanning=False, scan_id=scan_id)
            corridor = skills.assess_forward_corridor(samples)
            for attempt, offset in enumerate((4.0, -4.0), start=1):
                if corridor.get("status") != "range_unknown" or self._mission_stop_event.is_set():
                    break
                retry_id = f"{scan_id}-retry-{attempt}"
                shifted = [max(-90.0, min(90.0, angle + offset)) for angle in scan_angles]
                self.log(
                    f"Unreliable sonar sectors; repeat full scan with {offset:+.0f}° mast offset"
                )
                result = skills.scan_arc(
                    angles=shifted,
                    samples=3,
                    on_sample=lambda _sample, collected, current_id=retry_id: _publish_sweep(
                        collected, scanning=True, scan_id=current_id
                    ),
                )
                samples = list(result.data.get("samples", []))
                _publish_sweep(samples, scanning=False, scan_id=retry_id)
                corridor = skills.assess_forward_corridor(samples)
            return samples, corridor

        def _run() -> None:
            returncode = 0
            completed_steps = 0
            skills = None
            started = time.time()
            try:
                from skills import MarsySkills, SkillsConfig, SparseOccupancyGrid

                MAPS_DIR.mkdir(parents=True, exist_ok=True)
                removed_maps = _clear_map_artifacts()
                if removed_maps:
                    self.log(f"Removed {removed_maps} stale map files before exploration")
                with self._hardware_lock:
                    self._ensure_initialized()
                    assert self._rover is not None
                    assert self._motion is not None
                    assert self._sensors is not None
                    self._motion.stop_and_reset()
                    config = SkillsConfig(
                        safe_distance_cm=safe_distance,
                        danger_distance_cm=danger_distance,
                        max_range_cm=max_range,
                        default_speed=speed,
                        default_turn_speed=rotate_speed,
                    )
                    occupancy_map = SparseOccupancyGrid(
                        resolution_cm=resolution,
                        max_range_cm=max_range,
                    )
                    skills = MarsySkills(
                        self._rover,
                        motion=self._motion,
                        sensors=self._sensors,
                        occupancy_map=occupancy_map,
                        config=config,
                        stop_requested=lambda: self._mission_stop_event.is_set() or self._stop_threads.is_set(),
                    )
                    skills.sync_pose()
                    skills.home_pose = skills.pose.normalized()
                    skills.set_led_status("exploring")

                self.log(
                    "Dashboard explore area started: "
                    f"steps={steps}, step={step_distance} cm, safe={safe_distance} cm"
                )
                navigation = ExploreMotionState()
                max_actions = max(steps * 6, steps + 8)

                def _side_summary(evidence: Any) -> Dict[str, Any]:
                    return {
                        "trusted": getattr(evidence, "trusted_count", 0),
                        "clear": getattr(evidence, "clear_count", 0),
                        "median_cm": getattr(evidence, "median_clearance_cm", None),
                        "minimum_cm": getattr(evidence, "minimum_clearance_cm", None),
                    }

                def _perform_committed_turn(
                    current_skills: Any,
                    samples: list[Any],
                    corridor: Dict[str, Any],
                ) -> bool:
                    minimum = corridor.get("minimum_distance_cm")
                    if (
                        minimum is not None
                        and float(minimum) < danger_distance
                        and not navigation.reversed_for_current_obstacle
                    ):
                        reverse = current_skills.move_backward(distance_cm=5.0, speed=20)
                        if reverse.ok:
                            navigation.note_reverse()

                    sign, left, right, reason = choose_turn_direction(
                        samples,
                        safe_distance_cm=safe_distance,
                        committed_sign=navigation.committed_turn_sign,
                        default_sign=1,
                    )
                    committed = navigation.commit_turn(sign)
                    angle = committed * turn_step
                    self.log(
                        f"Front unavailable ({corridor.get('status')}): committed turn {angle:+.0f}°; "
                        f"reason={reason}; left={_side_summary(left)}; right={_side_summary(right)}"
                    )
                    rotation = current_skills.rotate(angle, speed=rotate_speed)
                    if not rotation.ok:
                        return False
                    navigation.note_turn()
                    if navigation.turns_without_progress >= max_turns_without_progress:
                        self.log(
                            "Explore stopped after the committed turn limit: "
                            "no forward progress, refusing to oscillate"
                        )
                        return False
                    return True

                while navigation.completed_moves < steps and navigation.actions < max_actions:
                    if self._mission_stop_event.is_set() or self._stop_threads.is_set():
                        break
                    if run_seconds is not None and time.time() - started >= run_seconds:
                        self.log("Explore area run time finished")
                        break
                    assert skills is not None
                    scan_id = f"explore-{int(started * 1000)}-{navigation.actions + 1}"
                    continue_running = True
                    with self._hardware_lock:
                        samples, corridor = _scan_with_retry(skills, scan_id=scan_id)
                        if corridor.get("safe"):
                            movement = skills.move_forward(
                                distance_cm=step_distance,
                                speed=speed,
                                require_recent_scan=True,
                            )
                            if movement_made_progress(movement, step_distance):
                                navigation.note_progress()
                                self.log(
                                    f"Explore forward progress {navigation.completed_moves}/{steps}; "
                                    f"travelled={movement.data.get('travelled_cm')} cm"
                                )
                            else:
                                navigation.note_failed_forward_action()
                                if (
                                    movement.status == "range_unknown"
                                    and navigation.may_retry_same_heading(limit=1)
                                ):
                                    self.log(
                                        "Forward range uncertain; rescan the same heading once "
                                        "before choosing a turn"
                                    )
                                else:
                                    failed_corridor = {
                                        "safe": False,
                                        "status": movement.status,
                                        "minimum_distance_cm": movement.data.get("obstacle_cm")
                                        or movement.data.get("distance_cm"),
                                    }
                                    continue_running = _perform_committed_turn(
                                        skills, samples, failed_corridor
                                    )
                        elif (
                            corridor.get("status") == "range_unknown"
                            and navigation.may_retry_same_heading(limit=1)
                        ):
                            self.log(
                                "Front corridor uncertain; hold heading and rescan once before turning"
                            )
                        else:
                            continue_running = _perform_committed_turn(skills, samples, corridor)

                    completed_steps = navigation.completed_moves
                    json_path, svg_path = _save(skills, completed_steps)
                    with self._lock:
                        self.state.mission.progress = {
                            "step": completed_steps,
                            "total_steps": steps,
                            "actions": navigation.actions,
                            "turns_without_progress": navigation.turns_without_progress,
                            "turn_commitment": navigation.committed_turn_sign,
                            "elapsed_s": round(time.time() - started, 1),
                            "latest_map": json_path.name,
                            "latest_svg": svg_path.name,
                        }
                        self.state.last_command = (
                            f"explore move {completed_steps}/{steps}; actions={navigation.actions}"
                        )
                    self.log(f"Explore map saved: {json_path.name}")
                    if not continue_running:
                        break

                if return_home and not self._mission_stop_event.is_set() and skills is not None:
                    with self._hardware_lock:
                        result = skills.return_home()
                    self.log(f"Return home: {result.message}")
                if skills is not None and not self._mission_stop_event.is_set():
                    with self._hardware_lock:
                        skills.set_led_status("success")
            except Exception as exc:
                returncode = 1
                self.log(f"Dashboard explore area failed: {type(exc).__name__}: {exc}")
                with self._lock:
                    self.state.last_error = f"mission failed: {type(exc).__name__}: {exc}"
            finally:
                if skills is not None:
                    try:
                        final_json, final_svg = _save(skills, completed_steps, final=True)
                        with self._lock:
                            self.state.mission.progress.update(
                                {"latest_map": final_json.name, "latest_svg": final_svg.name, "final": True}
                            )
                        self.log(f"Final explore map saved: {final_json.name}")
                    except Exception as exc:
                        self.log(f"final map save warning: {exc}")
                try:
                    with self._hardware_lock:
                        if self._motion is not None:
                            self._motion.brake_and_reset()
                except Exception as exc:
                    self.log(f"explore shutdown brake warning: {exc}")
                self._finish_mission("explore_area", returncode)
                self.log(f"Dashboard explore area finished with code {returncode}")

        thread = threading.Thread(target=_run, name="marsy-explore-area", daemon=True)
        self._mission_thread = thread
        thread.start()
        return self.snapshot_state()

    def stop_mission(self, reason: str = "manual") -> Dict[str, Any]:
        self._mission_stop_event.set()

        # Legacy subprocess cleanup if an older run somehow left one around.
        proc = self._mission_process
        if proc is not None and proc.poll() is None:
            self.log(f"Stopping legacy mission process: {reason}")
            try:
                proc.terminate()
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=1.0)
            except Exception as exc:
                self.log(f"legacy mission stop warning: {exc}")
        self._mission_process = None

        thread = self._mission_thread
        if thread is not None and thread.is_alive():
            self.log(f"Stopping dashboard mission thread: {reason}")
            thread.join(timeout=2.5)
            if thread.is_alive():
                self.log("mission thread did not stop immediately; watchdog will keep braking")
        self._mission_thread = None

        try:
            if self._hardware_lock.acquire(blocking=False):
                try:
                    if self._motion is not None:
                        self._motion.brake_and_reset()
                finally:
                    self._hardware_lock.release()
        except Exception as exc:
            self.log(f"mission stop brake warning: {exc}")

        with self._lock:
            self.state.mode = "manual"
            self.state.mission.running = False
            self._manual_active_command = None
            self.state.last_command = f"stop mission: {reason}"
        return self.snapshot_state()


class MarsyRequestHandler(BaseHTTPRequestHandler):
    server_version = "MarsyWeb/0.5"

    def _controller(self) -> RoverController:
        return self.server.controller  # type: ignore[attr-defined]

    def log_message(self, fmt: str, *args: Any) -> None:
        # Keep console readable; dashboard actions are logged separately.
        return

    def _send_bytes(self, data: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, payload: Dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self._send_bytes(data, "application/json; charset=utf-8", status=status)

    def _read_json(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            data = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            return {}
        return data if isinstance(data, dict) else {}

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == "/":
            return self._serve_file(STATIC_DIR / "index.html", "text/html; charset=utf-8")
        if path == "/maps":
            return self._serve_file(STATIC_DIR / "maps.html", "text/html; charset=utf-8")
        if path == "/static/app.js":
            return self._serve_file(STATIC_DIR / "app.js", "application/javascript; charset=utf-8")
        if path == "/static/style.css":
            return self._serve_file(STATIC_DIR / "style.css", "text/css; charset=utf-8")
        if path == "/static/maps.js":
            return self._serve_file(STATIC_DIR / "maps.js", "application/javascript; charset=utf-8")
        if path == "/static/maps.css":
            return self._serve_file(STATIC_DIR / "maps.css", "text/css; charset=utf-8")
        if path == "/static/missions.js":
            return self._serve_file(STATIC_DIR / "missions.js", "application/javascript; charset=utf-8")
        if path == "/static/missions.css":
            return self._serve_file(STATIC_DIR / "missions.css", "text/css; charset=utf-8")
        if path == "/api/missions":
            return self._send_json(self._controller().list_missions())
        if path == "/api/maps":
            return self._send_json(_list_maps())
        if path.startswith("/api/maps/"):
            data, error = _read_map(path[len("/api/maps/"):])
            if error is not None or data is None:
                status = HTTPStatus.NOT_FOUND if error == "map not found" else HTTPStatus.BAD_REQUEST
                return self._send_json({"error": error or "cannot read map"}, status=status)
            return self._send_json(data)
        if path.startswith("/maps/file/"):
            return self._serve_map_file(path[len("/maps/file/"):])
        if path == "/api/state":
            return self._send_json(self._controller().get_state())
        if path == "/video.mjpg":
            return self._serve_mjpeg()
        self._send_json({"error": "not found"}, status=HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        payload = self._read_json()
        controller = self._controller()
        if path == "/api/heartbeat":
            return self._send_json(controller.heartbeat())
        if path == "/api/command":
            return self._send_json(controller.command(payload))
        if path == "/api/lidar":
            return self._send_json(controller.lidar(payload))
        if path == "/api/stop":
            return self._send_json(controller.hard_stop())
        if path == "/api/mission/start":
            return self._send_json(controller.start_mission(payload))
        if path == "/api/mission/stop":
            return self._send_json(controller.stop_mission(reason="dashboard"))
        if path == "/api/shutdown":
            state = controller.request_shutdown_state(reason="dashboard button")
            self._send_json(state)
            self.server.request_shutdown("dashboard button")  # type: ignore[attr-defined]
            return
        self._send_json({"error": "not found"}, status=HTTPStatus.NOT_FOUND)

    def _serve_file(self, path: Path, content_type: str) -> None:
        if not path.exists():
            return self._send_json({"error": "missing file"}, status=HTTPStatus.NOT_FOUND)
        self._send_bytes(path.read_bytes(), content_type)

    def _serve_map_file(self, filename: str) -> None:
        path = _safe_map_path(filename, {".json", ".svg"})
        if path is None:
            return self._send_json({"error": "invalid map filename"}, status=HTTPStatus.BAD_REQUEST)
        if not path.exists() or not path.is_file():
            return self._send_json({"error": "map file not found"}, status=HTTPStatus.NOT_FOUND)
        if path.stat().st_size > MAP_MAX_BYTES:
            return self._send_json({"error": "map file is too large"}, status=HTTPStatus.REQUEST_ENTITY_TOO_LARGE)
        content_type = "application/json; charset=utf-8" if path.suffix.lower() == ".json" else "image/svg+xml; charset=utf-8"
        self._send_bytes(path.read_bytes(), content_type)

    def _serve_mjpeg(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=marsyframe")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            for chunk in self._controller().camera.mjpeg_frames():
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


class MarsyHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    controller: RoverController

    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs)
        self._shutdown_lock = threading.Lock()
        self._shutdown_started = False

    def request_shutdown(self, reason: str) -> None:
        with self._shutdown_lock:
            if self._shutdown_started:
                return
            self._shutdown_started = True

        def _shutdown() -> None:
            try:
                # Give the HTTP response a moment to flush before closing sockets.
                time.sleep(0.15)
                self.controller.log(f"Server shutdown thread started: {reason}")
                self.controller.cleanup()
                self.shutdown()
            except Exception as exc:
                try:
                    self.controller.log(f"server shutdown failed: {exc}")
                except Exception:
                    pass

        threading.Thread(target=_shutdown, daemon=True).start()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Marsy web dashboard")
    parser.add_argument("--host", default=os.getenv("MARSY_WEB_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.getenv("MARSY_WEB_PORT", "8080")))
    parser.add_argument("--camera-width", type=int, default=int(os.getenv("MARSY_CAMERA_WIDTH", "640")))
    parser.add_argument("--camera-height", type=int, default=int(os.getenv("MARSY_CAMERA_HEIGHT", "480")))
    parser.add_argument("--camera-fps", type=float, default=float(os.getenv("MARSY_CAMERA_FPS", "8")))
    parser.add_argument(
        "--camera-rotation",
        type=int,
        default=int(os.getenv("MARSY_CAMERA_ROTATION", "90")),
        choices=(0, 90, 180, 270),
        help="Clockwise camera rotation applied in the camera pipeline/code.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    stale_maps = _clear_map_artifacts()
    # atexit covers ordinary interpreter shutdown in addition to the explicit
    # server cleanup paths below. SIGKILL/power loss is handled next startup.
    atexit.register(_clear_map_artifacts)
    camera = CameraStream(
        width=args.camera_width,
        height=args.camera_height,
        fps=args.camera_fps,
        rotation_deg=args.camera_rotation,
    )
    controller = RoverController(camera=camera)
    server = MarsyHTTPServer((args.host, args.port), MarsyRequestHandler)
    server.controller = controller

    def request_shutdown_from_signal(signum: int, _frame: Any) -> None:
        signal_name = signal.Signals(signum).name
        controller.log(f"Signal received: {signal_name}")
        with controller._lock:
            controller.state.shutdown_requested = True
            controller.state.last_command = f"shutdown requested: {signal_name}"
        server.request_shutdown(signal_name)

    signal.signal(signal.SIGINT, request_shutdown_from_signal)
    signal.signal(signal.SIGTERM, request_shutdown_from_signal)

    controller.log(f"Marsy dashboard: http://{args.host}:{args.port}")
    controller.log(f"Backend mode: {DEFAULT_MODE}")
    if stale_maps:
        controller.log(f"Removed {stale_maps} stale map files at startup")
    controller.log(f"Camera rotation: {args.camera_rotation} deg clockwise")
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        controller.cleanup()
        server.server_close()
        _clear_map_artifacts()
        controller.log("Server socket closed")


if __name__ == "__main__":
    main()
