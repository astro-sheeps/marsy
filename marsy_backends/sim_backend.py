"""
Marsy simulator backend.

This backend is intentionally self-contained: it does not import the original
external 4tronix rover simulator package. It talks to our own PyQt/Flask
simulator at simulator/marsy_sim_ui.py over HTTP.

Expected simulator endpoints:
    POST /          accepts rover commands: wheelMotors, servos, rgbLeds
    GET  /state     returns simulator state and sonar distance
    GET  /distance  returns distance_cm / ultrasonicRange
    POST /control   optional control endpoint: stop/reset/manual

Used by:
    from marsy_backends.sim_backend import rover

Build stamp:
    SIM_BACKEND_NO_EXTERNAL_V1
"""

from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Mapping, Optional

BUILD_STAMP = "SIM_BACKEND_NO_EXTERNAL_V1"

SIM_HOST = os.getenv("MARSY_SIM_HOST", "127.0.0.1")
SIM_PORT = int(os.getenv("MARSY_SIM_PORT", "8523"))
SIM_BASE_URL = os.getenv("MARSY_SIM_BASE_URL", f"http://{SIM_HOST}:{SIM_PORT}")
SIM_TIMEOUT_S = float(os.getenv("MARSY_SIM_TIMEOUT", "0.25"))
SIM_VERBOSE = os.getenv("MARSY_SIM_VERBOSE", "0") == "1"


def _debug(message: str) -> None:
    if SIM_VERBOSE:
        print(f"[marsy sim_backend] {message}", flush=True)


def _url(path: str) -> str:
    if not path.startswith("/"):
        path = "/" + path
    return SIM_BASE_URL.rstrip("/") + path


def _post_json(path: str, payload: Mapping[str, Any], timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
    """POST JSON to the simulator and return JSON response if available."""
    if timeout is None:
        timeout = SIM_TIMEOUT_S

    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        _url(path),
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            raw = response.read()
            if not raw:
                return None
            try:
                return json.loads(raw.decode("utf-8"))
            except json.JSONDecodeError:
                return None
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        _debug(f"POST {path} failed: {exc}")
        return None


def _get_json(path: str, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
    """GET JSON from the simulator and return dict if available."""
    if timeout is None:
        timeout = SIM_TIMEOUT_S

    try:
        with urllib.request.urlopen(_url(path), timeout=timeout) as response:
            raw = response.read()
            if not raw:
                return None
            try:
                data = json.loads(raw.decode("utf-8"))
            except json.JSONDecodeError:
                return None
            return data if isinstance(data, dict) else None
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        _debug(f"GET {path} failed: {exc}")
        return None


def _clamp_speed(speed: Any) -> int:
    try:
        value = int(speed)
    except (TypeError, ValueError):
        value = 0
    return max(0, min(100, value))


def _clamp_servo_angle(degrees: Any) -> int:
    try:
        value = int(round(float(degrees)))
    except (TypeError, ValueError):
        value = 0
    return max(-90, min(90, value))


def _parse_distance(value: Any) -> Optional[float]:
    try:
        distance = float(value)
    except (TypeError, ValueError):
        return None
    return distance


@dataclass
class SimRover:
    """
    Small rover-like object compatible with the original 4tronix rover API.

    It intentionally implements only the calls used by Marsy core, missions,
    and simple tests. All hardware actions are translated into HTTP commands
    understood by simulator/marsy_sim_ui.py.
    """

    numPixels: int = 4
    offsets: list[int] = field(default_factory=lambda: [0] * 16)

    _brightness: int = 0
    _initialized: bool = False
    _leds: list[list[int]] = field(default_factory=lambda: [[0, 0, 0] for _ in range(4)])
    _last_distance_sequence: list[float] = field(default_factory=list)
    _last_distance_index: int = 0

    # ------------------------------------------------------------------
    # General API
    # ------------------------------------------------------------------

    def init(self, brightness: int = 0, PiBit: bool = False) -> None:  # noqa: N803 - keep 4tronix arg name
        self._brightness = int(brightness or 0)
        self._initialized = True
        _debug(f"init(brightness={brightness}, PiBit={PiBit}) using {SIM_BASE_URL}")
        # Keep init light-touch: do not force reset, because missions may set mode.
        _get_json("/state")

    def cleanup(self) -> None:
        # Do not call any external simulator cleanup. Just stop motors and servos.
        self.stop()
        self.stopServos()
        self._initialized = False

    def version(self) -> int:
        # Original 4tronix M.A.R.S. rover reports model 4.
        return 4

    # ------------------------------------------------------------------
    # Motor API
    # ------------------------------------------------------------------

    def _send_wheel_motors(self, left_fwd: int, left_rev: int, right_fwd: int, right_rev: int) -> None:
        payload = {
            "wheelMotors": {
                "l": [_clamp_speed(left_fwd), _clamp_speed(left_rev)],
                "r": [_clamp_speed(right_fwd), _clamp_speed(right_rev)],
            }
        }
        _post_json("/", payload)

    def stop(self) -> None:
        self._send_wheel_motors(0, 0, 0, 0)

    def brake(self) -> None:
        # In the simulator, fwd+rev together maps to speed=0, matching a hard stop.
        self._send_wheel_motors(100, 100, 100, 100)

    def forward(self, speed: int) -> None:
        speed = _clamp_speed(speed)
        self._send_wheel_motors(speed, 0, speed, 0)

    def reverse(self, speed: int) -> None:
        speed = _clamp_speed(speed)
        self._send_wheel_motors(0, speed, 0, speed)

    def spinLeft(self, speed: int) -> None:  # noqa: N802 - keep original API name
        speed = _clamp_speed(speed)
        self._send_wheel_motors(0, speed, speed, 0)

    def spinRight(self, speed: int) -> None:  # noqa: N802 - keep original API name
        speed = _clamp_speed(speed)
        self._send_wheel_motors(speed, 0, 0, speed)

    def turnForward(self, leftSpeed: int, rightSpeed: int) -> None:  # noqa: N802/N803
        self._send_wheel_motors(_clamp_speed(leftSpeed), 0, _clamp_speed(rightSpeed), 0)

    def turnReverse(self, leftSpeed: int, rightSpeed: int) -> None:  # noqa: N802/N803
        self._send_wheel_motors(0, _clamp_speed(leftSpeed), 0, _clamp_speed(rightSpeed))

    # ------------------------------------------------------------------
    # Servo API
    # ------------------------------------------------------------------

    def setServo(self, servo: int, degrees: int) -> None:  # noqa: N802
        servo_id = int(servo)
        angle = _clamp_servo_angle(degrees)
        _post_json("/", {"servos": {str(servo_id): angle}})

    def setServos(self, servo_angles: Mapping[int, int]) -> None:  # noqa: N802
        servos = {
            str(int(servo)): _clamp_servo_angle(angle)
            for servo, angle in dict(servo_angles).items()
        }
        if servos:
            _post_json("/", {"servos": servos})

    def stopServo(self, servo: int) -> None:  # noqa: N802
        # The simulator has no servo PWM hold/release distinction. Keep as no-op.
        return None

    def stopServos(self) -> None:  # noqa: N802
        # The simulator has no servo PWM hold/release distinction. Keep as no-op.
        return None

    def loadOffsets(self) -> None:  # noqa: N802
        return None

    def saveOffsets(self) -> None:  # noqa: N802
        return None

    # ------------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------------

    def getDistance(self) -> float:  # noqa: N802
        """Return simulated ultrasonic distance in cm. 0 means no object/no echo."""
        # First preference: live simulator endpoint.
        data = _get_json("/distance")
        if data is not None:
            distance = _parse_distance(data.get("distance_cm", data.get("ultrasonicRange")))
            if distance is not None:
                return distance

        # Fallback: full state endpoint.
        data = _get_json("/state")
        if data is not None:
            distance = _parse_distance(data.get("distance_cm", data.get("ultrasonicRange")))
            if distance is not None:
                return distance

        # Offline fallback for tests: MARSY_SIM_DISTANCE_SEQUENCE="100,80,50,20".
        sequence = os.getenv("MARSY_SIM_DISTANCE_SEQUENCE", "").strip()
        if sequence:
            if not self._last_distance_sequence:
                values: list[float] = []
                for part in sequence.split(","):
                    parsed = _parse_distance(part.strip())
                    if parsed is not None:
                        values.append(parsed)
                self._last_distance_sequence = values or [0.0]
                self._last_distance_index = 0

            value = self._last_distance_sequence[min(self._last_distance_index, len(self._last_distance_sequence) - 1)]
            self._last_distance_index += 1
            return value

        # Offline fallback for tests: MARSY_SIM_DISTANCE_CM=100.
        env_distance = _parse_distance(os.getenv("MARSY_SIM_DISTANCE_CM"))
        if env_distance is not None:
            return env_distance

        return 0.0

    def getBattery(self) -> float:  # noqa: N802
        data = _get_json("/state")
        if data is not None:
            percent = _parse_distance(data.get("battery_percent"))
            if percent is not None:
                # Convert fake percent to a plausible 4xAA rover pack voltage.
                return round(6.0 + (percent / 100.0) * 2.4, 2)
        return 7.8

    def getSwitch(self) -> bool:  # noqa: N802
        return False

    def getKey(self) -> int:  # noqa: N802
        return 0

    # ------------------------------------------------------------------
    # RGB LED compatibility
    # ------------------------------------------------------------------

    def fromRGB(self, red: int, green: int, blue: int) -> int:  # noqa: N802
        return ((int(red) & 0xFF) << 16) + ((int(green) & 0xFF) << 8) + (int(blue) & 0xFF)

    def toRGB(self, color: int) -> tuple[int, int, int]:  # noqa: N802
        color = int(color)
        return ((color & 0xFF0000) >> 16, (color & 0x00FF00) >> 8, color & 0x0000FF)

    def setPixel(self, ID: int, color: int) -> None:  # noqa: N802/N803
        led_id = int(ID)
        if not 0 <= led_id < self.numPixels:
            return
        self._leds[led_id] = list(self.toRGB(color))

    def setColor(self, color: int) -> None:  # noqa: N802
        rgb = list(self.toRGB(color))
        for led_id in range(self.numPixels):
            self._leds[led_id] = rgb[:]

    def show(self) -> None:
        payload = {
            "rgbLeds": {
                str(i): self._leds[i]
                for i in range(min(self.numPixels, len(self._leds)))
            }
        }
        _post_json("/", payload)

    def clear(self) -> None:
        self._leds = [[0, 0, 0] for _ in range(self.numPixels)]

    def wheel(self, pos: int) -> int:
        pos = int(pos) % 256
        if pos < 85:
            return self.fromRGB(255 - pos * 3, pos * 3, 0)
        if pos < 170:
            pos -= 85
            return self.fromRGB(0, 255 - pos * 3, pos * 3)
        pos -= 170
        return self.fromRGB(pos * 3, 0, 255 - pos * 3)

    def rainbow(self) -> None:
        for i in range(self.numPixels):
            self.setPixel(i, self.wheel(int(i * 256 / max(1, self.numPixels))))

    # ------------------------------------------------------------------
    # Optional helpers for Marsy-specific code
    # ------------------------------------------------------------------

    def get_state(self) -> Dict[str, Any]:
        return _get_json("/state") or {}

    def simulator_stop_requested(self) -> bool:
        state = self.get_state()
        control = state.get("control", {}) if isinstance(state, dict) else {}
        return bool(state.get("mission_stop_requested") or control.get("mission_stop_requested"))

    def control(self, action: str, **extra: Any) -> Dict[str, Any]:
        payload = {"action": action}
        payload.update(extra)
        return _post_json("/control", payload) or {}


# Object imported by marsy_backends.loader.load_rover().
rover = SimRover()
