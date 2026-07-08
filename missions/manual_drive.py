"""
Manual driving mission for Marsy.

Project location:
    missions/manual_drive.py

This file is intentionally backend-neutral:
    MARSY_MODE=sim  -> simulator backend
    MARSY_MODE=real -> real 4tronix rover backend

Manual drive uses the shared Marsy layers:
    - marsy_backends.loader.load_rover
    - marsy_core.motion.MarsyMotion
    - marsy_core.sensors.MarsySensors
    - marsy_core.safety.MarsySafety
    - marsy_core.telemetry.log

The background safety monitor keeps checking the front ultrasonic distance while
manual_drive is waiting for keyboard input. This matters on the real rover:
after pressing 'w', the terminal waits for the next key, but the rover may still
be moving.
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass

from marsy_backends.loader import load_rover
from marsy_core.motion import MarsyMotion
from marsy_core.safety import MarsySafety
from marsy_core.sensors import MarsySensors
from marsy_core.telemetry import log, simulator_stop_requested


SOURCE = "manual_drive"
MANUAL_DRIVE_BUILD_STAMP = "manual_drive_ui_stop_v5_2026_07_08"


# -----------------------------------------------------------------------------
# Keyboard input
# -----------------------------------------------------------------------------


def readchar(timeout_s: float | None = None) -> str:
    """
    Read one character from terminal without waiting for Enter.

    If timeout_s is provided, return "" when no key is pressed before the
    timeout. This lets manual_drive poll simulator /state while still accepting
    keyboard controls.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)

        if timeout_s is not None:
            ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
            if not ready:
                return ""

        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    if not ch:
        return ""

    if ord(ch) == 3:
        raise KeyboardInterrupt

    return ch



def readkey(timeout_s: float | None = None) -> str:
    """
    Read normal keys and arrow keys.

    Arrow key convention follows the original 4tronix examples:
        chr(16) = Up
        chr(17) = Down
        chr(18) = Right
        chr(19) = Left

    Returns "" when timeout_s expires without a key.
    """
    c1 = readchar(timeout_s=timeout_s)

    if not c1:
        return ""

    # Normal key.
    if ord(c1) != 0x1B:
        return c1

    # Escape sequence for arrows. Use short follow-up timeouts so a lone Esc
    # does not block the mission-control polling loop.
    c2 = readchar(timeout_s=0.05)
    if not c2 or ord(c2) != 0x5B:
        return c1

    c3 = readchar(timeout_s=0.05)
    if not c3:
        return c1

    return chr(0x10 + ord(c3) - 65)


# -----------------------------------------------------------------------------
# Shared drive state
# -----------------------------------------------------------------------------


@dataclass
class BlockedState:
    active: bool = False
    distance_cm: float | None = None
    reason: str = ""


class DriveState:
    """Thread-safe state shared by keyboard loop and safety monitor."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.direction = "stopped"  # stopped | forward | reverse
        self.blocked = BlockedState()

    def set_direction(self, direction: str) -> None:
        with self._lock:
            self.direction = direction

    def get_direction(self) -> str:
        with self._lock:
            return self.direction

    def set_blocked(self, distance_cm: float | None, reason: str) -> None:
        with self._lock:
            self.blocked = BlockedState(
                active=True,
                distance_cm=distance_cm,
                reason=reason,
            )

    def clear_blocked(self) -> None:
        with self._lock:
            self.blocked = BlockedState()

    def get_blocked(self) -> BlockedState:
        with self._lock:
            return BlockedState(
                active=self.blocked.active,
                distance_cm=self.blocked.distance_cm,
                reason=self.blocked.reason,
            )


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------


def is_obstacle_too_close(distance_cm: float | None, safe_distance_cm: float) -> bool:
    """
    Decide whether the front ultrasonic reading should block forward movement.

    In the original 4tronix-style API, 0 or negative values can mean no echo /
    no object. Treat them as not blocked.
    """
    if distance_cm is None:
        return False

    if distance_cm <= 0:
        return False

    return distance_cm < safe_distance_cm



def format_distance(distance_cm: float | None) -> str:
    if distance_cm is None:
        return "None"
    try:
        return f"{float(distance_cm):.1f} cm"
    except (TypeError, ValueError):
        return str(distance_cm)



def say(message: str, *, level: str = "info", event: str | None = None, **fields) -> None:
    """Print to terminal and mirror into simulator telemetry panel."""
    log(message, source=SOURCE, level=level, event=event, **fields)


# -----------------------------------------------------------------------------
# Background safety monitor
# -----------------------------------------------------------------------------


class ManualSafetyMonitor:
    """
    Continuously checks safety while manual_drive waits for keyboard input.

    Obstacle handling is intentionally conservative and real-rover-safe:
        - forward is blocked when an obstacle is too close
        - reverse remains allowed, so the rover can escape
        - the manual emergency-stop key still blocks all movement until reset
    """

    def __init__(
        self,
        motion: MarsyMotion,
        sensors: MarsySensors,
        safety: MarsySafety,
        drive_state: DriveState,
        check_interval_s: float = 0.15,
    ) -> None:
        self.motion = motion
        self.sensors = sensors
        self.safety = safety
        self.drive_state = drive_state
        self.check_interval_s = check_interval_s
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self._last_block_report_time = 0.0
        self._last_clear_report_time = 0.0

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        say(
            "Manual safety monitor started",
            event="safety_monitor_started",
            interval_s=self.check_interval_s,
        )

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        say("Manual safety monitor stopped", event="safety_monitor_stopped")

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                self._check_once()
            except Exception as exc:  # keep safety monitor alive
                say(
                    f"Safety monitor error: {exc}",
                    level="warning",
                    event="safety_monitor_error",
                    error=str(exc),
                )

            time.sleep(self.check_interval_s)

    def _check_once(self) -> None:
        direction = self.drive_state.get_direction()

        # Timeout protection for active driving.
        if direction in {"forward", "reverse"}:
            self.safety.check_drive_timeout()

            if self.safety.emergency_stopped:
                self.motion.stop()
                self.drive_state.set_direction("stopped")
                say(
                    "Emergency stop active: motors stopped",
                    level="warning",
                    event="emergency_stop_active",
                )
                return

        # Obstacle protection only blocks forward movement.
        # Reverse must remain allowed so the rover can back away.
        if direction != "forward":
            return

        distance = self.sensors.distance_cm()

        if is_obstacle_too_close(distance, self.safety.safe_distance_cm):
            self.motion.stop()
            self.safety.clear_drive_timer()
            self.drive_state.set_direction("stopped")
            self.drive_state.set_blocked(distance, "front_obstacle")

            now = time.time()
            if now - self._last_block_report_time > 0.8:
                say(
                    f"BLOCKED: obstacle at {format_distance(distance)}. Reverse or steer away.",
                    level="warning",
                    event="blocked_front_obstacle",
                    distance_cm=distance,
                    safe_distance_cm=self.safety.safe_distance_cm,
                )
                self._last_block_report_time = now
            return

        # If the previous state was blocked but the reading is now safe, clear it.
        if self.drive_state.get_blocked().active:
            self.drive_state.clear_blocked()
            now = time.time()
            if now - self._last_clear_report_time > 0.8:
                say(
                    f"Path clear: distance {format_distance(distance)}",
                    event="path_clear",
                    distance_cm=distance,
                )
                self._last_clear_report_time = now


# -----------------------------------------------------------------------------
# UI text
# -----------------------------------------------------------------------------


def print_help(args: argparse.Namespace, speed: int) -> None:
    say("Marsy manual drive", event="help")
    say("------------------", event="help")
    say("w / ↑      forward", event="help")
    say("z / ↓      reverse", event="help")
    say("a / ←      steer left", event="help")
    say("s / →      steer right", event="help")
    say("space      stop", event="help")
    say("b          brake", event="help")
    say("e          emergency stop", event="help")
    say("r          reset emergency stop", event="help")
    say("c          center wheels", event="help")
    say("j          mast left", event="help")
    say("l          mast right", event="help")
    say("k          mast center", event="help")
    say("d          read distance", event="help")
    say(", / <      speed down", event="help")
    say(". / >      speed up", event="help")
    say("h          help", event="help")
    say("q          quit", event="help")
    say(
        (
            f"Settings: speed={speed}, steer_angle={args.steer_angle}, "
            f"mast_angle={args.mast_angle}, safe_distance={args.safe_distance} cm, "
            f"max_drive_seconds={args.max_drive_seconds} s"
        ),
        event="help_settings",
    )


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Marsy manual drive mission")

    parser.add_argument("--speed", type=int, default=60)
    parser.add_argument("--max-speed", type=int, default=70)
    parser.add_argument("--steer-angle", type=int, default=20)
    parser.add_argument("--mast-angle", type=int, default=45)
    parser.add_argument("--safe-distance", type=float, default=30.0)
    parser.add_argument("--max-drive-seconds", type=float, default=30.0)
    parser.add_argument("--monitor-interval", type=float, default=0.15)
    parser.add_argument("--ui-stop-poll-interval", type=float, default=0.15)

    return parser



def main() -> None:
    args = build_arg_parser().parse_args()

    rover = load_rover()
    motion = MarsyMotion(rover)
    sensors = MarsySensors(rover)
    safety = MarsySafety(
        rover,
        max_speed=args.max_speed,
        max_drive_seconds=args.max_drive_seconds,
        safe_distance_cm=args.safe_distance,
    )

    drive_state = DriveState()
    monitor = ManualSafetyMonitor(
        motion=motion,
        sensors=sensors,
        safety=safety,
        drive_state=drive_state,
        check_interval_s=args.monitor_interval,
    )

    speed = safety.clamp_speed(args.speed)

    try:
        rover.init(0)
        motion.wheels_straight()
        motion.mast_center()

        say("Manual drive started", event="manual_started", build=MANUAL_DRIVE_BUILD_STAMP)
        try:
            say(
                f"Mast calibration: {motion.mast_debug_config()}",
                event="mast_calibration",
                **motion.mast_debug_config(),
            )
        except Exception:
            pass

        monitor.start()
        print_help(args, speed)

        while True:
            # Top STOP in marsy_sim_ui.py sets mission_stop_requested in /state.
            # manual_drive used to block forever inside stdin.read(1), so the UI
            # STOP button could stop the simulated motors but not end this
            # mission process. readkey(timeout_s=...) now returns periodically,
            # allowing this poll to terminate the mission cleanly.
            if simulator_stop_requested():
                motion.stop()
                safety.clear_drive_timer()
                drive_state.set_direction("stopped")
                drive_state.clear_blocked()
                say(
                    "Manual drive stopped by simulator UI",
                    level="warning",
                    event="manual_ui_stop_requested",
                )
                break

            key = readkey(timeout_s=args.ui_stop_poll_interval)
            if not key:
                continue

            # -----------------------------------------------------------------
            # Forward
            # -----------------------------------------------------------------
            if key == "w" or (key and ord(key) == 16):
                if safety.emergency_stopped:
                    say(
                        "Emergency stop is active. Press r to reset.",
                        level="warning",
                        event="forward_refused_emergency_stop",
                    )
                    continue

                distance = sensors.distance_cm()

                if is_obstacle_too_close(distance, safety.safe_distance_cm):
                    motion.stop()
                    safety.clear_drive_timer()
                    drive_state.set_direction("stopped")
                    drive_state.set_blocked(distance, "front_obstacle")
                    say(
                        f"BLOCKED: obstacle at {format_distance(distance)}. Reverse or steer away.",
                        level="warning",
                        event="forward_blocked_front_obstacle",
                        distance_cm=distance,
                        safe_distance_cm=safety.safe_distance_cm,
                    )
                    continue

                safe_speed = safety.clamp_speed(speed)
                motion.forward(safe_speed, straighten=False)
                safety.mark_drive_command()
                drive_state.set_direction("forward")
                drive_state.clear_blocked()
                say(
                    f"Forward {safe_speed}",
                    event="forward",
                    speed=safe_speed,
                    distance_cm=distance,
                )

            # -----------------------------------------------------------------
            # Reverse
            # -----------------------------------------------------------------
            elif key == "z" or (key and ord(key) == 17):
                if safety.emergency_stopped:
                    say(
                        "Emergency stop is active. Press r to reset.",
                        level="warning",
                        event="reverse_refused_emergency_stop",
                    )
                    continue

                safe_speed = safety.clamp_speed(speed)
                motion.reverse(safe_speed, straighten=False)
                safety.mark_drive_command()
                drive_state.set_direction("reverse")
                drive_state.clear_blocked()
                say(f"Reverse {safe_speed}", event="reverse", speed=safe_speed)

            # -----------------------------------------------------------------
            # Steering
            # -----------------------------------------------------------------
            elif key == "a" or (key and ord(key) == 19):
                motion.steer_left(args.steer_angle)
                say(
                    f"Steer left {args.steer_angle}",
                    event="steer_left",
                    angle=args.steer_angle,
                )

            elif key == "s" or (key and ord(key) == 18):
                motion.steer_right(args.steer_angle)
                say(
                    f"Steer right {args.steer_angle}",
                    event="steer_right",
                    angle=args.steer_angle,
                )

            elif key == "c":
                motion.wheels_straight()
                say("Wheels centered", event="wheels_centered")

            # -----------------------------------------------------------------
            # Stop / brake / emergency
            # -----------------------------------------------------------------
            elif key == " ":
                motion.stop()
                safety.clear_drive_timer()
                drive_state.set_direction("stopped")
                drive_state.clear_blocked()
                say("Stop", event="stop")

            elif key == "b":
                motion.brake()
                motion.wheels_straight()
                safety.clear_drive_timer()
                drive_state.set_direction("stopped")
                drive_state.clear_blocked()
                say("Brake", event="brake")

            elif key == "e":
                motion.stop()
                safety.emergency_stop("manual key")
                safety.clear_drive_timer()
                drive_state.set_direction("stopped")
                say(
                    "EMERGENCY STOP: manual key",
                    level="warning",
                    event="manual_emergency_stop",
                )

            elif key == "r":
                safety.reset_emergency_stop()
                safety.clear_drive_timer()
                drive_state.set_direction("stopped")
                drive_state.clear_blocked()
                say("Emergency stop reset", event="emergency_stop_reset")

            # -----------------------------------------------------------------
            # Mast
            # -----------------------------------------------------------------
            elif key == "j":
                motion.mast_left(args.mast_angle)
                say(
                    f"Mast left {args.mast_angle}",
                    event="mast_left",
                    logical_angle=-abs(args.mast_angle),
                )

            elif key == "l":
                motion.mast_right(args.mast_angle)
                say(
                    f"Mast right {args.mast_angle}",
                    event="mast_right",
                    logical_angle=abs(args.mast_angle),
                )

            elif key == "k":
                motion.mast_center()
                say("Mast center", event="mast_center")

            # -----------------------------------------------------------------
            # Sensor readout
            # -----------------------------------------------------------------
            elif key == "d":
                distance = sensors.distance_cm()
                say(
                    f"Distance: {format_distance(distance)}",
                    event="distance_read",
                    distance_cm=distance,
                )

            # -----------------------------------------------------------------
            # Speed
            # -----------------------------------------------------------------
            elif key == "." or key == ">":
                speed = safety.clamp_speed(min(100, speed + 10))
                say(f"Speed up: {speed}", event="speed_up", speed=speed)

            elif key == "," or key == "<":
                speed = safety.clamp_speed(max(0, speed - 10))
                say(f"Speed down: {speed}", event="speed_down", speed=speed)

            # -----------------------------------------------------------------
            # Help / quit
            # -----------------------------------------------------------------
            elif key == "h":
                print_help(args, speed)

            elif key == "q":
                say("Quit", event="quit")
                break

            elif key:
                say(
                    f"Unknown key: {repr(key)}. Press h for help.",
                    level="warning",
                    event="unknown_key",
                    key=repr(key),
                )

    except KeyboardInterrupt:
        say("Interrupted", level="warning", event="keyboard_interrupt")

    finally:
        say("Cleanup...", event="cleanup_started")

        try:
            monitor.stop()
        except Exception as exc:
            say(
                f"Safety monitor stop failed: {exc}",
                level="warning",
                event="monitor_stop_failed",
                error=str(exc),
            )

        try:
            motion.stop()
            motion.wheels_straight()
            motion.mast_center()
            rover.cleanup()
            say("Cleaned up", event="cleanup_done")
        except Exception as exc:
            say(
                f"Cleanup failed: {exc}",
                level="error",
                event="cleanup_failed",
                error=str(exc),
            )


if __name__ == "__main__":
    main()
