"""
Reusable obstacle-avoidance behavior for Marsy.

Project location:
    behaviors/avoid_obstacle.py

This module is backend-agnostic. It talks only to MarsyMotion and MarsySensors,
so the same behavior works with:

    MARSY_MODE=sim   -> distance comes from simulator backend
    MARSY_MODE=real  -> distance comes from the real ultrasonic sensor
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from statistics import median
from typing import Dict, Optional

from marsy_core.telemetry import TELEMETRY_BUILD_STAMP as CORE_TELEMETRY_BUILD_STAMP
from marsy_core.telemetry import log as telemetry_log
from marsy_core.telemetry import simulator_stop_requested

AVOID_OBSTACLE_TELEMETRY_BUILD_STAMP = "avoid_obstacle_control_stop_v4_2026_07_08"


class MissionStopRequested(Exception):
    """Raised when the simulator UI asks an autonomous mission to stop."""


@dataclass
class AvoidObstacleConfig:
    """Tunable parameters for reactive obstacle avoidance."""

    # Distance thresholds, cm.
    safe_distance_cm: float = 45.0
    danger_distance_cm: float = 25.0
    max_valid_distance_cm: float = 250.0

    # Ultrasonic sensors often return 0 / None when there is no echo.
    # For Marsy this is usually safer to interpret as "nothing detected nearby",
    # not as an object at zero distance.
    no_echo_means_clear: bool = True

    # Conservative default speeds for the real rover.
    forward_speed: int = 35
    turn_speed: int = 30
    reverse_speed: int = 25

    # Steering and mast angles.
    steer_angle: int = 28
    scan_angle: int = 55

    # Timing.
    loop_delay_s: float = 0.15
    sample_delay_s: float = 0.04
    scan_settle_s: float = 0.35
    stop_pause_s: float = 0.15
    reverse_time_s: float = 0.55
    turn_time_s: float = 1.05

    # During longer manoeuvres we keep checking the sonar.
    safety_check_interval_s: float = 0.12

    # Number of ultrasonic readings used for one distance estimate.
    distance_samples: int = 3

    # Runtime. None means run until Ctrl+C.
    run_seconds: Optional[float] = None

    # Tie-break if left and right look equally clear.
    default_turn_direction: str = "right"  # "left" or "right"

    verbose: bool = True


class AvoidObstacleBehavior:
    """
    Reactive obstacle avoidance.

    The controller is deliberately simple:

    1. Read distance straight ahead.
    2. If clear, drive forward.
    3. If blocked, stop.
    4. If dangerously close, reverse briefly.
    5. Scan left / center / right with the mast.
    6. Turn toward the side with more free space.

    It does not know whether it runs in the simulator or on the real rover.
    """

    def __init__(self, motion, sensors, config: Optional[AvoidObstacleConfig] = None):
        self.motion = motion
        self.sensors = sensors
        self.config = config or AvoidObstacleConfig()
        self.last_distance_cm: Optional[float] = None
        self.last_scan: Dict[str, Optional[float]] = {
            "left": None,
            "center": None,
            "right": None,
        }
        self.last_action = "init"
        self.log(
            "AvoidObstacle telemetry active: "
            f"{AVOID_OBSTACLE_TELEMETRY_BUILD_STAMP}, core={CORE_TELEMETRY_BUILD_STAMP}",
            event="telemetry_active",
        )

    # ------------------------------------------------------------------
    # Logging and distance helpers
    # ------------------------------------------------------------------

    def log(self, message: str, *, level: str = "info", event: Optional[str] = None, **fields) -> None:
        """Print to terminal and mirror into simulator telemetry panel."""
        if self.config.verbose:
            telemetry_log(
                message,
                source="avoid_obstacle",
                level=level,
                event=event,
                **fields,
            )

    def check_external_stop(self) -> None:
        """Stop gracefully if simulator UI top STOP was pressed."""
        if simulator_stop_requested():
            self.stop(pause=False)
            self.log(
                "Mission stop requested from simulator UI",
                level="warning",
                event="mission_stop_requested",
            )
            raise MissionStopRequested()

    def _is_valid_distance(self, distance: Optional[float]) -> bool:
        if distance is None:
            return False
        if distance <= 0:
            return False
        if distance > self.config.max_valid_distance_cm:
            return False
        return True

    def _distance_score(self, distance: Optional[float]) -> float:
        """Higher score means more free space."""
        if self._is_valid_distance(distance):
            return float(distance)

        if self.config.no_echo_means_clear:
            return float(self.config.max_valid_distance_cm)

        return 0.0

    def read_distance(self, samples: Optional[int] = None) -> Optional[float]:
        """Read ultrasonic distance several times and return median valid value."""
        if samples is None:
            samples = self.config.distance_samples

        values = []

        for _ in range(max(1, samples)):
            self.check_external_stop()
            try:
                value = self.sensors.distance_cm()
            except Exception as exc:
                self.log(f"Distance read failed: {exc}", level="warning", event="distance_read_failed", error=str(exc))
                value = None

            if self._is_valid_distance(value):
                values.append(float(value))

            time.sleep(self.config.sample_delay_s)

        if not values:
            self.last_distance_cm = None
            return None

        self.last_distance_cm = round(float(median(values)), 1)
        return self.last_distance_cm

    def is_close(self, distance: Optional[float]) -> bool:
        return self._is_valid_distance(distance) and distance < self.config.safe_distance_cm

    def is_dangerous(self, distance: Optional[float]) -> bool:
        return self._is_valid_distance(distance) and distance < self.config.danger_distance_cm

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------

    def stop(self, pause: bool = True) -> None:
        self.motion.stop()
        self.last_action = "stop"
        if pause:
            time.sleep(self.config.stop_pause_s)

    def _drive_for(self, duration_s: float, check_front: bool = True) -> Optional[float]:
        """
        Let the currently active motion continue for a short time.

        On the real rover, even a one-second manoeuvre is long enough to hit
        something. Therefore we split the duration into small slices and keep
        checking front distance. Returns the first dangerous distance if the
        manoeuvre was interrupted, otherwise None.
        """
        end_time = time.time() + max(0.0, duration_s)

        while time.time() < end_time:
            self.check_external_stop()
            sleep_time = min(self.config.safety_check_interval_s, end_time - time.time())
            if sleep_time > 0:
                time.sleep(sleep_time)

            if not check_front:
                continue

            distance = self.read_distance(samples=1)
            if self.is_dangerous(distance):
                self.stop()
                self.log(f"Interrupted manoeuvre: obstacle at {distance:.1f} cm", level="warning", event="maneuver_interrupted", distance_cm=distance)
                return distance

        return None

    def reverse_briefly(self) -> None:
        self.log("Action: reverse briefly", event="reverse_briefly")
        self.motion.reverse(self.config.reverse_speed, straighten=True)
        self.last_action = "reverse"
        self._drive_for(self.config.reverse_time_s, check_front=False)
        self.stop()

    def turn_left(self) -> None:
        self.log("Action: turn left", event="turn_left")
        self.motion.forward_left(
            speed=self.config.turn_speed,
            angle=self.config.steer_angle,
        )
        self.last_action = "turn_left"
        self._drive_for(self.config.turn_time_s, check_front=True)
        self.stop()
        self.motion.wheels_straight()

    def turn_right(self) -> None:
        self.log("Action: turn right", event="turn_right")
        self.motion.forward_right(
            speed=self.config.turn_speed,
            angle=self.config.steer_angle,
        )
        self.last_action = "turn_right"
        self._drive_for(self.config.turn_time_s, check_front=True)
        self.stop()
        self.motion.wheels_straight()

    def drive_forward(self) -> None:
        self.motion.forward(self.config.forward_speed, straighten=True)
        self.last_action = "forward"

    # ------------------------------------------------------------------
    # Scanning and decision
    # ------------------------------------------------------------------

    def scan(self) -> Dict[str, Optional[float]]:
        """Scan left, center and right with the mast-mounted ultrasonic sensor."""
        cfg = self.config
        self.log("Scanning: left / center / right", event="scan_started")

        self.check_external_stop()
        self.motion.mast_left(cfg.scan_angle)
        time.sleep(cfg.scan_settle_s)
        left = self.read_distance()

        self.check_external_stop()
        self.motion.mast_center()
        time.sleep(cfg.scan_settle_s)
        center = self.read_distance()

        self.check_external_stop()
        self.motion.mast_right(cfg.scan_angle)
        time.sleep(cfg.scan_settle_s)
        right = self.read_distance()

        self.motion.mast_center()

        self.last_scan = {
            "left": left,
            "center": center,
            "right": right,
        }

        self.log(f"Scan result: left={left}, center={center}, right={right}", event="scan_result", left_cm=left, center_cm=center, right_cm=right)
        return self.last_scan

    def choose_direction(self, scan: Dict[str, Optional[float]]) -> str:
        """Choose side with more free space."""
        left_score = self._distance_score(scan.get("left"))
        right_score = self._distance_score(scan.get("right"))

        if left_score > right_score:
            return "left"
        if right_score > left_score:
            return "right"

        if self.config.default_turn_direction in ("left", "right"):
            return self.config.default_turn_direction
        return "right"

    # ------------------------------------------------------------------
    # Behavior steps
    # ------------------------------------------------------------------

    def avoid(self, current_distance: Optional[float]) -> None:
        """Perform one full avoidance manoeuvre."""
        self.log(f"Obstacle detected: distance={current_distance} cm", level="warning", event="obstacle_detected", distance_cm=current_distance)
        self.stop()

        if self.is_dangerous(current_distance):
            self.reverse_briefly()

        scan = self.scan()
        direction = self.choose_direction(scan)
        self.log(f"Chosen direction: {direction}", event="direction_chosen", direction=direction)

        # If center is still very close after scanning, make a bit more space.
        if self.is_dangerous(scan.get("center")):
            self.reverse_briefly()

        if direction == "left":
            self.turn_left()
        else:
            self.turn_right()

    def step_once(self) -> None:
        """One sense-decide-act cycle."""
        self.motion.mast_center()
        distance = self.read_distance()

        if self.is_close(distance):
            self.avoid(distance)
            return

        if distance is None:
            if self.config.no_echo_means_clear:
                self.log("Path clear: no echo -> forward", event="path_clear_no_echo")
            else:
                self.log("No valid distance reading -> stop", level="warning", event="no_valid_distance")
                self.stop()
                return
        else:
            self.log(f"Path clear: distance={distance} cm -> forward", event="path_clear", distance_cm=distance)

        self.drive_forward()

    def run(self, run_seconds: Optional[float] = None) -> None:
        """Run until timeout or Ctrl+C."""
        if run_seconds is None:
            run_seconds = self.config.run_seconds

        start = time.time()

        while True:
            try:
                self.check_external_stop()

                if run_seconds is not None and time.time() - start >= run_seconds:
                    self.log("Run time finished", event="run_finished")
                    break

                self.step_once()
                time.sleep(self.config.loop_delay_s)
            except MissionStopRequested:
                self.log("Obstacle avoidance stopped by simulator UI", level="warning", event="mission_stopped_by_ui")
                break

    def shutdown(self) -> None:
        """Return rover to a safe stopped state."""
        try:
            self.stop(pause=False)
            self.motion.wheels_straight()
            self.motion.mast_center()
        except Exception as exc:
            self.log(f"AvoidObstacle shutdown failed: {exc}", level="warning", event="shutdown_failed", error=str(exc))


# Backward-compatible aliases in case some older mission code already imports them.
ObstacleAvoidanceConfig = AvoidObstacleConfig
ObstacleAvoidanceController = AvoidObstacleBehavior
