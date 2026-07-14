"""
Marsy motion abstraction.

Project location:
    marsy_core/motion.py

This layer exposes logical movement commands for Marsy and hides hardware
calibration details such as a reversed mast servo on the real rover.

Important convention:
    - Logical mast angle < 0 means sensor looks LEFT.
    - Logical mast angle > 0 means sensor looks RIGHT.

The real rover currently has the mast servo mounted in the opposite direction,
so by default the mast is reversed when MARSY_MODE=real.
Override with:
    MARSY_MAST_REVERSED=1   force reversed mast
    MARSY_MAST_REVERSED=0   force normal mast
"""

from __future__ import annotations

import os
import time


# Servo channels used by the original 4tronix library.
SERVO_MAST = 0
SERVO_FL = 9
SERVO_RL = 11
SERVO_RR = 13
SERVO_FR = 15


def clamp_speed(speed: int | float) -> int:
    """Clamp rover speed to the 0..100 range used by the 4tronix API."""
    return int(max(0, min(100, speed)))


def clamp_angle(angle: int | float) -> int:
    """Clamp logical servo angle to the conservative -90..90 range."""
    return int(max(-90, min(90, angle)))


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on", "y"}


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _default_mast_reversed() -> bool:
    """
    Default mast calibration.

    Our real Marsy mast is physically reversed. The simulator is not.
    Explicit MARSY_MAST_REVERSED always wins.
    """
    if os.getenv("MARSY_MAST_REVERSED") is not None:
        return _env_bool("MARSY_MAST_REVERSED", default=False)

    return os.getenv("MARSY_MODE", "sim").strip().lower() == "real"


class MarsyMotion:
    """
    High-level logical motion API.

    Mission/behavior code should use this class instead of calling rover.py
    directly. This lets the same behavior work in simulation and on the real
    rover even when hardware calibration differs.
    """

    def __init__(self, rover):
        self.rover = rover

        # Mast calibration.
        # Logical left/right remains stable; only the servo command is inverted.
        self.mast_reversed = _default_mast_reversed()
        self.mast_center_offset_deg = _env_float("MARSY_MAST_CENTER_OFFSET_DEG", 0.0)
        self.mast_min_deg = _env_float("MARSY_MAST_MIN_DEG", -90.0)
        self.mast_max_deg = _env_float("MARSY_MAST_MAX_DEG", 90.0)

        # Rotation steering pose. Before a differential spin, the four
        # steerable corner wheels are placed in an X pattern so their rolling
        # directions are approximately tangent to a circle around the rover
        # centre. This reduces lateral scrubbing compared with spinning while
        # all wheels point straight ahead.
        self.rotate_steer_angle_deg = _env_float("MARSY_ROTATE_STEER_ANGLE_DEG", 42.0)
        self.steering_settle_s = max(0.0, _env_float("MARSY_STEERING_SETTLE_S", 0.22))

    # ------------------------------------------------------------------
    # Generic helpers
    # ------------------------------------------------------------------

    def _set_servo(self, servo: int, angle: int | float) -> None:
        self.rover.setServo(servo, int(round(angle)))

    def _mast_servo_angle(self, logical_angle: int | float) -> int:
        """
        Convert logical mast angle to physical servo command.

        logical_angle:
            - negative -> sensor looks LEFT
            - positive -> sensor looks RIGHT

        If the mast servo is reversed, invert the command here only.
        """
        logical_angle = clamp_angle(logical_angle)

        if self.mast_reversed:
            logical_angle = -logical_angle

        physical_angle = self.mast_center_offset_deg + logical_angle
        physical_angle = max(self.mast_min_deg, min(self.mast_max_deg, physical_angle))
        return int(round(physical_angle))

    def mast_debug_config(self) -> dict:
        """Return current mast calibration for debug output."""
        return {
            "mast_reversed": self.mast_reversed,
            "mast_center_offset_deg": self.mast_center_offset_deg,
            "mast_min_deg": self.mast_min_deg,
            "mast_max_deg": self.mast_max_deg,
        }

    # ------------------------------------------------------------------
    # Wheel steering servos
    # ------------------------------------------------------------------

    def set_wheel_servos(self, fl: int, fr: int, rl: int, rr: int) -> None:
        fl = clamp_angle(fl)
        fr = clamp_angle(fr)
        rl = clamp_angle(rl)
        rr = clamp_angle(rr)

        # Simulator backend supports batch servo updates. The real 4tronix API
        # may not, so fall back to individual setServo calls.
        if hasattr(self.rover, "setServos"):
            self.rover.setServos(
                {
                    SERVO_FL: fl,
                    SERVO_FR: fr,
                    SERVO_RL: rl,
                    SERVO_RR: rr,
                }
            )
            return

        self._set_servo(SERVO_FL, fl)
        self._set_servo(SERVO_FR, fr)
        self._set_servo(SERVO_RL, rl)
        self._set_servo(SERVO_RR, rr)

    def wheels_straight(self) -> None:
        self.set_wheel_servos(0, 0, 0, 0)

    def steer_left(self, angle: int = 20) -> None:
        angle = clamp_angle(angle)
        self.set_wheel_servos(fl=-angle, fr=-angle, rl=angle, rr=angle)

    def steer_right(self, angle: int = 20) -> None:
        angle = clamp_angle(angle)
        self.set_wheel_servos(fl=angle, fr=angle, rl=-angle, rr=-angle)

    def wheels_for_rotation(self, angle: int | float | None = None) -> None:
        """Place the corner wheels in an X pose for an in-place rotation.

        The middle wheel pair cannot steer, so a real Marsy rotation still
        contains some skid. The X pose substantially reduces tyre scrubbing
        and servo load compared with spinning with all four steering wheels
        straight.
        """
        requested = self.rotate_steer_angle_deg if angle is None else float(angle)
        rotation_angle = abs(clamp_angle(requested))
        self.set_wheel_servos(
            fl=rotation_angle,
            fr=-rotation_angle,
            rl=-rotation_angle,
            rr=rotation_angle,
        )

    def _prepare_rotation(self, steering_angle: int | float | None = None) -> None:
        self.wheels_for_rotation(steering_angle)
        if self.steering_settle_s > 0:
            time.sleep(self.steering_settle_s)

    # ------------------------------------------------------------------
    # Drive motors
    # ------------------------------------------------------------------

    def forward(self, speed: int = 60, straighten: bool = True) -> None:
        speed = clamp_speed(speed)
        if straighten:
            self.wheels_straight()
        self.rover.forward(speed)

    def reverse(self, speed: int = 60, straighten: bool = True) -> None:
        speed = clamp_speed(speed)
        if straighten:
            self.wheels_straight()
        self.rover.reverse(speed)

    def stop(self) -> None:
        self.rover.stop()

    def brake(self) -> None:
        self.rover.brake()

    def spin_left(self, speed: int = 50, steering_angle: int | float | None = None) -> None:
        self._prepare_rotation(steering_angle)
        self.rover.spinLeft(clamp_speed(speed))

    def spin_right(self, speed: int = 50, steering_angle: int | float | None = None) -> None:
        self._prepare_rotation(steering_angle)
        self.rover.spinRight(clamp_speed(speed))

    def turn_forward(self, left_speed: int, right_speed: int) -> None:
        self.rover.turnForward(clamp_speed(left_speed), clamp_speed(right_speed))

    def turn_reverse(self, left_speed: int, right_speed: int) -> None:
        self.rover.turnReverse(clamp_speed(left_speed), clamp_speed(right_speed))

    def forward_left(self, speed: int = 50, angle: int = 20) -> None:
        self.steer_left(angle)
        self.rover.forward(clamp_speed(speed))

    def forward_right(self, speed: int = 50, angle: int = 20) -> None:
        self.steer_right(angle)
        self.rover.forward(clamp_speed(speed))

    def reverse_left(self, speed: int = 50, angle: int = 20) -> None:
        self.steer_left(angle)
        self.rover.reverse(clamp_speed(speed))

    def reverse_right(self, speed: int = 50, angle: int = 20) -> None:
        self.steer_right(angle)
        self.rover.reverse(clamp_speed(speed))

    # ------------------------------------------------------------------
    # Mast-mounted ultrasonic sensor
    # ------------------------------------------------------------------

    def mast_center(self) -> None:
        # Logical center is always 0; calibration offset is applied internally.
        self.mast_to(0)

    def mast_left(self, angle: int = 45) -> None:
        # Logical left is negative. Hardware inversion is handled in mast_to().
        self.mast_to(-abs(clamp_angle(angle)))

    def mast_right(self, angle: int = 45) -> None:
        # Logical right is positive. Hardware inversion is handled in mast_to().
        self.mast_to(abs(clamp_angle(angle)))

    def mast_to(self, logical_angle: int | float) -> None:
        servo_angle = self._mast_servo_angle(logical_angle)
        self._set_servo(SERVO_MAST, servo_angle)

    # ------------------------------------------------------------------
    # Safe convenience poses
    # ------------------------------------------------------------------

    def reset_pose(self) -> None:
        self.wheels_straight()
        self.mast_center()

    def stop_and_reset(self) -> None:
        self.stop()
        self.reset_pose()

    def brake_and_reset(self) -> None:
        self.brake()
        self.reset_pose()
