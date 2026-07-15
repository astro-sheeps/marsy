"""High-level, bounded, LLM-callable skills for Marsy."""

from __future__ import annotations

import math
import os
import select
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

from marsy_core.motion import MarsyMotion, clamp_speed
from marsy_core.sensors import MarsySensors
from marsy_core.telemetry import log, simulator_stop_requested

from .mapping import SparseOccupancyGrid
from .models import Detection, Pose2D, RangeSample, SkillResult, normalize_heading
from .registry import get_skill_names, get_skill_schemas
from .vision import CapabilityUnavailable, VisionSystem

SOURCE = "skills"


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)))
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)))
    except ValueError:
        return default


@dataclass
class SkillsConfig:
    full_speed_cm_s: float = _env_float("MARSY_FULL_SPEED_CM_S", 9.0)
    full_spin_deg_s: float = _env_float("MARSY_FULL_SPIN_DEG_S", 36.0)
    wheelbase_cm: float = _env_float("MARSY_WHEELBASE_CM", 16.0)
    safe_distance_cm: float = _env_float("MARSY_SKILLS_SAFE_DISTANCE_CM", 35.0)
    danger_distance_cm: float = _env_float("MARSY_SKILLS_DANGER_DISTANCE_CM", 22.0)
    max_motion_s: float = _env_float("MARSY_SKILLS_MAX_MOTION_S", 5.0)
    motion_slice_s: float = _env_float("MARSY_SKILLS_MOTION_SLICE_S", 0.12)
    scan_settle_s: float = _env_float("MARSY_SKILLS_SCAN_SETTLE_S", 0.35)
    max_range_cm: float = _env_float("MARSY_SKILLS_MAX_RANGE_CM", 200.0)
    distance_spread_abs_cm: float = _env_float("MARSY_DISTANCE_SPREAD_ABS_CM", 12.0)
    distance_spread_ratio: float = _env_float("MARSY_DISTANCE_SPREAD_RATIO", 0.25)
    suspicious_far_cm: float = _env_float("MARSY_SUSPICIOUS_FAR_CM", 140.0)
    suspicious_jump_cm: float = _env_float("MARSY_SUSPICIOUS_JUMP_CM", 50.0)
    suspicious_ratio: float = _env_float("MARSY_SUSPICIOUS_RATIO", 1.5)
    recent_scan_max_age_s: float = _env_float("MARSY_RECENT_SCAN_MAX_AGE_S", 8.0)
    front_unknown_limit: int = _env_int("MARSY_FRONT_UNKNOWN_LIMIT", 2)
    uncertain_step_cm: float = _env_float("MARSY_UNCERTAIN_STEP_CM", 6.0)
    uncertain_speed: int = _env_int("MARSY_UNCERTAIN_SPEED", 22)
    default_speed: int = 35
    default_turn_speed: int = 30
    default_steer_angle: int = 28


class MarsySkills:
    """
    Bounded skill API used by missions and future LLM planners.

    Motion commands stop automatically. Manual continuous driving stays in the
    separate manual_drive mission.
    """

    def __init__(
        self,
        rover: Any,
        *,
        motion: Optional[MarsyMotion] = None,
        sensors: Optional[MarsySensors] = None,
        vision: Optional[VisionSystem] = None,
        occupancy_map: Optional[SparseOccupancyGrid] = None,
        config: Optional[SkillsConfig] = None,
        capture_dir: str | Path = "artifacts/captures",
        stop_requested: Optional[Callable[[], bool]] = None,
    ) -> None:
        self.rover = rover
        self.motion = motion or MarsyMotion(rover)
        self.sensors = sensors or MarsySensors(rover)
        self.vision = vision or VisionSystem()
        self.config = config or SkillsConfig()
        self.map = occupancy_map or SparseOccupancyGrid(max_range_cm=self.config.max_range_cm)
        self.capture_dir = Path(capture_dir)
        self._stop_requested_callback = stop_requested
        self.pose = Pose2D()
        self._sync_pose()
        self.home_pose = self.pose.normalized()
        self.map.mark_pose(self.pose)
        self._last_scan_samples: list[RangeSample] = []
        self._last_scan_timestamp: float = 0.0
        self._last_motion_uncertain_checks: int = 0

    # ------------------------------------------------------------------
    # Registry / generic dispatch
    # ------------------------------------------------------------------

    @staticmethod
    def schemas() -> list[dict[str, Any]]:
        return get_skill_schemas()

    @staticmethod
    def names() -> list[str]:
        return get_skill_names()

    def call(self, skill_name: str, **arguments: Any) -> SkillResult:
        if skill_name not in self.names():
            return self._result(skill_name, False, "unknown_skill", f"Unknown skill: {skill_name}")
        method = getattr(self, skill_name)
        try:
            return method(**arguments)
        except TypeError as exc:
            return self._result(skill_name, False, "invalid_arguments", str(exc))
        except Exception as exc:
            self._safe_stop()
            return self._result(skill_name, False, "error", f"{type(exc).__name__}: {exc}")

    def _external_stop_requested(self) -> bool:
        try:
            callback_stop = bool(self._stop_requested_callback and self._stop_requested_callback())
        except Exception:
            callback_stop = True
        return callback_stop or simulator_stop_requested()

    def _result(self, skill: str, ok: bool, status: str, message: str = "", **data: Any) -> SkillResult:
        now = time.time()
        result = SkillResult(skill=skill, ok=ok, status=status, message=message, data=data, finished_at=now)
        level = "info" if ok else "warning"
        safe_fields = result.to_dict().get("data", {})
        log(
            message or f"{skill}: {status}",
            source=SOURCE,
            level=level,
            event=f"skill_{skill}",
            status=status,
            **safe_fields,
        )
        return result

    # ------------------------------------------------------------------
    # Pose and motion estimation
    # ------------------------------------------------------------------

    def _backend_pose(self) -> Optional[Pose2D]:
        getter = getattr(self.rover, "get_state", None)
        if not callable(getter):
            return None
        try:
            state = getter() or {}
            if not state.get("ready", True):
                return None
            if not all(key in state for key in ("x_cm", "y_cm", "heading_deg")):
                return None
            return Pose2D(
                x_cm=float(state["x_cm"]),
                y_cm=float(state["y_cm"]),
                heading_deg=float(state["heading_deg"]),
                source="simulator_state",
            ).normalized()
        except Exception:
            return None

    def sync_pose(self) -> Pose2D:
        """Refresh pose from the backend when available and return a copy."""
        return self._sync_pose().normalized()

    def _sync_pose(self) -> Pose2D:
        backend_pose = self._backend_pose()
        if backend_pose is not None:
            self.pose = backend_pose
        else:
            self.pose = self.pose.normalized()
            self.pose.timestamp = time.time()
        self.map.mark_pose(self.pose) if hasattr(self, "map") else None
        return self.pose

    def _linear_speed_cm_s(self, speed: int) -> float:
        return self.config.full_speed_cm_s * clamp_speed(speed) / 100.0

    def _spin_speed_deg_s(self, speed: int) -> float:
        return self.config.full_spin_deg_s * clamp_speed(speed) / 100.0

    def _integrate_linear(self, distance_cm: float, heading_change_deg: float = 0.0) -> None:
        if self._backend_pose() is not None:
            self._sync_pose()
            return
        midpoint_heading = self.pose.heading_deg + heading_change_deg / 2.0
        radians = math.radians(midpoint_heading)
        self.pose.x_cm += float(distance_cm) * math.sin(radians)
        self.pose.y_cm += float(distance_cm) * math.cos(radians)
        self.pose.heading_deg = normalize_heading(self.pose.heading_deg + heading_change_deg)
        self.pose.source = "dead_reckoning"
        self.pose.timestamp = time.time()
        self.map.mark_pose(self.pose)

    def _integrate_rotation(self, angle_deg: float) -> None:
        if self._backend_pose() is not None:
            self._sync_pose()
            return
        self.pose.heading_deg = normalize_heading(self.pose.heading_deg + angle_deg)
        self.pose.source = "dead_reckoning"
        self.pose.timestamp = time.time()
        self.map.mark_pose(self.pose)

    def _is_valid_distance(self, value: Optional[float]) -> bool:
        return value is not None and 0 < float(value) <= self.config.max_range_cm

    def _safe_stop(self) -> None:
        try:
            self.motion.stop()
        except Exception:
            pass

    def _collect_distance_measurement(self, samples: int = 3, sample_delay_s: float = 0.04) -> dict[str, Any]:
        requested = max(1, int(samples))
        raw_readings: list[Optional[float]] = []
        valid_readings: list[float] = []
        for _ in range(requested):
            if self._external_stop_requested():
                break
            try:
                value = self.sensors.distance_cm()
            except Exception:
                value = None
            numeric: Optional[float]
            try:
                numeric = None if value is None else float(value)
            except (TypeError, ValueError):
                numeric = None
            raw_readings.append(numeric)
            if self._is_valid_distance(numeric):
                valid_readings.append(float(numeric))
            time.sleep(max(0.0, float(sample_delay_s)))

        minimum_valid = max(1, requested // 2 + 1)
        if not valid_readings:
            return {
                "quality": "no_echo",
                "distance_cm": None,
                "readings": raw_readings,
                "valid_readings": [],
                "spread_cm": None,
                "minimum_valid_cm": None,
            }
        if len(valid_readings) < minimum_valid:
            return {
                "quality": "insufficient",
                "distance_cm": round(float(statistics.median(valid_readings)), 1),
                "readings": raw_readings,
                "valid_readings": valid_readings,
                "spread_cm": None,
                "minimum_valid_cm": round(min(valid_readings), 1),
            }

        distance = float(statistics.median(valid_readings))
        spread = max(valid_readings) - min(valid_readings) if len(valid_readings) > 1 else 0.0
        allowed_spread = max(self.config.distance_spread_abs_cm, distance * self.config.distance_spread_ratio)
        quality = "measured" if spread <= allowed_spread else "unstable"
        return {
            "quality": quality,
            "distance_cm": round(distance, 1),
            "readings": raw_readings,
            "valid_readings": valid_readings,
            "spread_cm": round(spread, 1),
            "minimum_valid_cm": round(min(valid_readings), 1),
        }

    def _run_for(self, duration_s: float, *, check_front: bool) -> tuple[float, Optional[float], str]:
        requested_duration = max(0.0, float(duration_s))
        duration = min(requested_duration, self.config.max_motion_s)
        started = time.time()
        blocked_distance: Optional[float] = None
        reason = "bounded" if requested_duration > self.config.max_motion_s else "completed"
        uncertain_checks = 0
        try:
            while time.time() - started < duration:
                if self._external_stop_requested():
                    reason = "external_stop"
                    break
                remaining = duration - (time.time() - started)
                time.sleep(max(0.0, min(self.config.motion_slice_s, remaining)))
                if check_front:
                    measurement = self._collect_distance_measurement(samples=3, sample_delay_s=0.01)
                    distance = measurement.get("distance_cm")
                    minimum_valid = measurement.get("minimum_valid_cm")

                    # Unreliable/no-echo measurements are retained as uncertainty,
                    # not converted into an obstacle. A real close echo still has
                    # absolute priority and stops the rover immediately.
                    if self._is_valid_distance(minimum_valid) and float(minimum_valid) < self.config.danger_distance_cm:
                        blocked_distance = float(minimum_valid)
                        reason = "blocked"
                        break

                    if measurement.get("quality") != "measured":
                        uncertain_checks += 1
                        continue

                    if self._is_valid_distance(distance) and float(distance) < self.config.safe_distance_cm:
                        blocked_distance = float(distance)
                        reason = "blocked"
                        break
        finally:
            self.motion.stop()
            self._last_motion_uncertain_checks = uncertain_checks
        return max(0.0, time.time() - started), blocked_distance, reason

    # ------------------------------------------------------------------
    # Basic motion skills
    # ------------------------------------------------------------------

    def move_forward(
        self,
        distance_cm: float = 10.0,
        speed: int = 35,
        require_recent_scan: bool = False,
    ) -> SkillResult:
        skill = "move_forward"
        speed = clamp_speed(speed)
        if speed <= 0:
            return self._result(skill, False, "invalid_speed", "Forward speed must be above zero")

        cautious = False
        corridor: Optional[dict[str, Any]] = None
        if require_recent_scan:
            corridor = self.assess_forward_corridor()
            if not corridor["safe"]:
                return self._result(
                    skill,
                    False,
                    corridor["status"],
                    corridor["message"],
                    corridor=corridor,
                    pose=self.pose,
                )
            cautious = corridor.get("status") == "uncertain_clear"

        requested = max(0.0, float(distance_cm))
        self.motion.mast_center()
        time.sleep(self.config.scan_settle_s)
        measurement = self._collect_distance_measurement(samples=3, sample_delay_s=0.02)
        current = measurement.get("distance_cm")
        minimum_valid = measurement.get("minimum_valid_cm")

        # Any real echo inside the danger zone is a hard stop, even if the
        # overall series is unstable. Otherwise an unreliable series is allowed
        # and simply switches the command to a short, slow cautious creep.
        if self._is_valid_distance(minimum_valid) and float(minimum_valid) < self.config.danger_distance_cm:
            return self._result(
                skill,
                False,
                "blocked",
                f"Possible close obstacle at {float(minimum_valid):.1f} cm",
                distance_cm=minimum_valid,
                measurement=measurement,
            )
        if measurement.get("quality") == "measured":
            if self._is_valid_distance(current) and float(current) < self.config.safe_distance_cm:
                return self._result(
                    skill,
                    False,
                    "blocked",
                    f"Obstacle at {float(current):.1f} cm",
                    distance_cm=current,
                )
        else:
            cautious = True

        commanded_distance = requested
        commanded_speed = speed
        if cautious:
            commanded_distance = min(requested, max(1.0, float(self.config.uncertain_step_cm)))
            commanded_speed = max(1, min(speed, clamp_speed(self.config.uncertain_speed)))

        duration = commanded_distance / max(0.001, self._linear_speed_cm_s(commanded_speed))
        self.motion.forward(commanded_speed, straighten=True)
        elapsed, blocked, reason = self._run_for(duration, check_front=True)
        travelled = self._linear_speed_cm_s(commanded_speed) * elapsed
        self._integrate_linear(travelled)

        common_data = {
            "travelled_cm": travelled,
            "requested_distance_cm": requested,
            "commanded_distance_cm": commanded_distance,
            "commanded_speed": commanded_speed,
            "cautious": cautious,
            "uncertain_checks": self._last_motion_uncertain_checks,
            "pose": self.pose,
        }
        if corridor is not None:
            common_data["corridor"] = corridor

        if reason == "blocked":
            return self._result(
                skill,
                False,
                "blocked",
                f"Stopped for obstacle at {blocked:.1f} cm",
                obstacle_cm=blocked,
                **common_data,
            )
        if reason == "external_stop":
            return self._result(
                skill,
                False,
                "stopped",
                "External stop requested",
                **common_data,
            )

        status = "cautious" if cautious or self._last_motion_uncertain_checks else (
            "completed" if duration <= self.config.max_motion_s else "bounded"
        )
        message = (
            f"Cautious forward creep about {travelled:.1f} cm"
            if status == "cautious"
            else f"Moved forward about {travelled:.1f} cm"
        )
        return self._result(skill, True, status, message, **common_data)

    def move_backward(self, distance_cm: float = 8.0, speed: int = 25) -> SkillResult:
        skill = "move_backward"
        speed = clamp_speed(speed)
        if speed <= 0:
            return self._result(skill, False, "invalid_speed", "Reverse speed must be above zero")
        requested = max(0.0, float(distance_cm))
        duration = requested / max(0.001, self._linear_speed_cm_s(speed))
        self.motion.reverse(speed, straighten=True)
        elapsed, _, reason = self._run_for(duration, check_front=False)
        travelled = self._linear_speed_cm_s(speed) * elapsed
        self._integrate_linear(-travelled)
        ok = reason in {"completed", "bounded"}
        return self._result(skill, ok, reason, f"Moved backward about {travelled:.1f} cm", travelled_cm=travelled, pose=self.pose)

    def _turn_arc(self, direction: str, angle_deg: float, speed: int, steer_angle: float) -> SkillResult:
        skill = f"turn_{direction}"
        speed = clamp_speed(speed)
        steer = max(1.0, min(60.0, abs(float(steer_angle))))
        requested_angle = max(0.0, abs(float(angle_deg)))
        linear_speed = self._linear_speed_cm_s(speed)
        angular_rate_rad_s = linear_speed / max(1.0, self.config.wheelbase_cm) * math.tan(math.radians(steer))
        angular_rate_deg_s = max(0.1, math.degrees(abs(angular_rate_rad_s)))
        duration = requested_angle / angular_rate_deg_s
        self.motion.mast_center()
        time.sleep(self.config.scan_settle_s)
        if direction == "left":
            self.motion.forward_left(speed=speed, angle=int(round(steer)))
            signed_angle = -1.0
        else:
            self.motion.forward_right(speed=speed, angle=int(round(steer)))
            signed_angle = 1.0
        elapsed, blocked, reason = self._run_for(duration, check_front=True)
        self.motion.wheels_straight()
        actual_angle = signed_angle * angular_rate_deg_s * elapsed
        distance = linear_speed * elapsed
        self._integrate_linear(distance, heading_change_deg=actual_angle)
        ok = reason in {"completed", "bounded"}
        message = f"Turned {direction} about {abs(actual_angle):.1f}°"
        if reason == "blocked":
            message += f"; obstacle at {blocked:.1f} cm"
        return self._result(skill, ok, reason, message, angle_deg=actual_angle, travelled_cm=distance, obstacle_cm=blocked, pose=self.pose)

    def turn_left(self, angle_deg: float = 35.0, speed: int = 30, steer_angle: float = 28.0) -> SkillResult:
        return self._turn_arc("left", angle_deg, speed, steer_angle)

    def turn_right(self, angle_deg: float = 35.0, speed: int = 30, steer_angle: float = 28.0) -> SkillResult:
        return self._turn_arc("right", angle_deg, speed, steer_angle)

    def rotate(self, angle_deg: float, speed: int = 30) -> SkillResult:
        skill = "rotate"
        angle = float(angle_deg)
        speed = clamp_speed(speed)
        if abs(angle) < 0.1:
            return self._result(skill, True, "completed", "Rotation not needed", angle_deg=0.0, pose=self.pose)
        angular_speed = self._spin_speed_deg_s(speed)
        if angular_speed <= 0:
            return self._result(skill, False, "invalid_speed", "Rotation speed must be above zero")
        requested_duration = abs(angle) / angular_speed
        try:
            if angle > 0:
                self.motion.spin_right(speed)
            else:
                self.motion.spin_left(speed)
            elapsed, _, reason = self._run_for(requested_duration, check_front=False)
        finally:
            # spin_left/spin_right place the corner wheels in an X rotation
            # pose. Always restore the neutral steering pose afterwards.
            self.motion.wheels_straight()
        actual_angle = math.copysign(angular_speed * elapsed, angle)
        self._integrate_rotation(actual_angle)
        ok = reason in {"completed", "bounded"}
        return self._result(skill, ok, reason, f"Rotated about {actual_angle:.1f}°", angle_deg=actual_angle, pose=self.pose)

    def stop(self, brake: bool = False, reset_pose: bool = True) -> SkillResult:
        if brake:
            self.motion.brake()
        else:
            self.motion.stop()
        if reset_pose:
            self.motion.reset_pose()
        return self._result("stop", True, "completed", "Marsy stopped", brake=brake, reset_pose=reset_pose)

    # ------------------------------------------------------------------
    # Range sensing and mapping
    # ------------------------------------------------------------------

    def measure_distance(self, samples: int = 3, sample_delay_s: float = 0.04) -> SkillResult:
        measurement = self._collect_distance_measurement(samples=samples, sample_delay_s=sample_delay_s)
        quality = str(measurement["quality"])
        distance = measurement.get("distance_cm")
        if quality == "measured" and distance is not None:
            return self._result(
                "measure_distance",
                True,
                "measured",
                f"Distance {float(distance):.1f} cm",
                **measurement,
            )
        return self._result(
            "measure_distance",
            True,
            "unknown",
            f"Ultrasonic range is {quality}",
            **measurement,
        )

    def _mark_suspicious_far_samples(self, samples: list[RangeSample]) -> None:
        ordered = sorted(samples, key=lambda item: item.mast_angle_deg)
        for index, sample in enumerate(ordered):
            if not sample.trusted or sample.distance_cm is None:
                continue
            distance = float(sample.distance_cm)
            if distance < self.config.suspicious_far_cm:
                continue
            neighbours: list[float] = []
            for neighbour_index in (index - 1, index + 1):
                if 0 <= neighbour_index < len(ordered):
                    neighbour = ordered[neighbour_index]
                    if neighbour.trusted and neighbour.distance_cm is not None:
                        neighbours.append(float(neighbour.distance_cm))
            if not neighbours:
                continue
            neighbour_median = float(statistics.median(neighbours))
            if (
                distance - neighbour_median >= self.config.suspicious_jump_cm
                and distance >= neighbour_median * self.config.suspicious_ratio
            ):
                sample.quality = "suspicious_far"
                sample.valid_hit = False
                sample.no_echo = False

    def assess_forward_corridor(
        self,
        samples: Optional[list[RangeSample]] = None,
        *,
        max_age_s: Optional[float] = None,
    ) -> dict[str, Any]:
        selected = list(self._last_scan_samples if samples is None else samples)
        age_limit = self.config.recent_scan_max_age_s if max_age_s is None else float(max_age_s)
        if samples is None and (not selected or time.time() - self._last_scan_timestamp > age_limit):
            return {
                "safe": False,
                "status": "stale_scan",
                "message": "A fresh full scan is required before moving",
                "unknown_angles": [-25.0, 0.0, 25.0],
                "minimum_distance_cm": None,
                "minimum_observed_cm": None,
                "degraded": False,
            }

        front: list[RangeSample] = []
        missing: list[float] = []
        for target in (-25.0, 0.0, 25.0):
            candidates = [item for item in selected if abs(float(item.mast_angle_deg) - target) <= 8.0]
            if not candidates:
                missing.append(target)
                continue
            front.append(min(candidates, key=lambda item: abs(float(item.mast_angle_deg) - target)))

        unknown = [float(item.mast_angle_deg) for item in front if not item.trusted] + missing
        reliable_distances = [
            float(item.distance_cm)
            for item in front
            if item.trusted and item.distance_cm is not None
        ]
        observed_distances: list[float] = []
        for item in front:
            for value in getattr(item, "readings", []) or []:
                try:
                    numeric = float(value)
                except (TypeError, ValueError):
                    continue
                if self._is_valid_distance(numeric):
                    observed_distances.append(numeric)
            if item.distance_cm is not None and self._is_valid_distance(item.distance_cm):
                observed_distances.append(float(item.distance_cm))

        minimum = min(reliable_distances) if reliable_distances else None
        minimum_observed = min(observed_distances) if observed_distances else None

        # A close real echo always blocks, regardless of the quality label.
        if minimum_observed is not None and minimum_observed < self.config.danger_distance_cm:
            return {
                "safe": False,
                "status": "blocked",
                "message": f"Front corridor has a close echo at {minimum_observed:.1f} cm",
                "unknown_angles": sorted(unknown),
                "minimum_distance_cm": minimum,
                "minimum_observed_cm": minimum_observed,
                "degraded": bool(unknown),
            }

        # A trusted obstacle inside the normal safety distance also blocks.
        if minimum is not None and minimum < self.config.safe_distance_cm:
            return {
                "safe": False,
                "status": "blocked",
                "message": f"Front corridor is blocked at {minimum:.1f} cm",
                "unknown_angles": sorted(unknown),
                "minimum_distance_cm": minimum,
                "minimum_observed_cm": minimum_observed,
                "degraded": bool(unknown),
            }

        # Unknown/no-echo/unstable sectors are no longer treated as obstacles.
        # They permit only a short, slow creep; the motion loop still stops on
        # any close echo that appears while the rover is moving.
        if unknown:
            return {
                "safe": True,
                "status": "uncertain_clear",
                "message": "Front corridor is uncertain; cautious forward creep allowed",
                "unknown_angles": sorted(unknown),
                "minimum_distance_cm": minimum,
                "minimum_observed_cm": minimum_observed,
                "degraded": True,
            }

        if minimum is None:
            return {
                "safe": True,
                "status": "uncertain_clear",
                "message": "No reliable front echo; cautious forward creep allowed",
                "unknown_angles": [-25.0, 0.0, 25.0],
                "minimum_distance_cm": None,
                "minimum_observed_cm": minimum_observed,
                "degraded": True,
            }

        return {
            "safe": True,
            "status": "clear",
            "message": f"Front corridor clear; minimum {minimum:.1f} cm",
            "unknown_angles": [],
            "minimum_distance_cm": minimum,
            "minimum_observed_cm": minimum_observed,
            "degraded": False,
        }

    def scan_arc(
        self,
        start_deg: float = -60.0,
        end_deg: float = 60.0,
        step_deg: float = 30.0,
        samples: int = 3,
        angles: Optional[list[float]] = None,
        on_sample: Optional[Callable[[RangeSample, list[RangeSample]], None]] = None,
    ) -> SkillResult:
        skill = "scan_arc"
        if angles is None:
            step = abs(float(step_deg))
            if step <= 0:
                return self._result(skill, False, "invalid_step", "step_deg must be positive")
            start = float(start_deg)
            end = float(end_deg)
            direction = 1.0 if end >= start else -1.0
            angles = []
            value = start
            while (direction > 0 and value <= end + 1e-6) or (direction < 0 and value >= end - 1e-6):
                angles.append(round(value, 4))
                value += direction * step

        range_samples: list[RangeSample] = []
        try:
            for angle in angles:
                if self._external_stop_requested():
                    break
                logical_angle = max(-90.0, min(90.0, float(angle)))
                self.motion.mast_to(logical_angle)
                time.sleep(self.config.scan_settle_s)
                measurement = self._collect_distance_measurement(samples=samples, sample_delay_s=0.04)
                distance = measurement.get("distance_cm")
                quality = str(measurement.get("quality", "unknown"))
                self._sync_pose()
                valid_hit = quality == "measured" and self._is_valid_distance(distance)
                sample = RangeSample(
                    mast_angle_deg=logical_angle,
                    global_angle_deg=normalize_heading(self.pose.heading_deg + logical_angle),
                    distance_cm=None if distance is None else float(distance),
                    valid_hit=valid_hit,
                    no_echo=quality == "no_echo",
                    pose=self.pose.normalized(),
                    quality=quality,
                    readings=[float(value) for value in measurement.get("valid_readings", [])],
                    spread_cm=measurement.get("spread_cm"),
                )
                range_samples.append(sample)
                if on_sample is not None:
                    try:
                        on_sample(sample, list(range_samples))
                    except Exception as exc:
                        log(
                            f"scan_arc telemetry callback failed: {type(exc).__name__}: {exc}",
                            source=SOURCE,
                            level="warning",
                        )

            self._mark_suspicious_far_samples(range_samples)
            for sample in range_samples:
                # Unknown/no-echo/unstable rays must not create false free space.
                if not sample.trusted:
                    continue
                self.map.observe_ray(
                    sample.pose,
                    sample.mast_angle_deg,
                    sample.distance_cm,
                    valid_hit=True,
                )
            self._last_scan_samples = list(range_samples)
            self._last_scan_timestamp = time.time()
        finally:
            self.motion.mast_center()
        trusted = sum(1 for sample in range_samples if sample.trusted)
        unknown = len(range_samples) - trusted
        return self._result(
            skill,
            True,
            "completed",
            f"Collected {len(range_samples)} range samples ({trusted} trusted, {unknown} unknown)",
            samples=range_samples,
            trusted_count=trusted,
            unknown_count=unknown,
            pose=self.pose,
        )

    def scan_360(self, step_deg: float = 45.0, rotate_speed: int = 30) -> SkillResult:
        skill = "scan_360"
        step = max(15.0, min(180.0, abs(float(step_deg))))
        sectors = max(2, int(round(360.0 / step)))
        actual_step = 360.0 / sectors
        all_samples: list[RangeSample] = []
        for _ in range(sectors):
            if self._external_stop_requested():
                return self._result(skill, False, "stopped", "360° scan interrupted", samples=all_samples, pose=self.pose)
            scan = self.scan_arc(angles=[-45.0, 0.0, 45.0], samples=2)
            all_samples.extend(scan.data.get("samples", []))
            rotation = self.rotate(actual_step, speed=rotate_speed)
            if not rotation.ok:
                return self._result(skill, False, "stopped", "360° scan interrupted", samples=all_samples, pose=self.pose)
        return self._result(skill, True, "completed", f"Completed 360° scan with {len(all_samples)} samples", samples=all_samples, pose=self.pose)

    def avoid_obstacle(self) -> SkillResult:
        from behaviors.avoid_obstacle import AvoidObstacleBehavior, AvoidObstacleConfig

        skill = "avoid_obstacle"
        behavior = AvoidObstacleBehavior(
            self.motion,
            self.sensors,
            AvoidObstacleConfig(
                safe_distance_cm=self.config.safe_distance_cm,
                danger_distance_cm=self.config.danger_distance_cm,
                max_valid_distance_cm=self.config.max_range_cm,
                forward_speed=self.config.default_speed,
                turn_speed=self.config.default_turn_speed,
                reverse_speed=25,
                scan_angle=55,
                steer_angle=self.config.default_steer_angle,
                verbose=False,
            ),
        )
        distance = behavior.read_distance()
        if not behavior.is_close(distance):
            return self._result(skill, True, "clear", "No nearby obstacle requires avoidance", distance_cm=distance)
        self.stop(brake=False, reset_pose=False)
        if behavior.is_dangerous(distance):
            self.move_backward(distance_cm=8.0, speed=25)
        scan = behavior.scan()
        self._sync_pose()
        mapped_samples = []
        for label, angle in (("left", -55.0), ("center", 0.0), ("right", 55.0)):
            value = scan.get(label)
            valid = self._is_valid_distance(value)
            sample = RangeSample(angle, normalize_heading(self.pose.heading_deg + angle), value, valid, not valid, self.pose.normalized())
            mapped_samples.append(sample)
            self.map.observe_ray(self.pose, angle, value, valid_hit=valid)
        direction = behavior.choose_direction(scan)
        if behavior.is_dangerous(scan.get("center")):
            self.move_backward(distance_cm=6.0, speed=25)
        turn = self.turn_left() if direction == "left" else self.turn_right()
        return self._result(skill, turn.ok, "completed" if turn.ok else turn.status, f"Avoided obstacle by turning {direction}", direction=direction, scan=scan, pose=self.pose)

    # ------------------------------------------------------------------
    # Camera and visual skills
    # ------------------------------------------------------------------

    def capture_image(self, filename: Optional[str] = None) -> SkillResult:
        skill = "capture_image"
        if not filename:
            filename = time.strftime("marsy_%Y%m%d_%H%M%S.jpg")
        output = self.capture_dir / Path(filename).name
        try:
            path = self.vision.capture(output)
            return self._result(skill, True, "captured", f"Captured {path}", image_path=path)
        except CapabilityUnavailable as exc:
            return self._result(skill, False, "unavailable", str(exc))

    def _detect_current_view(self, query: str) -> tuple[Optional[Path], list[Detection], Optional[str]]:
        captured = self.capture_image()
        if not captured.ok:
            return None, [], captured.message
        path = Path(captured.data["image_path"])
        try:
            detections = self.vision.detect(path, query)
            return path, detections, None
        except CapabilityUnavailable as exc:
            return path, [], str(exc)

    def search_visual(self, query: str, step_deg: float = 30.0, max_rotation_deg: float = 360.0, min_confidence: float = 0.25) -> SkillResult:
        skill = "search_visual"
        step = max(5.0, min(90.0, abs(float(step_deg))))
        sectors = max(1, int(math.ceil(abs(float(max_rotation_deg)) / step)))
        images: list[str] = []
        rotated = 0.0
        for index in range(sectors + 1):
            image_path, detections, error = self._detect_current_view(query)
            if image_path is not None:
                images.append(str(image_path))
            if error:
                return self._result(skill, False, "unavailable", error, images=images)
            matches = [item for item in detections if item.confidence >= min_confidence]
            if matches:
                best = max(matches, key=lambda item: item.confidence)
                return self._result(skill, True, "found", f"Found {best.label}", detection=best, image_path=image_path, rotated_deg=rotated)
            if index < sectors:
                rotation = self.rotate(step, speed=25)
                rotated += float(rotation.data.get("angle_deg", 0.0))
                if not rotation.ok:
                    break
        return self._result(skill, False, "not_found", f"Did not find {query}", images=images, rotated_deg=rotated)

    def track_object(self, query: str, tolerance: float = 0.10, max_iterations: int = 8) -> SkillResult:
        skill = "track_object"
        for iteration in range(max(1, int(max_iterations))):
            image_path, detections, error = self._detect_current_view(query)
            if error:
                return self._result(skill, False, "unavailable", error)
            if not detections:
                return self._result(skill, False, "lost", f"Object {query} is not visible", iteration=iteration)
            best = max(detections, key=lambda item: item.confidence)
            center_x = best.center_x
            if center_x is None:
                return self._result(skill, False, "no_bbox", "Detector did not return a bounding box", detection=best)
            error_x = center_x - 0.5
            if abs(error_x) <= tolerance:
                return self._result(skill, True, "centered", f"Centered {best.label}", detection=best, image_path=image_path, horizontal_error=error_x)
            correction = max(-20.0, min(20.0, error_x * 60.0))
            rotation = self.rotate(correction, speed=22)
            if not rotation.ok:
                return self._result(skill, False, "stopped", "Tracking rotation was interrupted", detection=best)
        return self._result(skill, False, "not_centered", f"Could not center {query} within iteration limit")

    def approach_object(self, query: str, target_distance_cm: float = 35.0, step_distance_cm: float = 8.0, max_steps: int = 12) -> SkillResult:
        skill = "approach_object"
        for step in range(max(1, int(max_steps))):
            tracked = self.track_object(query)
            if not tracked.ok:
                return self._result(skill, False, tracked.status, tracked.message, step=step)
            distance_result = self.measure_distance(samples=3)
            distance = distance_result.data.get("distance_cm")
            if distance is None:
                return self._result(skill, False, "no_range", "Object is centered but sonar has no valid range", step=step)
            if float(distance) <= float(target_distance_cm):
                self.stop(reset_pose=False)
                return self._result(skill, True, "arrived", f"Reached {query} at {float(distance):.1f} cm", distance_cm=distance, step=step)
            movement = self.move_forward(distance_cm=min(step_distance_cm, max(1.0, float(distance) - target_distance_cm)), speed=25)
            if not movement.ok:
                return self._result(skill, False, movement.status, movement.message, distance_cm=distance, step=step)
        return self._result(skill, False, "step_limit", f"Did not reach {query} within step limit")

    def find_marker(self, marker_id: Optional[int] = None, search: bool = True, step_deg: float = 30.0) -> SkillResult:
        skill = "find_marker"
        sectors = 12 if search else 0
        images: list[str] = []
        for index in range(sectors + 1):
            captured = self.capture_image()
            if not captured.ok:
                return self._result(skill, False, captured.status, captured.message)
            image_path = Path(captured.data["image_path"])
            images.append(str(image_path))
            try:
                detections = self.vision.find_marker(image_path, marker_id)
            except CapabilityUnavailable as exc:
                return self._result(skill, False, "unavailable", str(exc), images=images)
            if detections:
                best = max(detections, key=lambda item: item.area or 0.0)
                return self._result(skill, True, "found", f"Found marker {best.marker_id}", detection=best, image_path=image_path)
            if index < sectors:
                rotation = self.rotate(step_deg, speed=22)
                if not rotation.ok:
                    break
        return self._result(skill, False, "not_found", f"Marker {marker_id if marker_id is not None else ''} not found".strip(), images=images)

    def go_to_marker(self, marker_id: int, target_distance_cm: float = 35.0) -> SkillResult:
        found = self.find_marker(marker_id=marker_id, search=True)
        if not found.ok:
            return self._result("go_to_marker", False, found.status, found.message)
        result = self.approach_object(f"marker:{int(marker_id)}", target_distance_cm=target_distance_cm)
        return self._result("go_to_marker", result.ok, result.status, result.message, marker_id=marker_id, details=result.to_dict())

    # ------------------------------------------------------------------
    # LEDs, navigation, and user interaction
    # ------------------------------------------------------------------

    def set_led_status(self, status: str) -> SkillResult:
        colors = {
            "off": (0, 0, 0),
            "idle": (0, 70, 180),
            "moving": (255, 255, 255),
            "exploring": (0, 180, 160),
            "waiting": (150, 50, 180),
            "obstacle": (255, 120, 0),
            "success": (0, 220, 70),
            "error": (255, 0, 0),
        }
        key = str(status).strip().lower()
        if key not in colors:
            return self._result("set_led_status", False, "invalid_status", f"Unknown LED status: {status}")
        try:
            red, green, blue = colors[key]
            color = self.rover.fromRGB(red, green, blue)
            self.rover.setColor(color)
            self.rover.show()
            return self._result("set_led_status", True, "completed", f"LED status: {key}", rgb=colors[key])
        except Exception as exc:
            return self._result("set_led_status", False, "unavailable", f"LED control failed: {exc}")

    def _bearing_to(self, target: Pose2D) -> float:
        dx = target.x_cm - self.pose.x_cm
        dy = target.y_cm - self.pose.y_cm
        return normalize_heading(math.degrees(math.atan2(dx, dy)))

    def _go_to_pose(self, target: Pose2D, tolerance_cm: float, max_steps: int = 40) -> SkillResult:
        for step in range(max_steps):
            if self._external_stop_requested():
                return self._result("return_home", False, "stopped", "Return home interrupted", pose=self.pose)
            self._sync_pose()
            distance = self.pose.distance_to(target)
            if distance <= tolerance_cm:
                return self._result("return_home", True, "arrived", f"Reached home within {distance:.1f} cm", distance_cm=distance, pose=self.pose)
            desired = self._bearing_to(target)
            rotation = normalize_heading(desired - self.pose.heading_deg)
            if abs(rotation) > 5.0:
                rotate_result = self.rotate(rotation, speed=25)
                if not rotate_result.ok:
                    return self._result("return_home", False, rotate_result.status, rotate_result.message, step=step)
                if rotate_result.status == "bounded":
                    continue
            movement = self.move_forward(distance_cm=min(15.0, distance), speed=28)
            if not movement.ok:
                avoided = self.avoid_obstacle()
                if not avoided.ok:
                    return self._result("return_home", False, "blocked", "Could not clear route home", step=step, pose=self.pose)
        return self._result("return_home", False, "step_limit", "Home navigation exceeded step limit", pose=self.pose)

    def return_home(self, tolerance_cm: float = 12.0) -> SkillResult:
        return self._go_to_pose(self.home_pose, max(1.0, float(tolerance_cm)))

    def wait_for_user(self, prompt: str = "Press Enter to continue", timeout_s: Optional[float] = None, source: str = "either") -> SkillResult:
        skill = "wait_for_user"
        source = str(source).lower()
        if source not in {"terminal", "switch", "either"}:
            return self._result(skill, False, "invalid_source", f"Unknown source: {source}")
        deadline = None if timeout_s is None else time.time() + max(0.0, float(timeout_s))
        terminal_enabled = source in {"terminal", "either"} and sys.stdin is not None and sys.stdin.isatty()
        switch_enabled = source in {"switch", "either"} and callable(getattr(self.rover, "getSwitch", None))
        if terminal_enabled:
            print(prompt, flush=True)
        while True:
            if self._external_stop_requested():
                return self._result(skill, False, "stopped", "External stop requested")
            if switch_enabled:
                try:
                    if bool(self.rover.getSwitch()):
                        return self._result(skill, True, "confirmed", "User confirmed with rover switch")
                except Exception:
                    switch_enabled = False
            if terminal_enabled:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if ready:
                    sys.stdin.readline()
                    return self._result(skill, True, "confirmed", "User confirmed in terminal")
            else:
                time.sleep(0.1)
            if deadline is not None and time.time() >= deadline:
                return self._result(skill, False, "timeout", "Timed out waiting for user")
            if not terminal_enabled and not switch_enabled and deadline is None:
                return self._result(skill, False, "unavailable", "No non-blocking user input source is available")
