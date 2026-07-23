"""Validated mission-plan executor for Marsy."""

from __future__ import annotations

import time
from typing import Any, Callable

from behaviors.explore_area import ExploreAreaConfig, run_explore_area
from behaviors.explore_navigation import choose_turn_direction

from .models import (
    AgentPlan,
    CompositeSkillResult,
    ExecutionReport,
    PlanStep,
    StepRecord,
    jsonable,
)
from .planner import Planner
from .specs import MAX_REPLANS
from .validator import PlanValidator

REPLANNABLE_STATUSES = {
    "blocked",
    "range_unknown",
    "stale_scan",
    "unknown",
    "no_clear_direction",
    "no_progress",
}
NONCRITICAL_SKILLS: set[str] = set()
SCAN_ANGLES = [-75.0, -50.0, -25.0, 0.0, 25.0, 50.0, 75.0]
CLEAR_DIRECTION_TURN_STEP_DEG = 30.0
CLEAR_DIRECTION_MAX_TURNS = 4


class AgentExecutor:
    def __init__(
        self,
        *,
        skills: Any,
        planner: Planner,
        validator: PlanValidator | None = None,
        max_replans: int = MAX_REPLANS,
        progress_callback: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self.skills = skills
        self.planner = planner
        self.validator = validator or PlanValidator()
        self.max_replans = max(0, min(2, int(max_replans)))
        self.progress_callback = progress_callback

    def execute(self, *, goal: str, plan: AgentPlan) -> ExecutionReport:
        initial_plan = self.validator.validate(plan)
        current_plan = initial_plan
        records: list[StepRecord] = []
        replans: list[AgentPlan] = []
        completed_context: list[dict[str, Any]] = []
        started_at = time.time()
        status = "failed"
        message = "Mission failed"
        failure: dict[str, Any] | None = None
        replan_count = 0

        try:
            while True:
                restart_with_replan = False
                total_steps = len(current_plan.steps)
                for step_index, step in enumerate(current_plan.steps, start=1):
                    if self._external_stop_requested():
                        status = "stopped"
                        message = "Agent mission stopped"
                        failure = {"status": status, "message": message}
                        restart_with_replan = False
                        break
                    self._emit_progress({
                        "phase": "step_started",
                        "plan_revision": replan_count,
                        "step": step_index,
                        "total_steps": total_steps,
                        "step_id": step.id,
                        "skill": step.skill,
                    })
                    step_started = time.time()
                    result = self._dispatch(step)
                    step_finished = time.time()
                    result_data = self._result_dict(result)
                    self._emit_progress({
                        "phase": "step_finished",
                        "plan_revision": replan_count,
                        "step": step_index,
                        "total_steps": total_steps,
                        "step_id": step.id,
                        "skill": step.skill,
                        "ok": self._result_ok(result),
                        "status": result_data.get("status"),
                        "message": result_data.get("message"),
                    })
                    noncritical = step.skill in NONCRITICAL_SKILLS and not self._result_ok(result)
                    records.append(
                        StepRecord(
                            plan_revision=replan_count,
                            step=step,
                            result=result_data,
                            started_at=step_started,
                            finished_at=step_finished,
                            noncritical=noncritical,
                        )
                    )

                    if self._result_ok(result) or noncritical:
                        completed_context.append(
                            {
                                "step": step.to_dict(),
                                "result": result_data,
                                "noncritical": noncritical,
                            }
                        )
                        continue

                    failure = result_data
                    result_status = str(result_data.get("status", "failed"))
                    if (
                        result_status in REPLANNABLE_STATUSES
                        and replan_count < self.max_replans
                    ):
                        if self._external_stop_requested():
                            status = "stopped"
                            message = "Agent mission stopped before replanning"
                            failure = {"status": status, "message": message}
                            restart_with_replan = False
                            break
                        remaining = self.max_replans - replan_count
                        self._emit_progress({
                            "phase": "replanning",
                            "plan_revision": replan_count,
                            "failed_step": step.id,
                            "remaining_replans": remaining,
                        })
                        replacement = self.planner.replan(
                            goal=goal,
                            completed_steps=completed_context,
                            failed_step=step.to_dict(),
                            failure=result_data,
                            remaining_replans=remaining,
                        )
                        current_plan = self.validator.validate(replacement)
                        replans.append(current_plan)
                        replan_count += 1
                        self._emit_progress({
                            "phase": "replanned",
                            "plan_revision": replan_count,
                            "plan_source": "replan",
                            "plan": current_plan.to_dict(),
                            "step": 0,
                            "total_steps": len(current_plan.steps),
                        })
                        restart_with_replan = True
                        break

                    status = result_status
                    message = result_data.get("message") or f"Step {step.id} failed"
                    restart_with_replan = False
                    break
                else:
                    status = "completed"
                    message = "Agent mission completed"
                    failure = None
                    break

                if restart_with_replan:
                    continue
                break

        except KeyboardInterrupt:
            status = "stopped"
            message = "Agent mission interrupted"
            failure = {"status": status, "message": message}
        except Exception as exc:
            status = "error"
            message = f"{type(exc).__name__}: {exc}"
            failure = {"status": status, "message": message}
        finally:
            self._emit_progress({"phase": "finished", "status": status, "message": message})
            self._safe_stop()

        return ExecutionReport(
            goal=goal,
            ok=status == "completed",
            status=status,
            message=message,
            initial_plan=initial_plan,
            final_plan=current_plan,
            step_records=records,
            replans=replans,
            started_at=started_at,
            finished_at=time.time(),
            failure=failure,
        )

    def _dispatch(self, step: PlanStep) -> Any:
        if step.skill == "turn_to_clear_direction":
            return self._turn_to_clear_direction(**step.arguments)
        if step.skill == "explore_for":
            return self._explore_for(**step.arguments)
        if step.skill == "wait":
            return self._wait(**step.arguments)
        return self.skills.call(step.skill, **step.arguments)

    def _turn_to_clear_direction(
        self,
        *,
        rotate_speed: int,
    ) -> CompositeSkillResult:
        """Orient Marsy using committed, incremental sonar-guided turns.

        The LLM chooses only this composite skill. Python selects the turn
        direction from side evidence, keeps that direction until a safe
        corridor is found, and validates the corridor after every 30° turn.
        This mirrors the stable local policy used by ``explore_area`` and
        avoids oscillating between two individually promising sonar rays.
        """
        started = time.time()
        scan = self.skills.scan_arc(angles=SCAN_ANGLES, samples=3)
        if not self._result_ok(scan):
            return self._composite_from(
                "turn_to_clear_direction",
                scan,
                started,
                message="Initial direction scan failed",
            )

        samples = list(getattr(scan, "data", {}).get("samples", []))
        safe_distance = float(
            getattr(getattr(self.skills, "config", None), "safe_distance_cm", 40.0)
        )
        danger_distance = float(
            getattr(getattr(self.skills, "config", None), "danger_distance_cm", 24.0)
        )
        corridor_getter = getattr(self.skills, "assess_forward_corridor", None)

        def assess(scan_samples: list[Any]) -> dict[str, Any]:
            if callable(corridor_getter):
                return corridor_getter(scan_samples)
            return self._fallback_corridor_assessment(
                scan_samples,
                safe_distance=safe_distance,
                danger_distance=danger_distance,
            )

        initial_corridor = assess(samples)
        if initial_corridor.get("safe"):
            return CompositeSkillResult(
                skill="turn_to_clear_direction",
                ok=True,
                status=str(initial_corridor.get("status", "clear")),
                message="Forward direction is already usable",
                data={
                    "direction": "forward",
                    "angle_deg": 0.0,
                    "corridor": jsonable(initial_corridor),
                    "scan": self._result_dict(scan),
                    "attempts": [],
                },
                started_at=started,
                finished_at=time.time(),
            )

        direction_sign, left_evidence, right_evidence, reason = choose_turn_direction(
            samples,
            safe_distance_cm=safe_distance,
            committed_sign=0,
            default_sign=1,
        )
        direction_sign = -1 if int(direction_sign) < 0 else 1
        direction = "left" if direction_sign < 0 else "right"
        attempts: list[dict[str, Any]] = []
        cumulative_turn = 0.0
        last_corridor = initial_corridor
        last_scan = scan

        for attempt_index in range(1, CLEAR_DIRECTION_MAX_TURNS + 1):
            if self._external_stop_requested():
                return CompositeSkillResult(
                    skill="turn_to_clear_direction",
                    ok=False,
                    status="stopped",
                    message="Direction search stopped",
                    data={"attempts": attempts},
                    started_at=started,
                    finished_at=time.time(),
                )
            commanded_turn = direction_sign * CLEAR_DIRECTION_TURN_STEP_DEG
            rotation = self.skills.rotate(commanded_turn, speed=int(rotate_speed))
            if not self._result_ok(rotation):
                return self._composite_from(
                    "turn_to_clear_direction",
                    rotation,
                    started,
                    message=f"Could not continue turning {direction}",
                )
            cumulative_turn += commanded_turn

            # The fresh scan is retained by MarsySkills and can be reused by
            # the following move_forward(require_recent_scan=True) step.
            post_scan = self.skills.scan_arc(angles=SCAN_ANGLES, samples=3)
            if not self._result_ok(post_scan):
                return self._composite_from(
                    "turn_to_clear_direction",
                    post_scan,
                    started,
                    message="Post-turn scan failed",
                )

            post_samples = list(getattr(post_scan, "data", {}).get("samples", []))
            corridor = assess(post_samples)
            attempts.append(
                {
                    "attempt": attempt_index,
                    "direction": direction,
                    "commanded_turn_deg": commanded_turn,
                    "cumulative_turn_deg": cumulative_turn,
                    "corridor": jsonable(corridor),
                    "scan": self._result_dict(post_scan),
                }
            )
            last_corridor = corridor
            last_scan = post_scan

            if corridor.get("safe"):
                return CompositeSkillResult(
                    skill="turn_to_clear_direction",
                    ok=True,
                    status=str(corridor.get("status", "completed")),
                    message=(
                        f"Found a usable corridor after turning {direction} "
                        f"{abs(cumulative_turn):.0f}°"
                    ),
                    data={
                        "direction": direction,
                        "angle_deg": cumulative_turn,
                        "selection_reason": reason,
                        "left_evidence": jsonable(left_evidence),
                        "right_evidence": jsonable(right_evidence),
                        "attempts": attempts,
                        "corridor": jsonable(corridor),
                    },
                    started_at=started,
                    finished_at=time.time(),
                )

        return CompositeSkillResult(
            skill="turn_to_clear_direction",
            ok=False,
            status=str(last_corridor.get("status", "no_clear_direction")),
            message=(
                f"No usable corridor found after {CLEAR_DIRECTION_MAX_TURNS} "
                f"committed turns to the {direction}"
            ),
            data={
                "direction": direction,
                "angle_deg": cumulative_turn,
                "selection_reason": reason,
                "left_evidence": jsonable(left_evidence),
                "right_evidence": jsonable(right_evidence),
                "attempts": attempts,
                "corridor": jsonable(last_corridor),
                "scan": self._result_dict(last_scan),
            },
            started_at=started,
            finished_at=time.time(),
        )

    def _explore_for(
        self,
        *,
        duration_seconds: float,
        step_distance_cm: float,
        speed: int,
        rotate_speed: int,
        max_steps: int,
    ) -> CompositeSkillResult:
        started = time.time()
        safe_distance = float(
            getattr(getattr(self.skills, "config", None), "safe_distance_cm", 40.0)
        )
        result = run_explore_area(
            self.skills,
            ExploreAreaConfig(
                steps=int(max_steps),
                run_seconds=float(duration_seconds),
                step_distance_cm=float(step_distance_cm),
                speed=int(speed),
                rotate_speed=int(rotate_speed),
                safe_distance_cm=safe_distance,
                clear_existing_maps=True,
                save_maps=True,
                source="agent_mission",
                map_mission="agent_mission:explore_for",
            ),
            stop_requested=self._external_stop_requested,
        )
        return CompositeSkillResult(
            skill="explore_for",
            ok=result.ok,
            status=result.status,
            message=result.message,
            data=result.to_dict(),
            started_at=started,
            finished_at=time.time(),
        )

    def _wait(self, *, duration_seconds: float) -> CompositeSkillResult:
        started = time.time()
        duration = max(0.0, float(duration_seconds))
        while time.time() - started < duration:
            if self._external_stop_requested():
                return CompositeSkillResult(
                    skill="wait",
                    ok=False,
                    status="stopped",
                    message="Wait interrupted",
                    started_at=started,
                    finished_at=time.time(),
                )
            time.sleep(min(0.1, duration - (time.time() - started)))
        return CompositeSkillResult(
            skill="wait",
            ok=True,
            status="completed",
            message=f"Waited {duration:.1f} seconds",
            started_at=started,
            finished_at=time.time(),
        )

    @staticmethod
    def _sample_angle(sample: Any) -> float | None:
        try:
            return float(getattr(sample, "mast_angle_deg"))
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _sample_distance(sample: Any) -> float | None:
        try:
            value = getattr(sample, "distance_cm")
            return None if value is None else float(value)
        except (TypeError, ValueError):
            return None

    def _trusted_distances(self, samples: list[Any], *, lower: float, upper: float) -> list[float]:
        values: list[float] = []
        for sample in samples:
            angle = self._sample_angle(sample)
            distance = self._sample_distance(sample)
            trusted = bool(getattr(sample, "trusted", False))
            if angle is not None and lower <= angle <= upper and trusted and distance is not None and distance > 0:
                values.append(distance)
        return values

    def _observed_distances(self, samples: list[Any], *, lower: float, upper: float) -> list[float]:
        values: list[float] = []
        for sample in samples:
            angle = self._sample_angle(sample)
            if angle is None or not (lower <= angle <= upper):
                continue
            distance = self._sample_distance(sample)
            if distance is not None and distance > 0:
                values.append(distance)
            for raw in getattr(sample, "readings", []) or []:
                try:
                    numeric = float(raw)
                except (TypeError, ValueError):
                    continue
                if numeric > 0:
                    values.append(numeric)
        return values

    def _clear_direction_candidates(
        self,
        samples: list[Any],
        *,
        safe_distance: float,
        danger_distance: float,
        limit: int,
    ) -> list[dict[str, float]]:
        ranked: list[dict[str, float]] = []
        for sample in samples:
            angle = self._sample_angle(sample)
            distance = self._sample_distance(sample)
            trusted = bool(getattr(sample, "trusted", False))
            if (
                angle is None
                or distance is None
                or not trusted
                or abs(angle) < 12.0
                or distance < safe_distance
            ):
                continue
            readings = self._sample_readings(sample)
            if readings and min(readings) < danger_distance:
                continue
            ranked.append(
                {
                    "angle_deg": float(angle),
                    "distance_cm": float(distance),
                }
            )

        ranked.sort(key=lambda item: (-item["distance_cm"], abs(item["angle_deg"])))
        if not ranked or limit <= 0:
            return []

        selected = [ranked[0]]
        first_sign = -1 if ranked[0]["angle_deg"] < 0 else 1
        opposite = next(
            (
                item
                for item in ranked[1:]
                if (-1 if item["angle_deg"] < 0 else 1) != first_sign
            ),
            None,
        )
        if opposite is not None and len(selected) < limit:
            selected.append(opposite)

        for item in ranked[1:]:
            if len(selected) >= limit:
                break
            if item in selected:
                continue
            if all(
                abs(item["angle_deg"] - chosen["angle_deg"]) >= 20.0
                for chosen in selected
            ):
                selected.append(item)
        return selected

    def _fallback_corridor_assessment(
        self,
        samples: list[Any],
        *,
        safe_distance: float,
        danger_distance: float,
    ) -> dict[str, Any]:
        center = self._trusted_distances(samples, lower=-12.0, upper=12.0)
        observed = self._observed_distances(samples, lower=-12.0, upper=12.0)
        minimum_observed = min(observed) if observed else None
        if minimum_observed is not None and minimum_observed < danger_distance:
            return {
                "safe": False,
                "status": "blocked",
                "message": f"Front corridor is blocked at {minimum_observed:.1f} cm",
                "minimum_distance_cm": minimum_observed,
            }
        if center and min(center) >= safe_distance:
            return {
                "safe": True,
                "status": "clear",
                "message": f"Front corridor is clear at {min(center):.1f} cm",
                "minimum_distance_cm": min(center),
            }
        return {
            "safe": False,
            "status": "range_unknown" if not center else "blocked",
            "message": "Front corridor is not confirmed safe",
            "minimum_distance_cm": min(center) if center else None,
        }

    @staticmethod
    def _sample_readings(sample: Any) -> list[float]:
        values: list[float] = []
        for raw in getattr(sample, "readings", []) or []:
            try:
                numeric = float(raw)
            except (TypeError, ValueError):
                continue
            if numeric > 0:
                values.append(numeric)
        return values

    def _external_stop_requested(self) -> bool:
        checker = getattr(self.skills, "_external_stop_requested", None)
        if callable(checker):
            try:
                return bool(checker())
            except Exception:
                return True
        return False

    @staticmethod
    def _result_ok(result: Any) -> bool:
        if isinstance(result, dict):
            return bool(result.get("ok"))
        return bool(getattr(result, "ok", False))

    @staticmethod
    def _result_dict(result: Any) -> dict[str, Any]:
        if isinstance(result, dict):
            return jsonable(result)
        if hasattr(result, "to_dict") and callable(result.to_dict):
            return jsonable(result.to_dict())
        return {
            "skill": str(getattr(result, "skill", "unknown")),
            "ok": bool(getattr(result, "ok", False)),
            "status": str(getattr(result, "status", "unknown")),
            "message": str(getattr(result, "message", "")),
            "data": jsonable(getattr(result, "data", {})),
        }

    def _composite_from(
        self,
        skill: str,
        result: Any,
        started: float,
        *,
        message: str,
    ) -> CompositeSkillResult:
        data = self._result_dict(result)
        return CompositeSkillResult(
            skill=skill,
            ok=False,
            status=str(data.get("status", "failed")),
            message=message + (f": {data.get('message')}" if data.get("message") else ""),
            data={"cause": data},
            started_at=started,
            finished_at=time.time(),
        )

    def _emit_progress(self, payload: dict[str, Any]) -> None:
        if self.progress_callback is None:
            return
        try:
            self.progress_callback(dict(payload))
        except Exception:
            pass

    def _safe_stop(self) -> None:
        try:
            self.skills.stop(brake=False, reset_pose=True)
        except Exception:
            try:
                self.skills.call("stop", brake=False, reset_pose=True)
            except Exception:
                pass
