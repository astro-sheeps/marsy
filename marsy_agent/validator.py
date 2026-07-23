"""Semantic validation for plans returned by an LLM."""

from __future__ import annotations

from numbers import Real
from typing import Any

from .models import AgentPlan
from .specs import MAX_PLAN_STEPS, SKILL_SPECS, ArgumentSpec


class PlanValidationError(ValueError):
    def __init__(self, errors: list[str]) -> None:
        self.errors = errors
        super().__init__("Invalid Marsy plan:\n- " + "\n- ".join(errors))


class PlanValidator:
    def __init__(self, max_steps: int = MAX_PLAN_STEPS) -> None:
        self.max_steps = max(1, int(max_steps))

    def validate(self, plan: AgentPlan) -> AgentPlan:
        errors: list[str] = []
        if plan.version != 1:
            errors.append(f"Unsupported plan version: {plan.version}")
        if not plan.goal:
            errors.append("Plan goal is empty")
        if len(plan.goal) > 1000:
            errors.append("Plan goal is too long")
        if not plan.steps:
            errors.append("Plan has no steps")
        if len(plan.steps) > self.max_steps:
            errors.append(f"Plan has {len(plan.steps)} steps; maximum is {self.max_steps}")

        ids: set[str] = set()
        stop_indexes: list[int] = []
        for index, step in enumerate(plan.steps):
            prefix = f"step {index + 1}"
            if not step.id:
                errors.append(f"{prefix}: id is empty")
            elif step.id in ids:
                errors.append(f"{prefix}: duplicate id {step.id!r}")
            ids.add(step.id)

            spec = SKILL_SPECS.get(step.skill)
            if spec is None:
                errors.append(f"{prefix}: skill {step.skill!r} is not allowed")
                continue
            if step.skill == "stop":
                stop_indexes.append(index)

            expected = set(spec.arguments)
            supplied = set(step.arguments)
            missing = sorted(expected - supplied)
            extra = sorted(supplied - expected)
            if missing:
                errors.append(f"{prefix}: missing arguments: {', '.join(missing)}")
            if extra:
                errors.append(f"{prefix}: unknown arguments: {', '.join(extra)}")

            for name in sorted(expected & supplied):
                self._validate_argument(
                    errors,
                    prefix=prefix,
                    name=name,
                    value=step.arguments[name],
                    spec=spec.arguments[name],
                )

            if step.skill == "move_forward" and step.arguments.get("require_recent_scan") is not True:
                errors.append(f"{prefix}: move_forward must require a recent scan")
            if step.skill == "rotate" and self._is_number(step.arguments.get("angle_deg")):
                if abs(float(step.arguments["angle_deg"])) < 1.0:
                    errors.append(f"{prefix}: rotation angle must be at least 1 degree")

        if len(stop_indexes) != 1:
            errors.append("Plan must contain exactly one stop step")
        elif stop_indexes[0] != len(plan.steps) - 1:
            errors.append("stop must be the final plan step")

        if errors:
            raise PlanValidationError(errors)
        return plan

    @staticmethod
    def _is_number(value: Any) -> bool:
        return isinstance(value, Real) and not isinstance(value, bool)

    def _validate_argument(
        self,
        errors: list[str],
        *,
        prefix: str,
        name: str,
        value: Any,
        spec: ArgumentSpec,
    ) -> None:
        label = f"{prefix}.{name}"
        if spec.kind == "boolean":
            if not isinstance(value, bool):
                errors.append(f"{label}: expected boolean")
            return
        if spec.kind == "string":
            if not isinstance(value, str):
                errors.append(f"{label}: expected string")
                return
            if spec.choices and value not in spec.choices:
                errors.append(f"{label}: must be one of {', '.join(map(str, spec.choices))}")
            return
        if spec.kind == "integer":
            if not isinstance(value, int) or isinstance(value, bool):
                errors.append(f"{label}: expected integer")
                return
        elif spec.kind == "number":
            if not self._is_number(value):
                errors.append(f"{label}: expected number")
                return
        else:
            errors.append(f"{label}: unsupported validator type {spec.kind!r}")
            return

        numeric = float(value)
        if spec.minimum is not None and numeric < spec.minimum:
            errors.append(f"{label}: {value} is below minimum {spec.minimum}")
        if spec.maximum is not None and numeric > spec.maximum:
            errors.append(f"{label}: {value} exceeds maximum {spec.maximum}")
        if spec.choices and value not in spec.choices:
            errors.append(f"{label}: must be one of {', '.join(map(str, spec.choices))}")
