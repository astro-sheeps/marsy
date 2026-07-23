"""Data models used by the Marsy agent mission."""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field, is_dataclass
from pathlib import Path
from typing import Any


def jsonable(value: Any) -> Any:
    """Convert nested Marsy results and dataclasses into JSON-safe values."""
    # Dataclasses may also expose to_dict(); handle them first to avoid a
    # recursive to_dict() -> jsonable(self) loop in our own result models.
    if is_dataclass(value):
        return jsonable(asdict(value))
    if hasattr(value, "to_dict") and callable(value.to_dict):
        return jsonable(value.to_dict())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


@dataclass(frozen=True)
class PlanStep:
    id: str
    skill: str
    arguments: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "PlanStep":
        if not isinstance(value, dict):
            raise TypeError("Each plan step must be an object")
        return cls(
            id=str(value.get("id", "")).strip(),
            skill=str(value.get("skill", "")).strip(),
            arguments=dict(value.get("arguments") or {}),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "skill": self.skill,
            "arguments": jsonable(self.arguments),
        }


@dataclass(frozen=True)
class AgentPlan:
    version: int
    goal: str
    steps: tuple[PlanStep, ...]

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "AgentPlan":
        if not isinstance(value, dict):
            raise TypeError("Plan must be a JSON object")
        raw_steps = value.get("steps")
        if not isinstance(raw_steps, list):
            raise TypeError("Plan.steps must be an array")
        return cls(
            version=int(value.get("version", 1)),
            goal=str(value.get("goal", "")).strip(),
            steps=tuple(PlanStep.from_dict(step) for step in raw_steps),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": self.version,
            "goal": self.goal,
            "steps": [step.to_dict() for step in self.steps],
        }


@dataclass
class CompositeSkillResult:
    skill: str
    ok: bool
    status: str
    message: str = ""
    data: dict[str, Any] = field(default_factory=dict)
    started_at: float = field(default_factory=time.time)
    finished_at: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return jsonable(self)


@dataclass
class StepRecord:
    plan_revision: int
    step: PlanStep
    result: dict[str, Any]
    started_at: float
    finished_at: float
    noncritical: bool = False

    def to_dict(self) -> dict[str, Any]:
        return jsonable(self)


@dataclass
class ExecutionReport:
    goal: str
    ok: bool
    status: str
    message: str
    initial_plan: AgentPlan
    final_plan: AgentPlan
    step_records: list[StepRecord] = field(default_factory=list)
    replans: list[AgentPlan] = field(default_factory=list)
    started_at: float = field(default_factory=time.time)
    finished_at: float = field(default_factory=time.time)
    failure: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        return jsonable(self)
