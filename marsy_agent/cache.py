"""Single-entry cache for the latest validated Groq plan."""

from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path
from typing import Any

from .models import AgentPlan, jsonable
from .prompts import SYSTEM_PROMPT
from .specs import build_plan_json_schema

CACHE_VERSION = 1
DEFAULT_FILENAME = "agent_plan_latest.json"


def _planner_fingerprint() -> str:
    payload = {
        "cache_version": CACHE_VERSION,
        "system_prompt": SYSTEM_PROMPT,
        "schema": build_plan_json_schema(),
    }
    encoded = json.dumps(payload, ensure_ascii=False, sort_keys=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class PlanCache:
    """Store only the latest initial plan and reuse it on an exact match."""

    def __init__(
        self,
        *,
        directory: str | Path = "artifacts/agent_cache",
        filename: str = DEFAULT_FILENAME,
    ) -> None:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / filename
        self.fingerprint = _planner_fingerprint()

    @staticmethod
    def _normalise_goal(goal: str) -> str:
        return " ".join(str(goal).split())

    def load(self, *, goal: str, planner: str, model: str) -> AgentPlan | None:
        if not self.path.exists():
            return None
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            if payload.get("cache_version") != CACHE_VERSION:
                return None
            if payload.get("planner_fingerprint") != self.fingerprint:
                return None
            if payload.get("planner") != planner:
                return None
            if payload.get("model") != model:
                return None
            if payload.get("goal") != self._normalise_goal(goal):
                return None
            plan_payload = payload.get("plan")
            if not isinstance(plan_payload, dict):
                return None
            return AgentPlan.from_dict(plan_payload)
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return None

    def save(self, *, goal: str, planner: str, model: str, plan: AgentPlan) -> None:
        payload: dict[str, Any] = {
            "cache_version": CACHE_VERSION,
            "planner_fingerprint": self.fingerprint,
            "planner": planner,
            "model": model,
            "goal": self._normalise_goal(goal),
            "saved_at": time.time(),
            "plan": jsonable(plan),
        }
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        temporary.replace(self.path)

    def clear(self) -> None:
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass
