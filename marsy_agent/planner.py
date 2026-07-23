"""Groq and deterministic mock planners for Marsy."""

from __future__ import annotations

import json
import os
from typing import Any, Protocol

from .models import AgentPlan
from .prompts import SYSTEM_PROMPT, create_plan_prompt, replan_prompt
from .specs import build_plan_json_schema

DEFAULT_MODEL = "openai/gpt-oss-20b"


class Planner(Protocol):
    model: str

    def create_plan(self, goal: str) -> AgentPlan: ...

    def replan(
        self,
        *,
        goal: str,
        completed_steps: list[dict[str, Any]],
        failed_step: dict[str, Any],
        failure: dict[str, Any],
        remaining_replans: int,
    ) -> AgentPlan: ...


class GroqPlanner:
    def __init__(
        self,
        *,
        model: str | None = None,
        api_key: str | None = None,
        timeout_seconds: float = 30.0,
    ) -> None:
        self.model = model or os.getenv("MARSY_LLM_MODEL", DEFAULT_MODEL)
        self.api_key = api_key or os.getenv("GROQ_API_KEY")
        self.timeout_seconds = max(1.0, float(timeout_seconds))

    def _client(self):
        if not self.api_key:
            raise RuntimeError(
                "GROQ_API_KEY is not set. Export it before requesting a new Groq plan, "
                "or use a matching cached plan / --planner mock."
            )
        try:
            from groq import Groq
        except ImportError as exc:
            raise RuntimeError(
                "The Groq Python SDK is not installed. Run: python -m pip install groq"
            ) from exc
        return Groq(api_key=self.api_key, timeout=self.timeout_seconds)

    def _request(self, user_prompt: str) -> AgentPlan:
        client = self._client()
        response = client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": user_prompt},
            ],
            temperature=0.1,
            max_completion_tokens=2200,
            response_format={
                "type": "json_schema",
                "json_schema": {
                    "name": "marsy_agent_plan",
                    "strict": True,
                    "schema": build_plan_json_schema(),
                },
            },
        )
        content = response.choices[0].message.content
        if not content:
            raise RuntimeError("Groq returned an empty plan")
        try:
            payload = json.loads(content)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Groq returned invalid JSON: {exc}") from exc
        return AgentPlan.from_dict(payload)

    def create_plan(self, goal: str) -> AgentPlan:
        return self._request(create_plan_prompt(goal))

    def replan(
        self,
        *,
        goal: str,
        completed_steps: list[dict[str, Any]],
        failed_step: dict[str, Any],
        failure: dict[str, Any],
        remaining_replans: int,
    ) -> AgentPlan:
        return self._request(
            replan_prompt(
                goal=goal,
                completed_steps=completed_steps,
                failed_step=failed_step,
                failure=failure,
                remaining_replans=remaining_replans,
            )
        )


class MockPlanner:
    """Offline deterministic planner for simulator and validator testing."""

    model = "mock/cautious-v1"

    @staticmethod
    def _plan(goal: str, steps: list[dict[str, Any]]) -> AgentPlan:
        return AgentPlan.from_dict({"version": 1, "goal": goal, "steps": steps})

    def create_plan(self, goal: str) -> AgentPlan:
        return self._plan(
            goal,
            [
                {
                    "id": "step_1",
                    "skill": "turn_to_clear_direction",
                    "arguments": {"rotate_speed": 22},
                },
                {
                    "id": "step_2",
                    "skill": "move_forward",
                    "arguments": {
                        "distance_cm": 6,
                        "speed": 22,
                        "require_recent_scan": True,
                    },
                },
                {
                    "id": "step_3",
                    "skill": "explore_for",
                    "arguments": {
                        "duration_seconds": 30,
                        "step_distance_cm": 6,
                        "speed": 22,
                        "rotate_speed": 22,
                        "max_steps": 10,
                    },
                },
                {
                    "id": "step_4",
                    "skill": "stop",
                    "arguments": {"brake": False, "reset_pose": True},
                },
            ],
        )

    def replan(
        self,
        *,
        goal: str,
        completed_steps: list[dict[str, Any]],
        failed_step: dict[str, Any],
        failure: dict[str, Any],
        remaining_replans: int,
    ) -> AgentPlan:
        del completed_steps, failed_step, failure, remaining_replans
        return self._plan(
            goal,
            [
                {
                    "id": "replan_1",
                    "skill": "turn_to_clear_direction",
                    "arguments": {"rotate_speed": 22},
                },
                {
                    "id": "replan_2",
                    "skill": "move_forward",
                    "arguments": {
                        "distance_cm": 6,
                        "speed": 22,
                        "require_recent_scan": True,
                    },
                },
                {
                    "id": "replan_3",
                    "skill": "stop",
                    "arguments": {"brake": False, "reset_pose": True},
                },
            ],
        )
