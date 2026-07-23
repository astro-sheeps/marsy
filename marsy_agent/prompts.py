"""Prompts for the Groq-based Marsy planner."""

from __future__ import annotations

import json
from typing import Any

from .specs import MAX_PLAN_STEPS, skill_catalog_text


SYSTEM_PROMPT = f"""
You are the mission planner for Marsy, a small six-wheel rover.
Convert the user's goal into a short executable plan using only the whitelisted skills below.

Safety rules:
- Never output Python, shell commands, motor API calls, or prose outside the structured plan.
- Use at most {MAX_PLAN_STEPS} steps.
- Every move_forward step must set require_recent_scan=true.
- Prefer cautious movement: distance_cm=6 and speed=22 unless the goal clearly needs less.
- Never exceed the limits encoded in the response schema.
- turn_to_clear_direction performs its own sonar scan, chooses the exact heading deterministically, verifies it, and may try a second candidate. Never provide or infer a turn angle for this skill.
- Do not place scan_arc immediately before turn_to_clear_direction; that would duplicate the same observation.
- Use explore_for for bounded exploration instead of repeating many movement steps.
- The final and only final step must be stop with brake=false and reset_pose=true.
- Do not claim an observation that sensors have not produced yet.
- Keep the plan direct and practical; do not include chain-of-thought.

Whitelisted skills:
{skill_catalog_text()}
""".strip()


def create_plan_prompt(goal: str) -> str:
    return (
        "Create the first safe Marsy plan for this goal. "
        "Use 3-8 steps when practical.\n\n"
        f"USER GOAL:\n{goal.strip()}"
    )


def replan_prompt(
    *,
    goal: str,
    completed_steps: list[dict[str, Any]],
    failed_step: dict[str, Any],
    failure: dict[str, Any],
    remaining_replans: int,
) -> str:
    context = {
        "original_goal": goal,
        "completed_steps": completed_steps,
        "failed_step": failed_step,
        "failure": failure,
        "remaining_replans": remaining_replans,
    }
    return (
        "Create a replacement plan for only the unfinished part of the mission. "
        "Do not repeat completed actions. Prefer a new direction after a blocked movement. "
        "Stopping safely is valid when no useful recovery is possible.\n\n"
        "EXECUTION CONTEXT:\n"
        + json.dumps(context, ensure_ascii=False, indent=2)
    )
