"""Whitelisted skills and hard safety limits for LLM-generated plans."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

MAX_PLAN_STEPS = 8
MAX_REPLANS = 1


@dataclass(frozen=True)
class ArgumentSpec:
    kind: str
    minimum: float | None = None
    maximum: float | None = None
    choices: tuple[Any, ...] = ()


@dataclass(frozen=True)
class SkillSpec:
    description: str
    arguments: dict[str, ArgumentSpec]


SKILL_SPECS: dict[str, SkillSpec] = {
    "scan_arc": SkillSpec(
        "Sweep the mast sonar over a bounded arc and store a fresh range scan.",
        {
            "start_deg": ArgumentSpec("number", -90, 90),
            "end_deg": ArgumentSpec("number", -90, 90),
            "step_deg": ArgumentSpec("number", 5, 45),
            "samples": ArgumentSpec("integer", 1, 5),
        },
    ),
    "turn_to_clear_direction": SkillSpec(
        "Scan deterministically, select the best safe sonar heading, rotate to it, and verify the new corridor.",
        {"rotate_speed": ArgumentSpec("integer", 1, 35)},
    ),
    "move_forward": SkillSpec(
        "Move a short bounded distance with sonar safety checks and a mandatory recent scan.",
        {
            "distance_cm": ArgumentSpec("number", 1, 30),
            "speed": ArgumentSpec("integer", 1, 35),
            "require_recent_scan": ArgumentSpec("boolean"),
        },
    ),
    "move_backward": SkillSpec(
        "Move backward a short bounded distance. Rearward motion has no sonar protection.",
        {
            "distance_cm": ArgumentSpec("number", 1, 15),
            "speed": ArgumentSpec("integer", 1, 30),
        },
    ),
    "turn_left": SkillSpec(
        "Drive a bounded forward arc to the left.",
        {
            "angle_deg": ArgumentSpec("number", 1, 90),
            "speed": ArgumentSpec("integer", 1, 35),
            "steer_angle": ArgumentSpec("number", 5, 45),
        },
    ),
    "turn_right": SkillSpec(
        "Drive a bounded forward arc to the right.",
        {
            "angle_deg": ArgumentSpec("number", 1, 90),
            "speed": ArgumentSpec("integer", 1, 35),
            "steer_angle": ArgumentSpec("number", 5, 45),
        },
    ),
    "rotate": SkillSpec(
        "Rotate in place. Positive angles turn right; negative angles turn left.",
        {
            "angle_deg": ArgumentSpec("number", -180, 180),
            "speed": ArgumentSpec("integer", 1, 35),
        },
    ),
    "explore_for": SkillSpec(
        "Run a deterministic scan-turn-move exploration loop for a bounded duration.",
        {
            "duration_seconds": ArgumentSpec("number", 5, 180),
            "step_distance_cm": ArgumentSpec("number", 1, 12),
            "speed": ArgumentSpec("integer", 1, 35),
            "rotate_speed": ArgumentSpec("integer", 1, 35),
            "max_steps": ArgumentSpec("integer", 1, 30),
        },
    ),
    "wait": SkillSpec(
        "Wait without moving for a bounded time.",
        {"duration_seconds": ArgumentSpec("number", 0, 30)},
    ),
    "stop": SkillSpec(
        "Stop motors and reset the rover pose. This must be the final step.",
        {
            "brake": ArgumentSpec("boolean"),
            "reset_pose": ArgumentSpec("boolean"),
        },
    ),
}


def _argument_schema(spec: ArgumentSpec) -> dict[str, Any]:
    schema: dict[str, Any] = {"type": spec.kind}
    if spec.minimum is not None:
        schema["minimum"] = spec.minimum
    if spec.maximum is not None:
        schema["maximum"] = spec.maximum
    if spec.choices:
        schema["enum"] = list(spec.choices)
    return schema


def build_plan_json_schema() -> dict[str, Any]:
    """Build a strict Groq Structured Outputs schema for the whitelist."""
    step_variants: list[dict[str, Any]] = []
    for skill_name, skill_spec in SKILL_SPECS.items():
        argument_properties = {
            name: _argument_schema(argument_spec)
            for name, argument_spec in skill_spec.arguments.items()
        }
        step_variants.append(
            {
                "type": "object",
                "properties": {
                    "id": {"type": "string"},
                    "skill": {"type": "string", "enum": [skill_name]},
                    "arguments": {
                        "type": "object",
                        "properties": argument_properties,
                        "required": list(argument_properties),
                        "additionalProperties": False,
                    },
                },
                "required": ["id", "skill", "arguments"],
                "additionalProperties": False,
            }
        )

    return {
        "type": "object",
        "properties": {
            "version": {"type": "integer", "enum": [1]},
            "goal": {"type": "string"},
            "steps": {
                "type": "array",
                "minItems": 1,
                "maxItems": MAX_PLAN_STEPS,
                "items": {"anyOf": step_variants},
            },
        },
        "required": ["version", "goal", "steps"],
        "additionalProperties": False,
    }


def skill_catalog_text() -> str:
    lines = []
    for name, spec in SKILL_SPECS.items():
        arguments = ", ".join(spec.arguments) or "no arguments"
        lines.append(f"- {name}({arguments}): {spec.description}")
    return "\n".join(lines)
