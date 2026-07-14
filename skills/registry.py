"""LLM/tool-call metadata for the Marsy skill API."""

from __future__ import annotations

from copy import deepcopy
from typing import Any


def _tool(name: str, description: str, properties: dict[str, Any], required: list[str] | None = None) -> dict[str, Any]:
    return {
        "type": "function",
        "function": {
            "name": name,
            "description": description,
            "parameters": {
                "type": "object",
                "properties": properties,
                "required": required or [],
                "additionalProperties": False,
            },
        },
    }


SPEED = {"type": "integer", "minimum": 0, "maximum": 100}
ANGLE = {"type": "number", "minimum": -360, "maximum": 360}
DISTANCE = {"type": "number", "minimum": 0}

SKILL_SCHEMAS = [
    _tool("move_forward", "Move forward a bounded distance with ultrasonic safety checks.", {"distance_cm": DISTANCE, "speed": SPEED}),
    _tool("move_backward", "Move backward a bounded distance.", {"distance_cm": DISTANCE, "speed": SPEED}),
    _tool("turn_left", "Drive a bounded forward arc to the left.", {"angle_deg": DISTANCE, "speed": SPEED, "steer_angle": DISTANCE}),
    _tool("turn_right", "Drive a bounded forward arc to the right.", {"angle_deg": DISTANCE, "speed": SPEED, "steer_angle": DISTANCE}),
    _tool("rotate", "Rotate in place. Positive angle is right/clockwise; negative is left.", {"angle_deg": ANGLE, "speed": SPEED}, ["angle_deg"]),
    _tool("stop", "Stop motors and optionally brake/reset steering.", {"brake": {"type": "boolean"}, "reset_pose": {"type": "boolean"}}),
    _tool("scan_arc", "Sweep the mast ultrasonic sensor across an arc and update the occupancy map.", {"start_deg": ANGLE, "end_deg": ANGLE, "step_deg": DISTANCE, "samples": {"type": "integer", "minimum": 1, "maximum": 10}}),
    _tool("scan_360", "Rotate through a full circle, collecting ultrasonic scans and updating the map.", {"step_deg": {"type": "number", "minimum": 15, "maximum": 180}, "rotate_speed": SPEED}),
    _tool("measure_distance", "Measure median ultrasonic distance in centimeters.", {"samples": {"type": "integer", "minimum": 1, "maximum": 15}}),
    _tool("avoid_obstacle", "Perform one bounded obstacle-avoidance maneuver using sonar scanning.", {}),
    _tool("search_visual", "Rotate and search camera images for a requested visual object.", {"query": {"type": "string"}, "step_deg": DISTANCE, "max_rotation_deg": DISTANCE}, ["query"]),
    _tool("track_object", "Center a detected object in the camera view by small rotations.", {"query": {"type": "string"}, "tolerance": {"type": "number", "minimum": 0.01, "maximum": 0.5}}, ["query"]),
    _tool("approach_object", "Track and approach an object until sonar reaches the target stand-off distance.", {"query": {"type": "string"}, "target_distance_cm": DISTANCE}, ["query"]),
    _tool("capture_image", "Capture and save a correctly rotated camera image.", {"filename": {"type": "string"}}),
    _tool("set_led_status", "Set RGB LEDs to a semantic rover status.", {"status": {"type": "string", "enum": ["off", "idle", "moving", "exploring", "waiting", "obstacle", "success", "error"]}}, ["status"]),
    _tool("find_marker", "Find an ArUco marker, optionally by numeric marker ID.", {"marker_id": {"type": "integer", "minimum": 0}, "search": {"type": "boolean"}}),
    _tool("go_to_marker", "Find, center, and approach an ArUco marker.", {"marker_id": {"type": "integer", "minimum": 0}, "target_distance_cm": DISTANCE}, ["marker_id"]),
    _tool("return_home", "Navigate back to the recorded home pose using current localization and obstacle checks.", {"tolerance_cm": DISTANCE}),
    _tool("wait_for_user", "Wait for terminal Enter or a supported rover switch, with optional timeout.", {"prompt": {"type": "string"}, "timeout_s": {"type": ["number", "null"], "minimum": 0}, "source": {"type": "string", "enum": ["terminal", "switch", "either"]}}),
]


def get_skill_schemas() -> list[dict[str, Any]]:
    return deepcopy(SKILL_SCHEMAS)


def get_skill_names() -> list[str]:
    return [tool["function"]["name"] for tool in SKILL_SCHEMAS]
