"""Shared data models for Marsy skills."""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field, is_dataclass
from pathlib import Path
from typing import Any, Optional


def normalize_heading(degrees: float) -> float:
    """Normalize heading to [-180, 180). Positive angles turn right/clockwise."""
    return (float(degrees) + 180.0) % 360.0 - 180.0


@dataclass
class Pose2D:
    """Marsy pose in centimeters. Heading 0 points toward +Y; positive is right."""

    x_cm: float = 0.0
    y_cm: float = 0.0
    heading_deg: float = 0.0
    source: str = "dead_reckoning"
    timestamp: float = field(default_factory=time.time)

    def normalized(self) -> "Pose2D":
        return Pose2D(
            x_cm=float(self.x_cm),
            y_cm=float(self.y_cm),
            heading_deg=normalize_heading(self.heading_deg),
            source=self.source,
            timestamp=self.timestamp,
        )

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(other.x_cm - self.x_cm, other.y_cm - self.y_cm)


@dataclass
class RangeSample:
    mast_angle_deg: float
    global_angle_deg: float
    distance_cm: Optional[float]
    valid_hit: bool
    no_echo: bool
    pose: Pose2D
    quality: str = "measured"
    readings: list[float] = field(default_factory=list)
    spread_cm: Optional[float] = None
    timestamp: float = field(default_factory=time.time)

    @property
    def trusted(self) -> bool:
        return self.quality == "measured" and self.valid_hit and self.distance_cm is not None


@dataclass
class Detection:
    """Normalized object detection. bbox is [x_min, y_min, x_max, y_max] in 0..1."""

    label: str
    confidence: float = 0.0
    bbox: Optional[list[float]] = None
    marker_id: Optional[int] = None
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def center_x(self) -> Optional[float]:
        if not self.bbox or len(self.bbox) != 4:
            return None
        return (float(self.bbox[0]) + float(self.bbox[2])) / 2.0

    @property
    def center_y(self) -> Optional[float]:
        if not self.bbox or len(self.bbox) != 4:
            return None
        return (float(self.bbox[1]) + float(self.bbox[3])) / 2.0

    @property
    def area(self) -> Optional[float]:
        if not self.bbox or len(self.bbox) != 4:
            return None
        width = max(0.0, float(self.bbox[2]) - float(self.bbox[0]))
        height = max(0.0, float(self.bbox[3]) - float(self.bbox[1]))
        return width * height

    @classmethod
    def from_mapping(cls, value: dict[str, Any]) -> "Detection":
        bbox = value.get("bbox")
        if bbox is not None:
            bbox = [float(v) for v in bbox]
        marker_id = value.get("marker_id")
        return cls(
            label=str(value.get("label", "object")),
            confidence=float(value.get("confidence", 0.0)),
            bbox=bbox,
            marker_id=None if marker_id is None else int(marker_id),
            metadata=dict(value.get("metadata") or {}),
        )


def _jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return {key: _jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


@dataclass
class SkillResult:
    skill: str
    ok: bool
    status: str
    message: str = ""
    data: dict[str, Any] = field(default_factory=dict)
    started_at: float = field(default_factory=time.time)
    finished_at: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return _jsonable(self)
