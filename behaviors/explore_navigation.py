"""Decision memory for stable local exploration.

The SLAM-lite map is useful for visualisation, but the real rover does not yet
have accurate enough localisation for map-frontier scores to steer every
motion.  This module therefore keeps navigation local and stateful:

* drive straight whenever the freshly scanned front corridor is safe;
* choose a turn only when forward motion is unavailable;
* keep the chosen turn direction until a real forward step succeeds;
* do not count a tiny aborted movement as progress.

The turn commitment prevents the left/right oscillation caused by selecting a
new best sonar sector after every small rotation.
"""

from __future__ import annotations

from dataclasses import dataclass
from statistics import median
from typing import Any, Iterable, Optional

LEFT = -1
RIGHT = 1


def _value(sample: Any, name: str, default: Any = None) -> Any:
    if isinstance(sample, dict):
        return sample.get(name, default)
    return getattr(sample, name, default)


def _trusted(sample: Any) -> bool:
    quality = str(_value(sample, "quality", "unknown"))
    distance = _value(sample, "distance_cm")
    return quality == "measured" and distance is not None


@dataclass(frozen=True)
class SideEvidence:
    """Robust evidence for one side of the rover."""

    sign: int
    trusted_count: int
    clear_count: int
    median_clearance_cm: Optional[float]
    minimum_clearance_cm: Optional[float]

    @property
    def rank(self) -> tuple[int, int, float, float]:
        """Comparison key that favours several clear rays over one far outlier."""
        median_value = -1.0 if self.median_clearance_cm is None else self.median_clearance_cm
        minimum_value = -1.0 if self.minimum_clearance_cm is None else self.minimum_clearance_cm
        return (self.clear_count, self.trusted_count, median_value, minimum_value)


def side_evidence(samples: Iterable[Any], sign: int, safe_distance_cm: float) -> SideEvidence:
    sign = LEFT if sign < 0 else RIGHT
    distances: list[float] = []
    for sample in samples:
        if not _trusted(sample):
            continue
        angle = float(_value(sample, "mast_angle_deg", 0.0))
        if abs(angle) < 12.0:
            continue
        if sign == LEFT and angle >= 0:
            continue
        if sign == RIGHT and angle <= 0:
            continue
        try:
            distances.append(float(_value(sample, "distance_cm")))
        except (TypeError, ValueError):
            continue

    if not distances:
        return SideEvidence(sign, 0, 0, None, None)
    return SideEvidence(
        sign=sign,
        trusted_count=len(distances),
        clear_count=sum(distance >= float(safe_distance_cm) for distance in distances),
        median_clearance_cm=float(median(distances)),
        minimum_clearance_cm=min(distances),
    )


def choose_turn_direction(
    samples: Iterable[Any],
    *,
    safe_distance_cm: float,
    committed_sign: int = 0,
    default_sign: int = RIGHT,
) -> tuple[int, SideEvidence, SideEvidence, str]:
    """Choose a turn direction while preserving an existing commitment."""
    left = side_evidence(samples, LEFT, safe_distance_cm)
    right = side_evidence(samples, RIGHT, safe_distance_cm)

    if committed_sign in {LEFT, RIGHT}:
        return committed_sign, left, right, "committed"
    if left.rank > right.rank:
        return LEFT, left, right, "left_has_better_side_evidence"
    if right.rank > left.rank:
        return RIGHT, left, right, "right_has_better_side_evidence"
    chosen = LEFT if default_sign < 0 else RIGHT
    return chosen, left, right, "tie_default"


def movement_made_progress(result: Any, requested_distance_cm: float, fraction: float = 0.60) -> bool:
    """Return true only for a successful, substantial forward movement."""
    if not bool(getattr(result, "ok", False)):
        return False
    data = getattr(result, "data", {}) or {}
    try:
        travelled = float(data.get("travelled_cm", 0.0))
    except (TypeError, ValueError):
        travelled = 0.0
    try:
        commanded = float(data.get("commanded_distance_cm", requested_distance_cm))
    except (TypeError, ValueError):
        commanded = float(requested_distance_cm)
    threshold = max(1.0, commanded * max(0.1, min(1.0, fraction)))
    return travelled >= threshold


@dataclass
class ExploreMotionState:
    """Small state machine that prevents turn reversals without progress."""

    completed_moves: int = 0
    actions: int = 0
    committed_turn_sign: int = 0
    turns_without_progress: int = 0
    uncertainty_retries: int = 0
    reversed_for_current_obstacle: bool = False

    def may_retry_same_heading(self, limit: int = 1) -> bool:
        if self.uncertainty_retries >= max(0, int(limit)):
            return False
        self.uncertainty_retries += 1
        return True

    def commit_turn(self, sign: int) -> int:
        if self.committed_turn_sign not in {LEFT, RIGHT}:
            self.committed_turn_sign = LEFT if sign < 0 else RIGHT
        return self.committed_turn_sign

    def note_turn(self) -> None:
        self.actions += 1
        self.turns_without_progress += 1
        self.uncertainty_retries = 0

    def note_reverse(self) -> None:
        self.actions += 1
        self.reversed_for_current_obstacle = True

    def note_progress(self) -> None:
        self.actions += 1
        self.completed_moves += 1
        self.committed_turn_sign = 0
        self.turns_without_progress = 0
        self.uncertainty_retries = 0
        self.reversed_for_current_obstacle = False

    def note_failed_forward_action(self) -> None:
        self.actions += 1
