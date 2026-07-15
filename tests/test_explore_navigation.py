from __future__ import annotations

import unittest
from dataclasses import dataclass

from behaviors.explore_navigation import (
    ExploreMotionState,
    LEFT,
    RIGHT,
    choose_turn_direction,
    movement_made_progress,
)


@dataclass
class Sample:
    mast_angle_deg: float
    distance_cm: float | None
    quality: str = "measured"


@dataclass
class Result:
    ok: bool
    data: dict


class ExploreNavigationTests(unittest.TestCase):
    def test_clearer_side_is_selected_robustly(self) -> None:
        samples = [
            Sample(-75, 160),
            Sample(-50, 25),
            Sample(-25, 25),
            Sample(25, 80),
            Sample(50, 90),
            Sample(75, 100),
        ]
        sign, left, right, _ = choose_turn_direction(
            samples, safe_distance_cm=45, committed_sign=0
        )
        self.assertEqual(sign, RIGHT)
        self.assertEqual(left.clear_count, 1)
        self.assertEqual(right.clear_count, 3)

    def test_commitment_prevents_turning_back(self) -> None:
        samples = [
            Sample(-75, 200),
            Sample(-50, 190),
            Sample(-25, 180),
            Sample(25, 30),
            Sample(50, 30),
            Sample(75, 30),
        ]
        sign, _, _, reason = choose_turn_direction(
            samples,
            safe_distance_cm=45,
            committed_sign=RIGHT,
        )
        self.assertEqual(sign, RIGHT)
        self.assertEqual(reason, "committed")

    def test_progress_resets_commitment(self) -> None:
        state = ExploreMotionState()
        state.commit_turn(LEFT)
        state.note_turn()
        state.note_progress()
        self.assertEqual(state.completed_moves, 1)
        self.assertEqual(state.committed_turn_sign, 0)
        self.assertEqual(state.turns_without_progress, 0)

    def test_tiny_motion_is_not_progress(self) -> None:
        self.assertFalse(
            movement_made_progress(Result(True, {"travelled_cm": 1.0}), 10.0)
        )
        self.assertTrue(
            movement_made_progress(Result(True, {"travelled_cm": 7.0}), 10.0)
        )


    def test_cautious_commanded_distance_counts_as_progress(self) -> None:
        result = Result(
            True,
            {
                "travelled_cm": 3.0,
                "commanded_distance_cm": 4.0,
            },
        )
        self.assertTrue(movement_made_progress(result, 12.0))

    def test_one_same_heading_retry_only(self) -> None:
        state = ExploreMotionState()
        self.assertTrue(state.may_retry_same_heading(limit=1))
        self.assertFalse(state.may_retry_same_heading(limit=1))
        state.note_turn()
        self.assertTrue(state.may_retry_same_heading(limit=1))


if __name__ == "__main__":
    unittest.main()
