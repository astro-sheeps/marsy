from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from dataclasses import dataclass, field

from behaviors.explore_area import ExploreAreaConfig, run_explore_area
from marsy_agent.cache import PlanCache
from marsy_agent.executor import AgentExecutor
from marsy_agent.journal import AgentJournal
from marsy_agent.models import AgentPlan
from marsy_agent.planner import GroqPlanner, MockPlanner
from marsy_agent.specs import SKILL_SPECS
from marsy_agent.validator import PlanValidationError, PlanValidator


@dataclass
class FakeResult:
    skill: str
    ok: bool
    status: str
    message: str = ""
    data: dict = field(default_factory=dict)

    def to_dict(self):
        return {
            "skill": self.skill,
            "ok": self.ok,
            "status": self.status,
            "message": self.message,
            "data": self.data,
        }


class FakeSkills:
    def __init__(self):
        self.calls = []
        self.move_attempts = 0

    def call(self, skill, **arguments):
        self.calls.append((skill, arguments))
        if skill == "move_forward":
            self.move_attempts += 1
            if self.move_attempts == 1:
                return FakeResult(skill, False, "blocked", "obstacle")
            return FakeResult(skill, True, "completed", data={"travelled_cm": 6})
        return FakeResult(skill, True, "completed")

    def set_led_status(self, status):
        self.calls.append(("set_led_status", {"status": status}))
        return FakeResult("set_led_status", True, "completed")

    def stop(self, brake=False, reset_pose=True):
        self.calls.append(("stop", {"brake": brake, "reset_pose": reset_pose}))
        return FakeResult("stop", True, "completed")




@dataclass
class FakeRangeSample:
    mast_angle_deg: float
    distance_cm: float | None
    quality: str = "measured"
    readings: list[float] = field(default_factory=list)

    @property
    def trusted(self):
        return self.quality == "measured" and self.distance_cm is not None


class FakeExploreSkills:
    def __init__(self):
        self.config = type("Config", (), {"danger_distance_cm": 24.0})()
        self.moves = 0

    def scan_arc(self, **kwargs):
        samples = [
            FakeRangeSample(-25.0, 100.0, readings=[100.0]),
            FakeRangeSample(0.0, 100.0, readings=[100.0]),
            FakeRangeSample(25.0, 100.0, readings=[100.0]),
        ]
        return FakeResult("scan_arc", True, "completed", data={"samples": samples})

    def assess_forward_corridor(self, samples):
        return {
            "safe": True,
            "status": "clear",
            "minimum_distance_cm": 100.0,
            "unknown_angles": [],
        }

    def move_forward(self, distance_cm, speed, require_recent_scan):
        self.moves += 1
        return FakeResult(
            "move_forward",
            True,
            "completed",
            data={
                "travelled_cm": distance_cm,
                "commanded_distance_cm": distance_cm,
            },
        )

    def move_backward(self, **kwargs):
        return FakeResult("move_backward", True, "completed")

    def rotate(self, *args, **kwargs):
        return FakeResult("rotate", True, "completed")


class FakeDirectionSkills:
    def __init__(self, scans):
        self.config = type(
            "Config",
            (),
            {"safe_distance_cm": 40.0, "danger_distance_cm": 24.0},
        )()
        self.scans = list(scans)
        self.scan_index = 0
        self.rotations = []

    def scan_arc(self, **kwargs):
        del kwargs
        samples = self.scans[self.scan_index]
        self.scan_index += 1
        return FakeResult("scan_arc", True, "completed", data={"samples": samples})

    def assess_forward_corridor(self, samples):
        center = [
            sample.distance_cm
            for sample in samples
            if abs(sample.mast_angle_deg) <= 12
            and sample.trusted
            and sample.distance_cm is not None
        ]
        minimum = min(center) if center else None
        safe = minimum is not None and minimum >= self.config.safe_distance_cm
        return {
            "safe": safe,
            "status": "clear" if safe else "blocked",
            "message": (
                f"Front corridor is clear at {minimum:.1f} cm"
                if safe
                else f"Front corridor is blocked at {minimum:.1f} cm"
            ),
            "minimum_distance_cm": minimum,
        }

    def rotate(self, angle_deg, speed):
        self.rotations.append((float(angle_deg), int(speed)))
        return FakeResult(
            "rotate",
            True,
            "completed",
            data={"angle_deg": float(angle_deg)},
        )


def direction_scan(*, center, left=111.0, right=86.0):
    return [
        FakeRangeSample(-75.0, None, quality="no_echo"),
        FakeRangeSample(-50.0, left, readings=[left]),
        FakeRangeSample(-25.0, 27.0, readings=[27.0]),
        FakeRangeSample(0.0, center, readings=[center]),
        FakeRangeSample(25.0, 27.0, readings=[27.0]),
        FakeRangeSample(50.0, right, readings=[right]),
        FakeRangeSample(75.0, None, quality="no_echo"),
    ]


class AgentMissionTests(unittest.TestCase):
    def setUp(self):
        self.validator = PlanValidator()

    def test_mock_plan_is_valid(self):
        plan = MockPlanner().create_plan("Explore")
        self.assertIs(self.validator.validate(plan), plan)


    def test_led_skill_is_not_exposed_to_agent(self):
        self.assertNotIn("set_led_status", SKILL_SPECS)
        plan = MockPlanner().create_plan("Explore")
        self.assertNotIn("set_led_status", [step.skill for step in plan.steps])

    def test_executor_honours_dashboard_stop_before_first_step(self):
        skills = FakeSkills()
        skills._external_stop_requested = lambda: True
        progress = []
        plan = MockPlanner().create_plan("Explore")
        executor = AgentExecutor(
            skills=skills,
            planner=MockPlanner(),
            validator=self.validator,
            progress_callback=progress.append,
        )
        report = executor.execute(goal="Explore", plan=plan)
        self.assertFalse(report.ok)
        self.assertEqual(report.status, "stopped")
        self.assertFalse(any(name == "set_led_status" for name, _ in skills.calls))
        self.assertEqual(progress[-1]["phase"], "finished")

    def test_unsafe_speed_is_rejected(self):
        plan = AgentPlan.from_dict(
            {
                "version": 1,
                "goal": "Move",
                "steps": [
                    {
                        "id": "move",
                        "skill": "move_forward",
                        "arguments": {
                            "distance_cm": 6,
                            "speed": 80,
                            "require_recent_scan": True,
                        },
                    },
                    {
                        "id": "stop",
                        "skill": "stop",
                        "arguments": {"brake": False, "reset_pose": True},
                    },
                ],
            }
        )
        with self.assertRaises(PlanValidationError):
            self.validator.validate(plan)

    def test_stop_must_be_final(self):
        plan = AgentPlan.from_dict(
            {
                "version": 1,
                "goal": "Bad plan",
                "steps": [
                    {
                        "id": "stop",
                        "skill": "stop",
                        "arguments": {"brake": False, "reset_pose": True},
                    },
                    {
                        "id": "wait",
                        "skill": "wait",
                        "arguments": {"duration_seconds": 1},
                    },
                ],
            }
        )
        with self.assertRaises(PlanValidationError):
            self.validator.validate(plan)

    def test_shared_explore_runner_completes_one_step(self):
        skills = FakeExploreSkills()
        result = run_explore_area(
            skills,
            ExploreAreaConfig(
                steps=1,
                run_seconds=10,
                step_distance_cm=6,
                speed=22,
                save_maps=False,
                clear_existing_maps=False,
            ),
            stop_requested=lambda: False,
        )
        self.assertTrue(result.ok)
        self.assertEqual(result.completed_moves, 1)
        self.assertEqual(skills.moves, 1)

    def test_turn_to_clear_direction_keeps_committed_direction_until_clear(self):
        skills = FakeDirectionSkills(
            [
                direction_scan(center=33.0, left=111.0, right=60.0),
                direction_scan(center=27.0),
                direction_scan(center=80.0),
            ]
        )
        executor = AgentExecutor(
            skills=skills,
            planner=MockPlanner(),
            validator=self.validator,
        )
        result = executor._turn_to_clear_direction(rotate_speed=15)
        self.assertTrue(result.ok)
        self.assertEqual(skills.rotations, [(-30.0, 15), (-30.0, 15)])
        self.assertEqual(result.data["angle_deg"], -60.0)
        self.assertEqual(result.data["direction"], "left")
        self.assertEqual(len(result.data["attempts"]), 2)

    def test_turn_to_clear_direction_can_commit_right(self):
        skills = FakeDirectionSkills(
            [
                direction_scan(center=33.0, left=55.0, right=115.0),
                direction_scan(center=80.0),
            ]
        )
        executor = AgentExecutor(
            skills=skills,
            planner=MockPlanner(),
            validator=self.validator,
        )
        result = executor._turn_to_clear_direction(rotate_speed=15)
        self.assertTrue(result.ok)
        self.assertEqual(skills.rotations, [(30.0, 15)])
        self.assertEqual(result.data["angle_deg"], 30.0)
        self.assertEqual(result.data["direction"], "right")

    def test_turn_to_clear_direction_stops_after_bounded_attempts(self):
        skills = FakeDirectionSkills(
            [direction_scan(center=20.0, left=111.0, right=60.0)]
            + [direction_scan(center=25.0) for _ in range(4)]
        )
        executor = AgentExecutor(
            skills=skills,
            planner=MockPlanner(),
            validator=self.validator,
        )
        result = executor._turn_to_clear_direction(rotate_speed=15)
        self.assertFalse(result.ok)
        self.assertEqual(skills.rotations, [(-30.0, 15)] * 4)
        self.assertEqual(result.data["angle_deg"], -120.0)
        self.assertEqual(len(result.data["attempts"]), 4)

    def test_llm_cannot_supply_turn_angle(self):
        plan = AgentPlan.from_dict(
            {
                "version": 1,
                "goal": "Choose a direction",
                "steps": [
                    {
                        "id": "turn",
                        "skill": "turn_to_clear_direction",
                        "arguments": {
                            "turn_angle_deg": 45,
                            "rotate_speed": 15,
                        },
                    },
                    {
                        "id": "stop",
                        "skill": "stop",
                        "arguments": {"brake": False, "reset_pose": True},
                    },
                ],
            }
        )
        with self.assertRaises(PlanValidationError):
            self.validator.validate(plan)

    def test_executor_replans_once_after_blocked_move(self):
        planner = MockPlanner()
        initial = AgentPlan.from_dict(
            {
                "version": 1,
                "goal": "Move safely",
                "steps": [
                    {
                        "id": "move",
                        "skill": "move_forward",
                        "arguments": {
                            "distance_cm": 6,
                            "speed": 22,
                            "require_recent_scan": True,
                        },
                    },
                    {
                        "id": "stop",
                        "skill": "stop",
                        "arguments": {"brake": False, "reset_pose": True},
                    },
                ],
            }
        )
        skills = FakeSkills()
        progress = []
        executor = AgentExecutor(
            skills=skills,
            planner=planner,
            validator=self.validator,
            max_replans=1,
            progress_callback=progress.append,
        )
        # Avoid testing sonar direction selection here; replace the composite
        # with a successful deterministic result.
        executor._turn_to_clear_direction = lambda **kwargs: FakeResult(
            "turn_to_clear_direction", True, "completed"
        )
        report = executor.execute(goal="Move safely", plan=initial)
        self.assertTrue(report.ok)
        self.assertEqual(len(report.replans), 1)
        self.assertEqual(skills.move_attempts, 2)
        serialized = report.to_dict()
        self.assertEqual(serialized["status"], "completed")
        self.assertTrue(serialized["step_records"])
        replanned = next(item for item in progress if item["phase"] == "replanned")
        self.assertEqual(replanned["plan_source"], "replan")
        self.assertEqual(replanned["plan"], report.replans[0].to_dict())


    def test_groq_planner_does_not_require_key_until_request(self):
        planner = GroqPlanner(model="openai/gpt-oss-20b", api_key=None)
        self.assertEqual(planner.model, "openai/gpt-oss-20b")

    def test_plan_cache_reuses_only_exact_goal_and_model(self):
        plan = MockPlanner().create_plan("Explore")
        with tempfile.TemporaryDirectory() as directory:
            cache = PlanCache(directory=directory)
            cache.save(
                goal="Explore the room",
                planner="groq",
                model="openai/gpt-oss-20b",
                plan=plan,
            )
            hit = cache.load(
                goal="Explore   the room",
                planner="groq",
                model="openai/gpt-oss-20b",
            )
            self.assertIsNotNone(hit)
            self.assertEqual(hit.to_dict(), plan.to_dict())
            self.assertIsNone(
                cache.load(
                    goal="Explore another room",
                    planner="groq",
                    model="openai/gpt-oss-20b",
                )
            )
            self.assertIsNone(
                cache.load(
                    goal="Explore the room",
                    planner="groq",
                    model="openai/gpt-oss-120b",
                )
            )
            self.assertEqual(
                [item.name for item in Path(directory).glob("*.json")],
                ["agent_plan_latest.json"],
            )

    def test_journal_keeps_only_latest_json(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "agent_20260723_120000_deadbeef.json").write_text("{}")
            (root / "agent_20260723_130000_cafebabe.json").write_text("{}")
            journal = AgentJournal(
                goal="Explore",
                planner="groq",
                model="openai/gpt-oss-20b",
                directory=root,
            )
            journal.set_plan_source("cache")
            journal.set_initial_plan(MockPlanner().create_plan("Explore"))
            self.assertEqual(journal.path.name, "agent_mission_latest.json")
            self.assertEqual(
                [item.name for item in root.glob("*.json")],
                ["agent_mission_latest.json"],
            )


if __name__ == "__main__":
    unittest.main()
