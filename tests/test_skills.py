from __future__ import annotations

import unittest

from skills import MarsySkills, SkillsConfig, SparseOccupancyGrid, get_skill_names
from skills.models import Pose2D


class FakeMotion:
    def __init__(self):
        self.commands = []

    def __getattr__(self, name):
        def command(*args, **kwargs):
            self.commands.append((name, args, kwargs))
        return command


class FakeSensors:
    def __init__(self, values=None):
        self.values = list(values or [100.0])
        self.index = 0

    def distance_cm(self):
        value = self.values[min(self.index, len(self.values) - 1)]
        self.index += 1
        return value


class FakeRover:
    numPixels = 4

    def __init__(self):
        self.led = None

    def fromRGB(self, red, green, blue):
        return (red, green, blue)

    def setColor(self, color):
        self.led = color

    def show(self):
        pass


class SkillsTests(unittest.TestCase):
    def make_skills(self, distances=None):
        return MarsySkills(
            FakeRover(),
            motion=FakeMotion(),
            sensors=FakeSensors(distances),
            config=SkillsConfig(
                full_speed_cm_s=100.0,
                full_spin_deg_s=360.0,
                safe_distance_cm=30.0,
                max_motion_s=1.0,
                motion_slice_s=0.001,
            ),
        )

    def test_registry_contains_requested_skills(self):
        requested = {
            "move_forward", "move_backward", "turn_left", "turn_right", "rotate", "stop",
            "scan_arc", "scan_360", "measure_distance", "avoid_obstacle", "search_visual",
            "track_object", "approach_object", "capture_image", "set_led_status", "find_marker",
            "go_to_marker", "return_home", "wait_for_user",
        }
        self.assertTrue(requested.issubset(set(get_skill_names())))

    def test_forward_is_blocked(self):
        skills = self.make_skills([20.0])
        result = skills.move_forward(distance_cm=10, speed=50)
        self.assertFalse(result.ok)
        self.assertEqual(result.status, "blocked")

    def test_rotate_updates_dead_reckoning(self):
        skills = self.make_skills([100.0])
        result = skills.rotate(90, speed=100)
        self.assertTrue(result.ok)
        self.assertAlmostEqual(skills.pose.heading_deg, 90.0, delta=3.0)

    def test_map_marks_hit(self):
        grid = SparseOccupancyGrid(resolution_cm=5.0, max_range_cm=100.0, sensor_offset_cm=0.0)
        pose = Pose2D(0, 0, 0)
        grid.observe_ray(pose, 0, 20, valid_hit=True)
        self.assertEqual(grid.state(grid.world_to_cell(0, 20)), "occupied")
        self.assertEqual(grid.state(grid.world_to_cell(0, 10)), "free")

    def test_led_status(self):
        rover = FakeRover()
        skills = MarsySkills(rover, motion=FakeMotion(), sensors=FakeSensors())
        result = skills.set_led_status("success")
        self.assertTrue(result.ok)
        self.assertEqual(rover.led, (0, 220, 70))


if __name__ == "__main__":
    unittest.main()
