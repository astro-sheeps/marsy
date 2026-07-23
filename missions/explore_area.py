"""
Explore an area while building a lightweight occupancy map.

The reusable navigation loop lives in ``behaviors.explore_area`` so that the
standalone mission and the LLM agent execute exactly the same controller.

Simulator:
    python simulator/marsy_sim_ui.py
    MARSY_MODE=sim python -m missions.explore_area --steps 20

Real rover, conservative first test:
    MARSY_MODE=real python -m missions.explore_area \
        --steps 10 --step-distance 10 --speed 25 --safe-distance 45
"""

from __future__ import annotations

import argparse

from behaviors.explore_area import ExploreAreaConfig, parse_scan_angles, run_explore_area
from marsy_backends.loader import load_rover
from marsy_core.motion import MarsyMotion
from marsy_core.sensors import MarsySensors
from marsy_core.telemetry import log, simulator_stop_requested
from skills import MarsySkills, SkillsConfig, SparseOccupancyGrid

SOURCE = "explore_area"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Marsy area exploration with SLAM-lite mapping")
    parser.add_argument("--steps", type=int, default=20, help="Maximum exploration movement steps")
    parser.add_argument("--run-seconds", type=float, default=None, help="Optional mission time limit")
    parser.add_argument("--step-distance", type=float, default=12.0, help="Forward distance per exploration step, cm")
    parser.add_argument("--speed", type=int, default=30, help="Forward speed 0-100")
    parser.add_argument("--rotate-speed", type=int, default=25, help="In-place rotation speed 0-100")
    parser.add_argument("--turn-step", type=float, default=30.0, help="Committed recovery turn increment, degrees")
    parser.add_argument(
        "--max-turns-without-progress",
        type=int,
        default=4,
        help="Stop instead of spinning forever after this many committed turns",
    )
    parser.add_argument("--safe-distance", type=float, default=40.0, help="Minimum clear distance for forward motion, cm")
    parser.add_argument("--danger-distance", type=float, default=24.0, help="Distance considered dangerously close, cm")
    parser.add_argument("--max-range", type=float, default=200.0, help="Maximum mapped sonar range, cm")
    parser.add_argument("--resolution", type=float, default=5.0, help="Map cell size, cm")
    parser.add_argument("--scan-angles", default="0,-25,25,-50,50,-75,75", help="Comma-separated mast angles")
    parser.add_argument("--map-dir", default="artifacts/maps", help="Directory for JSON and SVG maps")
    parser.add_argument("--return-home", action="store_true", help="Attempt dead-reckoning return to the start pose")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    rover = load_rover()
    motion = MarsyMotion(rover)
    sensors = MarsySensors(rover)
    skills_config = SkillsConfig(
        safe_distance_cm=args.safe_distance,
        danger_distance_cm=args.danger_distance,
        max_range_cm=args.max_range,
        default_speed=args.speed,
    )
    occupancy_map = SparseOccupancyGrid(resolution_cm=args.resolution, max_range_cm=args.max_range)
    skills = MarsySkills(
        rover,
        motion=motion,
        sensors=sensors,
        occupancy_map=occupancy_map,
        config=skills_config,
    )
    behavior_config = ExploreAreaConfig(
        steps=args.steps,
        run_seconds=args.run_seconds,
        step_distance_cm=args.step_distance,
        speed=args.speed,
        rotate_speed=args.rotate_speed,
        turn_step_deg=args.turn_step,
        max_turns_without_progress=args.max_turns_without_progress,
        safe_distance_cm=args.safe_distance,
        scan_angles=parse_scan_angles(args.scan_angles),
        map_dir=args.map_dir,
        return_home=args.return_home,
        clear_existing_maps=True,
        save_maps=True,
        source=SOURCE,
        map_mission=SOURCE,
    )

    try:
        rover.init(0)
        motion.stop_and_reset()
        skills.sync_pose()
        skills.home_pose = skills.pose.normalized()
        result = run_explore_area(
            skills,
            behavior_config,
            stop_requested=simulator_stop_requested,
        )
        log(
            result.message,
            source=SOURCE,
            level="info" if result.ok else "warning",
            event="mission_finished",
            status=result.status,
            completed_moves=result.completed_moves,
            actions=result.actions,
            elapsed_seconds=result.elapsed_seconds,
            map_json=result.map_json,
            map_svg=result.map_svg,
        )
    except KeyboardInterrupt:
        log("Explore area interrupted", source=SOURCE, level="warning", event="mission_interrupted")
    finally:
        skills.stop(brake=False, reset_pose=True)
        try:
            rover.cleanup()
        except Exception as exc:
            log(
                f"rover.cleanup() failed: {exc}",
                source=SOURCE,
                level="warning",
                event="cleanup_failed",
            )


if __name__ == "__main__":
    main()
