"""
Explore an area while building a lightweight occupancy map.

This is not full SLAM yet. It combines:
- simulator ground-truth pose when MARSY_MODE=sim;
- time-based dead reckoning on the real rover;
- mast ultrasonic scans;
- a sparse occupancy grid;
- a forward-first local controller with committed obstacle turns.

The occupancy map is currently observational: real-rover dead reckoning is not
yet accurate enough for map-frontier scores to steer every motion safely.

Simulator:
    python simulator/marsy_sim_ui.py
    MARSY_MODE=sim python -m missions.explore_area --steps 20

Real rover, conservative first test:
    MARSY_MODE=real python -m missions.explore_area \
        --steps 10 --step-distance 10 --speed 25 --safe-distance 45
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from behaviors.explore_navigation import (
    ExploreMotionState,
    choose_turn_direction,
    movement_made_progress,
)
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


def parse_angles(raw: str) -> list[float]:
    values = [float(part.strip()) for part in raw.split(",") if part.strip()]
    if not values:
        raise ValueError("At least one scan angle is required")
    return [max(-90.0, min(90.0, value)) for value in values]


LATEST_MAP_STEM = "explore_area_latest"


def clear_map_outputs(map_dir: Path) -> int:
    """Remove stale dashboard/exploration map exports from the map directory."""
    map_dir.mkdir(parents=True, exist_ok=True)
    removed = 0
    for path in map_dir.iterdir():
        if not path.is_file():
            continue
        if path.suffix.lower() not in {".json", ".svg", ".tmp"} and not path.name.endswith(".tmp"):
            continue
        path.unlink(missing_ok=True)
        removed += 1
    return removed


def save_map(skills: MarsySkills, map_dir: Path, step: int, final: bool = False) -> tuple[Path, Path]:
    """Overwrite the single current JSON/SVG map pair."""
    json_path = map_dir / f"{LATEST_MAP_STEM}.json"
    svg_path = map_dir / f"{LATEST_MAP_STEM}.svg"
    metadata = {
        "mission": "explore_area",
        "step": step,
        "final": final,
        "pose_source": skills.pose.source,
        "note": "SLAM-lite occupancy grid; no loop closure",
    }
    skills.map.save_json(json_path, current_pose=skills.pose, metadata=metadata)
    skills.map.save_svg(svg_path, current_pose=skills.pose)
    return json_path, svg_path


def scan_with_retry(
    skills: MarsySkills,
    scan_angles: list[float],
    *,
    retries: int = 2,
    offset_deg: float = 4.0,
) -> tuple[list[object], dict]:
    """Collect a full scan, retrying uncertain sectors without moving the rover."""
    result = skills.scan_arc(angles=scan_angles, samples=3)
    samples = list(result.data.get("samples", []))
    corridor = skills.assess_forward_corridor(samples)
    for attempt in range(max(0, int(retries))):
        if corridor.get("status") != "range_unknown":
            break
        sign = 1.0 if attempt % 2 == 0 else -1.0
        shifted = [max(-90.0, min(90.0, angle + sign * offset_deg)) for angle in scan_angles]
        log(
            f"Repeating unreliable scan with mast offset {sign * offset_deg:+.0f}°",
            source=SOURCE,
            level="warning",
            event="scan_retry",
        )
        result = skills.scan_arc(angles=shifted, samples=3)
        samples = list(result.data.get("samples", []))
        corridor = skills.assess_forward_corridor(samples)
    return samples, corridor


def _side_summary(evidence: object) -> dict:
    return {
        "trusted": getattr(evidence, "trusted_count", 0),
        "clear": getattr(evidence, "clear_count", 0),
        "median_cm": getattr(evidence, "median_clearance_cm", None),
        "minimum_cm": getattr(evidence, "minimum_clearance_cm", None),
    }


def main() -> None:
    args = build_parser().parse_args()
    scan_angles = parse_angles(args.scan_angles)
    rover = load_rover()
    motion = MarsyMotion(rover)
    sensors = MarsySensors(rover)
    config = SkillsConfig(
        safe_distance_cm=args.safe_distance,
        danger_distance_cm=args.danger_distance,
        max_range_cm=args.max_range,
        default_speed=args.speed,
    )
    occupancy_map = SparseOccupancyGrid(resolution_cm=args.resolution, max_range_cm=args.max_range)
    skills = MarsySkills(rover, motion=motion, sensors=sensors, occupancy_map=occupancy_map, config=config)

    map_dir = Path(args.map_dir)
    removed_maps = clear_map_outputs(map_dir)
    mission_started = time.time()
    completed_steps = 0

    try:
        rover.init(25)
        motion.stop_and_reset()
        skills.sync_pose()
        skills.home_pose = skills.pose.normalized()
        skills.set_led_status("exploring")
        if removed_maps:
            log(
                f"Removed {removed_maps} stale map files",
                source=SOURCE,
                event="stale_maps_removed",
                removed=removed_maps,
            )
        log(
            "Explore area started",
            source=SOURCE,
            event="mission_started",
            steps=args.steps,
            step_distance_cm=args.step_distance,
            safe_distance_cm=args.safe_distance,
            mapping_mode="slam_lite_no_loop_closure",
        )

        navigation = ExploreMotionState()
        max_actions = max(max(0, args.steps) * 6, max(0, args.steps) + 8)
        turn_step = max(15.0, min(45.0, abs(float(args.turn_step))))
        max_turns = max(2, int(args.max_turns_without_progress))

        def perform_committed_turn(samples: list[object], corridor: dict) -> bool:
            minimum = corridor.get("minimum_distance_cm")
            if (
                minimum is not None
                and float(minimum) < skills.config.danger_distance_cm
                and not navigation.reversed_for_current_obstacle
            ):
                reverse = skills.move_backward(distance_cm=5.0, speed=20)
                if reverse.ok:
                    navigation.note_reverse()

            sign, left, right, reason = choose_turn_direction(
                samples,
                safe_distance_cm=args.safe_distance,
                committed_sign=navigation.committed_turn_sign,
                default_sign=1,
            )
            committed = navigation.commit_turn(sign)
            angle = committed * turn_step
            log(
                f"Front unavailable ({corridor.get('status')}): committed turn {angle:+.0f}°",
                source=SOURCE,
                level="warning",
                event="committed_turn",
                decision_reason=reason,
                left=_side_summary(left),
                right=_side_summary(right),
                turns_without_progress=navigation.turns_without_progress,
            )
            rotation = skills.rotate(angle, speed=args.rotate_speed)
            if not rotation.ok:
                return False
            navigation.note_turn()
            if navigation.turns_without_progress >= max_turns:
                log(
                    "No forward progress after the committed turn limit; stopping instead of oscillating",
                    source=SOURCE,
                    level="warning",
                    event="turn_limit",
                    turns_without_progress=navigation.turns_without_progress,
                )
                return False
            return True

        while navigation.completed_moves < max(0, args.steps) and navigation.actions < max_actions:
            if simulator_stop_requested():
                log("Mission stopped from simulator UI", source=SOURCE, level="warning", event="ui_stop")
                break
            if args.run_seconds is not None and time.time() - mission_started >= args.run_seconds:
                log("Mission time limit reached", source=SOURCE, event="time_limit")
                break

            samples, corridor = scan_with_retry(skills, scan_angles)

            if corridor.get("safe"):
                movement = skills.move_forward(
                    distance_cm=args.step_distance,
                    speed=args.speed,
                    require_recent_scan=True,
                )
                if movement_made_progress(movement, args.step_distance):
                    navigation.note_progress()
                    completed_steps = navigation.completed_moves
                    log(
                        f"Forward progress {completed_steps}/{args.steps}",
                        source=SOURCE,
                        event="forward_progress",
                        completed_moves=completed_steps,
                        travelled_cm=movement.data.get("travelled_cm"),
                    )
                else:
                    navigation.note_failed_forward_action()
                    if movement.status == "range_unknown" and navigation.may_retry_same_heading(limit=1):
                        log(
                            "Forward range became uncertain; rescan the same heading before turning",
                            source=SOURCE,
                            level="warning",
                            event="same_heading_retry",
                            travelled_cm=movement.data.get("travelled_cm"),
                        )
                    else:
                        log(
                            f"Forward movement did not make enough progress ({movement.status}); "
                            "turn in the committed recovery direction",
                            source=SOURCE,
                            level="warning",
                            event="forward_progress_failed",
                            travelled_cm=movement.data.get("travelled_cm"),
                        )
                        failed_corridor = {
                            "safe": False,
                            "status": movement.status,
                            "minimum_distance_cm": movement.data.get("obstacle_cm")
                            or movement.data.get("distance_cm"),
                        }
                        if not perform_committed_turn(samples, failed_corridor):
                            break
            else:
                if corridor.get("status") == "range_unknown" and navigation.may_retry_same_heading(limit=1):
                    log(
                        "Front corridor is uncertain; hold heading and rescan once before turning",
                        source=SOURCE,
                        level="warning",
                        event="same_heading_retry",
                        unknown_angles=corridor.get("unknown_angles", []),
                    )
                else:
                    if not perform_committed_turn(samples, corridor):
                        break


            completed_steps = navigation.completed_moves
            json_path, svg_path = save_map(skills, map_dir, completed_steps)
            log(
                f"Map saved: {json_path} and {svg_path}",
                source=SOURCE,
                event="map_saved",
                step=completed_steps,
                actions=navigation.actions,
                json_path=str(json_path),
                svg_path=str(svg_path),
            )

        if args.return_home:
            log("Returning home", source=SOURCE, event="return_home_started")
            home_result = skills.return_home()
            log(home_result.message, source=SOURCE, event="return_home_finished", ok=home_result.ok)

        skills.set_led_status("success")

    except KeyboardInterrupt:
        log("Explore area interrupted", source=SOURCE, level="warning", event="mission_interrupted")
    finally:
        skills.stop(brake=False, reset_pose=True)
        final_json, final_svg = save_map(skills, map_dir, completed_steps, final=True)
        log(
            f"Final map: {final_json} and {final_svg}",
            source=SOURCE,
            event="final_map_saved",
            json_path=str(final_json),
            svg_path=str(final_svg),
        )
        try:
            rover.cleanup()
        except Exception as exc:
            log(f"rover.cleanup() failed: {exc}", source=SOURCE, level="warning", event="cleanup_failed")


if __name__ == "__main__":
    main()
