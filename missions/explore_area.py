"""
Explore an area while building a lightweight occupancy map.

This is not full SLAM yet. It combines:
- simulator ground-truth pose when MARSY_MODE=sim;
- time-based dead reckoning on the real rover;
- mast ultrasonic scans;
- a sparse occupancy grid;
- a simple frontier-like direction score.

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


def _sample_quality(sample: object) -> str:
    if isinstance(sample, dict):
        return str(sample.get("quality", "measured" if sample.get("valid_hit") else "unknown"))
    return str(getattr(sample, "quality", "measured" if getattr(sample, "valid_hit", False) else "unknown"))


def _sample_value(sample: object, name: str):
    return sample.get(name) if isinstance(sample, dict) else getattr(sample, name)


def choose_direction(skills: MarsySkills, samples: list[object], safe_distance_cm: float) -> tuple[float, float, float] | None:
    candidates: list[tuple[float, float, float]] = []
    for sample in samples:
        if _sample_quality(sample) != "measured":
            continue
        angle = float(_sample_value(sample, "mast_angle_deg"))
        distance = _sample_value(sample, "distance_cm")
        if distance is None:
            continue
        clearance = float(distance)
        if clearance < safe_distance_cm:
            continue
        score = skills.map.direction_score(skills.pose, angle, clearance)
        candidates.append((score, angle, clearance))
    if not candidates:
        return None
    return max(candidates, key=lambda item: item[0])


def scan_with_retry(
    skills: MarsySkills,
    scan_angles: list[float],
    *,
    retries: int = 1,
    offset_deg: float = 4.0,
) -> tuple[list[object], dict]:
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


def recovery_turn_angle(samples: list[object], step: int) -> float:
    trusted: list[tuple[float, float]] = []
    for sample in samples:
        if _sample_quality(sample) != "measured":
            continue
        angle = float(_sample_value(sample, "mast_angle_deg"))
        distance = _sample_value(sample, "distance_cm")
        if distance is None or abs(angle) < 8.0:
            continue
        trusted.append((float(distance), angle))
    if trusted:
        _, best_angle = max(trusted, key=lambda item: item[0])
        return max(-55.0, min(55.0, best_angle))
    return 35.0 if step % 2 == 0 else -35.0


def recover_without_forward_motion(
    skills: MarsySkills,
    samples: list[object],
    corridor: dict,
    *,
    step: int,
    rotate_speed: int,
) -> bool:
    skills.stop(brake=True, reset_pose=False)
    minimum = corridor.get("minimum_distance_cm")
    if minimum is not None and float(minimum) < skills.config.danger_distance_cm:
        skills.move_backward(distance_cm=5.0, speed=20)
    angle = recovery_turn_angle(samples, step)
    log(
        f"Unsafe or unknown corridor; rotate {angle:+.0f}° without moving forward",
        source=SOURCE,
        level="warning",
        event="safe_recovery",
        corridor_status=corridor.get("status"),
        unknown_angles=corridor.get("unknown_angles", []),
    )
    result = skills.rotate(angle, speed=rotate_speed)
    return result.ok


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

        for step in range(max(0, args.steps)):
            if simulator_stop_requested():
                log("Mission stopped from simulator UI", source=SOURCE, level="warning", event="ui_stop")
                break
            if args.run_seconds is not None and time.time() - mission_started >= args.run_seconds:
                log("Mission time limit reached", source=SOURCE, event="time_limit")
                break

            samples, corridor = scan_with_retry(skills, scan_angles)
            choice = choose_direction(skills, samples, args.safe_distance)

            if choice is None:
                skills.set_led_status("obstacle")
                if not recover_without_forward_motion(
                    skills, samples, corridor, step=step, rotate_speed=args.rotate_speed
                ):
                    break
                skills.set_led_status("exploring")
            else:
                score, angle, clearance = choice
                log(
                    f"Step {step}: choose {angle:+.0f}°; clearance={clearance:.1f} cm; score={score:.1f}",
                    source=SOURCE,
                    event="direction_selected",
                    step=step,
                    angle_deg=angle,
                    clearance_cm=clearance,
                    score=score,
                )
                if abs(angle) >= 8.0:
                    rotation = skills.rotate(angle, speed=args.rotate_speed)
                    if not rotation.ok:
                        break
                    # A scan made before rotation is no longer aligned with the
                    # rover. Always scan again before any forward movement.
                    samples, corridor = scan_with_retry(skills, scan_angles)

                if not corridor.get("safe"):
                    skills.set_led_status("obstacle")
                    if not recover_without_forward_motion(
                        skills, samples, corridor, step=step, rotate_speed=args.rotate_speed
                    ):
                        break
                    skills.set_led_status("exploring")
                else:
                    movement = skills.move_forward(
                        distance_cm=args.step_distance,
                        speed=args.speed,
                        require_recent_scan=True,
                    )
                    if not movement.ok:
                        skills.set_led_status("obstacle")
                        fallback_corridor = skills.assess_forward_corridor()
                        if not recover_without_forward_motion(
                            skills, samples, fallback_corridor, step=step, rotate_speed=args.rotate_speed
                        ):
                            break
                        skills.set_led_status("exploring")

            completed_steps = step + 1
            json_path, svg_path = save_map(skills, map_dir, completed_steps)
            log(
                f"Map saved: {json_path} and {svg_path}",
                source=SOURCE,
                event="map_saved",
                step=completed_steps,
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
