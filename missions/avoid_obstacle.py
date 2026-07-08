"""
Mission runner for Marsy obstacle avoidance.

Project location:
    missions/avoid_obstacle.py

Run in simulator:
    python simulator/marsy_sim_ui.py
    MARSY_MODE=sim python -m missions.avoid_obstacle

Run on real rover:
    MARSY_MODE=real python -m missions.avoid_obstacle

Conservative real-rover test:
    MARSY_MODE=real python -m missions.avoid_obstacle \
        --forward-speed 25 \
        --turn-speed 25 \
        --reverse-speed 20 \
        --safe-distance 55 \
        --danger-distance 35
"""

from __future__ import annotations

import argparse
import os

from marsy_backends.loader import load_rover
from marsy_core.motion import MarsyMotion
from marsy_core.sensors import MarsySensors
from marsy_core.telemetry import log
from behaviors.avoid_obstacle import AvoidObstacleBehavior, AvoidObstacleConfig


def parse_args():
    parser = argparse.ArgumentParser(description="Marsy obstacle avoidance mission")

    parser.add_argument(
        "--run-seconds",
        type=float,
        default=None,
        help="Run duration in seconds. Default: run until Ctrl+C.",
    )
    parser.add_argument(
        "--safe-distance",
        type=float,
        default=45.0,
        help="Distance below which Marsy starts avoiding an obstacle, in cm.",
    )
    parser.add_argument(
        "--danger-distance",
        type=float,
        default=25.0,
        help="Distance below which Marsy first reverses before turning, in cm.",
    )
    parser.add_argument(
        "--max-valid-distance",
        type=float,
        default=250.0,
        help="Ignore ultrasonic readings above this value, in cm.",
    )
    parser.add_argument(
        "--forward-speed",
        type=int,
        default=35,
        help="Forward speed 0-100. Keep conservative on the real rover.",
    )
    parser.add_argument(
        "--turn-speed",
        type=int,
        default=30,
        help="Avoidance turn speed 0-100.",
    )
    parser.add_argument(
        "--reverse-speed",
        type=int,
        default=25,
        help="Reverse speed 0-100.",
    )
    parser.add_argument(
        "--scan-angle",
        type=int,
        default=55,
        help="Mast scan angle left/right in degrees.",
    )
    parser.add_argument(
        "--steer-angle",
        type=int,
        default=28,
        help="Wheel steering angle for avoidance turns in degrees.",
    )
    parser.add_argument(
        "--reverse-time",
        type=float,
        default=0.55,
        help="How long to reverse when too close, in seconds.",
    )
    parser.add_argument(
        "--turn-time",
        type=float,
        default=1.05,
        help="How long to perform one avoidance arc turn, in seconds.",
    )
    parser.add_argument(
        "--no-echo-blocks",
        action="store_true",
        help="Treat missing/zero ultrasonic readings as unsafe instead of clear.",
    )
    parser.add_argument(
        "--default-turn",
        choices=["left", "right"],
        default="right",
        help="Tie-break turn direction if left and right scan look equal.",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Reduce logging.",
    )

    parser.add_argument(
        "--mast-reversed",
        action="store_true",
        help="Force reversed mast servo calibration for this run.",
    )
    parser.add_argument(
        "--mast-normal",
        action="store_true",
        help="Force normal mast servo calibration for this run.",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    if args.mast_reversed and args.mast_normal:
        raise SystemExit("Use only one of --mast-reversed or --mast-normal")

    if args.mast_reversed:
        os.environ["MARSY_MAST_REVERSED"] = "1"
    elif args.mast_normal:
        os.environ["MARSY_MAST_REVERSED"] = "0"

    rover = load_rover()
    motion = MarsyMotion(rover)
    sensors = MarsySensors(rover)

    config = AvoidObstacleConfig(
        safe_distance_cm=args.safe_distance,
        danger_distance_cm=args.danger_distance,
        max_valid_distance_cm=args.max_valid_distance,
        no_echo_means_clear=not args.no_echo_blocks,
        forward_speed=args.forward_speed,
        turn_speed=args.turn_speed,
        reverse_speed=args.reverse_speed,
        scan_angle=args.scan_angle,
        steer_angle=args.steer_angle,
        reverse_time_s=args.reverse_time,
        turn_time_s=args.turn_time,
        run_seconds=args.run_seconds,
        default_turn_direction=args.default_turn,
        verbose=not args.quiet,
    )

    behavior = AvoidObstacleBehavior(
        motion=motion,
        sensors=sensors,
        config=config,
    )

    try:
        log("Initializing Marsy obstacle avoidance...", source="avoid_obstacle", event="mission_init")
        rover.init(0)

        motion.stop()
        motion.wheels_straight()
        motion.mast_center()

        log("Obstacle avoidance started.", source="avoid_obstacle", event="mission_started")
        log("Press Ctrl+C to stop.", source="avoid_obstacle", event="mission_hint")
        log(
            "Config: "
            f"safe={config.safe_distance_cm} cm, "
            f"danger={config.danger_distance_cm} cm, "
            f"forward_speed={config.forward_speed}, "
            f"turn_speed={config.turn_speed}, "
            f"reverse_speed={config.reverse_speed}",
            source="avoid_obstacle",
            event="mission_config",
            safe_distance_cm=config.safe_distance_cm,
            danger_distance_cm=config.danger_distance_cm,
            forward_speed=config.forward_speed,
            turn_speed=config.turn_speed,
            reverse_speed=config.reverse_speed,
        )
        log(
            f"Mast calibration: {motion.mast_debug_config()}",
            source="avoid_obstacle",
            event="mast_calibration",
            **motion.mast_debug_config(),
        )

        behavior.run()

    except KeyboardInterrupt:
        log("Interrupted", source="avoid_obstacle", level="warning", event="mission_interrupted")

    finally:
        log("Cleanup...", source="avoid_obstacle", event="mission_cleanup_started")
        behavior.shutdown()
        try:
            rover.cleanup()
        except Exception as exc:
            log(f"rover.cleanup() failed: {exc}", source="avoid_obstacle", level="warning", event="rover_cleanup_failed", error=str(exc))
        log("Cleaned up", source="avoid_obstacle", event="mission_cleaned_up")


if __name__ == "__main__":
    main()
