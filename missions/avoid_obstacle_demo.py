from marsy_backends.loader import load_rover
from marsy_core.motion import MarsyMotion
from marsy_core.sensors import MarsySensors
from marsy_core.safety import MarsySafety
from behaviors.avoid_obstacle import AvoidObstacleBehavior


def main():
    rover = load_rover()

    motion = MarsyMotion(rover)
    sensors = MarsySensors(rover)

    safety = MarsySafety(
        rover,
        max_speed=60,
        max_drive_seconds=3.0,
        safe_distance_cm=25,
    )

    behavior = AvoidObstacleBehavior(
        rover=rover,
        motion=motion,
        sensors=sensors,
        safety=safety,
        drive_speed=45,
        turn_speed=45,
        safe_distance_cm=30,
        scan_angle=45,
    )

    try:
        rover.init(0)
        motion.wheels_straight()
        motion.mast_center()

        behavior.run(
            duration_s=30,
            loop_delay_s=0.3,
        )

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        try:
            motion.brake()
            motion.wheels_straight()
            motion.mast_center()
            rover.cleanup()
            print("Cleaned up")
        except Exception as exc:
            print(f"Cleanup failed: {exc}")


if __name__ == "__main__":
    main()