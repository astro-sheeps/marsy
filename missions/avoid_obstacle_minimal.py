import time

from marsy_backends.loader import load_rover
from marsy_core.motion import MarsyMotion
from marsy_core.sensors import MarsySensors
from marsy_core.safety import MarsySafety


SAFE_DISTANCE_CM = 30
DRIVE_SPEED = 45
TURN_DRIVE_SPEED = 35
TURN_STEER_ANGLE = 20
RUN_SECONDS = 20


def main():
    print("Loading rover...", flush=True)
    rover = load_rover()

    motion = MarsyMotion(rover)
    sensors = MarsySensors(rover)

    safety = MarsySafety(
        rover,
        max_speed=60,
        max_drive_seconds=3.0,
        safe_distance_cm=SAFE_DISTANCE_CM,
    )

    try:
        print("Calling rover.init...", flush=True)
        rover.init(0)
        print("rover.init done", flush=True)

        start = time.time()
        print("Minimal avoid-obstacle demo started", flush=True)

        while time.time() - start < RUN_SECONDS:
            distance = sensors.distance_cm()
            print(f"Distance: {distance}", flush=True)

            if distance is not None and distance > 0 and distance < SAFE_DISTANCE_CM:
                print(f"Obstacle detected at {distance:.1f} cm", flush=True)

                # Не вызываем stop(), потому что именно на нём у тебя виснет
                print("Steering right...", flush=True)
                motion.steer_right(TURN_STEER_ANGLE)
                print("Wheels turned right", flush=True)

                time.sleep(0.5)

                print("Driving right arc...", flush=True)
                rover.forward(TURN_DRIVE_SPEED)
                safety.mark_drive_command()

                time.sleep(1.0)

                print("Centering wheels...", flush=True)
                motion.wheels_straight()
                print("Wheels centered", flush=True)

                time.sleep(0.3)

            else:
                safe_speed = safety.clamp_speed(DRIVE_SPEED)
                motion.wheels_straight()
                print(f"Forward {safe_speed}", flush=True)
                rover.forward(safe_speed)
                safety.mark_drive_command()

            safety.check_drive_timeout()
            time.sleep(0.3)

        print("Demo finished", flush=True)

    except KeyboardInterrupt:
        print("\nInterrupted", flush=True)

    finally:
        print("Cleanup...", flush=True)
        try:
            # Если stop() у тебя проблемный, можно пока cleanup оставить как есть
            rover.cleanup()
            print("Cleaned up", flush=True)
        except Exception as exc:
            print(f"Cleanup failed: {exc}", flush=True)


if __name__ == "__main__":
    main()