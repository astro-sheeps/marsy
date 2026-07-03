import time

from marsy_backends.loader import load_rover
from marsy_core.motion import MarsyMotion


def main():
    rover = load_rover()
    motion = MarsyMotion(rover)

    try:
        rover.init(0)

        print("Forward")
        motion.forward(60)
        time.sleep(2)

        print("Stop")
        motion.stop()
        time.sleep(1)

        print("Steer left + forward")
        motion.steer_left(20)
        rover.forward(50)
        time.sleep(2)

        print("Stop")
        motion.stop()
        time.sleep(1)

        print("Steer right + forward")
        motion.steer_right(20)
        rover.forward(50)
        time.sleep(2)

        print("Mast left")
        motion.mast_left(45)
        time.sleep(1)

        print("Mast right")
        motion.mast_right(45)
        time.sleep(1)

        print("Mast center")
        motion.mast_center()
        time.sleep(1)

        print("Final stop")
        motion.stop()
        motion.wheels_straight()

    finally:
        rover.cleanup()


if __name__ == "__main__":
    main()