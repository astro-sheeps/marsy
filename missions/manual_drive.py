import sys
import tty
import termios

from marsy_backends.loader import load_rover
from marsy_core.motion import MarsyMotion
from marsy_core.safety import MarsySafety


def readchar():
    """
    Read one character from terminal without waiting for Enter.
    Works on macOS/Linux terminal.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    if ord(ch) == 3:
        raise KeyboardInterrupt

    return ch


def readkey():
    """
    Read normal keys and arrow keys.

    Arrow key convention follows original 4tronix examples:
        16 = Up
        17 = Down
        18 = Right
        19 = Left
    """
    c1 = readchar()

    # Normal key
    if ord(c1) != 0x1B:
        return c1

    # Escape sequence for arrows
    c2 = readchar()
    if ord(c2) != 0x5B:
        return c1

    c3 = readchar()
    return chr(0x10 + ord(c3) - 65)


def print_help(speed, steer_angle, mast_angle):
    print()
    print("Marsy manual drive")
    print("------------------")
    print("w / ↑      forward")
    print("z / ↓      reverse")
    print("a / ←      steer left")
    print("s / →      steer right")
    print("space      stop")
    print("b          brake")
    print("e          emergency stop")
    print("r          reset emergency stop")
    print("c          center wheels")
    print("j          mast left")
    print("l          mast right")
    print("k          mast center")
    print(", / <      speed down")
    print(". / >      speed up")
    print("h          help")
    print("q          quit")
    print()
    print(f"Current speed: {speed}")
    print(f"Steer angle:   {steer_angle}")
    print(f"Mast angle:    {mast_angle}")
    print()


def main():
    rover = load_rover()
    motion = MarsyMotion(rover)

    safety = MarsySafety(
        rover,
        max_speed=70,
        max_drive_seconds=3.0,
        safe_distance_cm=25,
    )

    speed = 60
    steer_angle = 20
    mast_angle = 45

    rover.init(0)
    motion.wheels_straight()
    motion.mast_center()

    print_help(speed, steer_angle, mast_angle)

    try:
        while True:
            key = readkey()

            if key == "w" or ord(key) == 16:
                if safety.emergency_stopped:
                    print("Emergency stop is active. Press r to reset.")
                    continue

                safe_speed = safety.clamp_speed(speed)
                motion.forward(speed, straighten=False)
                safety.mark_drive_command()
                print(f"Forward {safe_speed}")

            elif key == "z" or ord(key) == 17:
                if safety.emergency_stopped:
                    print("Emergency stop is active. Press r to reset.")
                    continue

                safe_speed = safety.clamp_speed(speed)
                motion.reverse(speed, straighten=False)
                safety.mark_drive_command()
                print(f"Reverse {safe_speed}")

            elif key == "a" or ord(key) == 19:
                motion.steer_left(steer_angle)
                print(f"Steer left {steer_angle}")

            elif key == "s" or ord(key) == 18:
                motion.steer_right(steer_angle)
                print(f"Steer right {steer_angle}")

            elif key == " ":
                motion.stop()
                safety.clear_drive_timer()
                print("Stop")

            elif key == "b":
                motion.brake()
                motion.wheels_straight()
                safety.clear_drive_timer()
                print("Brake")

            elif key == "e":
                safety.emergency_stop("manual key")
                safety.clear_drive_timer()

            elif key == "r":
                safety.reset_emergency_stop()
                safety.clear_drive_timer()

            elif key == "c":
                motion.wheels_straight()
                print("Wheels centered")

            elif key == "j":
                motion.mast_left(mast_angle)
                print(f"Mast left {mast_angle}")

            elif key == "l":
                motion.mast_right(mast_angle)
                print(f"Mast right {mast_angle}")

            elif key == "k":
                motion.mast_center()
                print("Mast center")

            elif key == "." or key == ">":
                speed = min(100, speed + 10)
                speed = safety.clamp_speed(speed)
                print(f"Speed up: {speed}")

            elif key == "," or key == "<":
                speed = max(0, speed - 10)
                print(f"Speed down: {speed}")

            elif key == "h":
                print_help(speed, steer_angle, mast_angle)

            elif key == "q":
                print("Quit")
                break

            else:
                print(f"Unknown key: {repr(key)}. Press h for help.")

            safety.check_drive_timeout()

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