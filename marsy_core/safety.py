import time


class MarsySafety:
    def __init__(
        self,
        rover,
        max_speed=70,
        max_drive_seconds=3.0,
        safe_distance_cm=25,
    ):
        self.rover = rover
        self.max_speed = max_speed
        self.max_drive_seconds = max_drive_seconds
        self.safe_distance_cm = safe_distance_cm
        self.last_drive_time = None
        self.emergency_stopped = False

    def clamp_speed(self, speed):
        return max(0, min(self.max_speed, speed))

    def mark_drive_command(self):
        self.last_drive_time = time.time()

    def clear_drive_timer(self):
        self.last_drive_time = None

    def emergency_stop(self, reason="manual"):
        self.rover.brake()
        self.emergency_stopped = True
        print(f"EMERGENCY STOP: {reason}")

    def reset_emergency_stop(self):
        self.emergency_stopped = False
        print("Emergency stop reset")

    def check_drive_timeout(self):
        if self.last_drive_time is None:
            return

        elapsed = time.time() - self.last_drive_time

        if elapsed > self.max_drive_seconds:
            self.emergency_stop(
                f"drive command active for {elapsed:.1f}s"
            )

    def check_distance(self):
        try:
            distance = self.rover.getDistance()
        except Exception:
            return None

        if distance > 0 and distance < self.safe_distance_cm:
            self.emergency_stop(
                f"obstacle at {distance:.1f} cm"
            )

        return distance