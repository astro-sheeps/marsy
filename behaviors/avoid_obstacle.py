import time


class AvoidObstacleBehavior:
    def __init__(
        self,
        rover,
        motion,
        sensors,
        safety,
        drive_speed=50,
        turn_speed=45,
        safe_distance_cm=30,
        scan_angle=45,
    ):
        self.rover = rover
        self.motion = motion
        self.sensors = sensors
        self.safety = safety

        self.drive_speed = drive_speed
        self.turn_speed = turn_speed
        self.safe_distance_cm = safe_distance_cm
        self.scan_angle = scan_angle

    def is_obstacle_close(self, distance):
        if distance is None:
            return False

        # In 4tronix rover.py, 0 can mean no object / no echo.
        if distance <= 0:
            return False

        return distance < self.safe_distance_cm

    def choose_direction(self, scan):
        """
        Choose the side with larger distance.
        If one side is None, prefer the valid side.
        If both are invalid, default to right.
        """
        left = scan.get("left")
        right = scan.get("right")

        if left is None and right is None:
            return "right"

        if left is None:
            return "right"

        if right is None:
            return "left"

        if left >= right:
            return "left"

        return "right"

    def turn_left_in_place(self, duration_s=1.0):
        print("Turning left")
        self.rover.spinLeft(self.turn_speed)
        time.sleep(duration_s)
        self.motion.stop()

    def turn_right_in_place(self, duration_s=1.0):
        print("Turning right")
        self.rover.spinRight(self.turn_speed)
        time.sleep(duration_s)
        self.motion.stop()

    def step(self):
        """
        One behavior iteration.
        Returns True if behavior should continue, False if it should stop.
        """
        if self.safety.emergency_stopped:
            print("Emergency stop active. Behavior halted.")
            return False

        distance = self.sensors.distance_cm()
        print(f"Distance: {distance}")

        if self.is_obstacle_close(distance):
            print(f"Obstacle detected at {distance:.1f} cm")
            self.motion.brake()
            self.safety.clear_drive_timer()
            time.sleep(0.5)

            scan = self.sensors.scan_mast(
                self.motion,
                left_angle=self.scan_angle,
                right_angle=self.scan_angle,
            )

            print(f"Scan: {scan}")

            direction = self.choose_direction(scan)
            print(f"Chosen direction: {direction}")

            if direction == "left":
                self.turn_left_in_place(duration_s=1.0)
            else:
                self.turn_right_in_place(duration_s=1.0)

            self.motion.wheels_straight()
            return True

        safe_speed = self.safety.clamp_speed(self.drive_speed)
        self.motion.forward(safe_speed)
        self.safety.mark_drive_command()

        return True

    def run(self, duration_s=20, loop_delay_s=0.3):
        """
        Run behavior for limited time.
        """
        start = time.time()

        print("AvoidObstacleBehavior started")

        try:
            while time.time() - start < duration_s:
                should_continue = self.step()

                if not should_continue:
                    break

                self.safety.check_drive_timeout()
                time.sleep(loop_delay_s)

        finally:
            print("AvoidObstacleBehavior stopping")
            self.motion.brake()
            self.motion.wheels_straight()
            self.motion.mast_center()
            self.safety.clear_drive_timer()