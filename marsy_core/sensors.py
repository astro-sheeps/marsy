import time


class MarsySensors:
    def __init__(self, rover):
        self.rover = rover

    def distance_cm(self):
        """
        Return ultrasonic distance in cm.

        The real rover.py returns distance from getDistance().
        The simulator also exposes getDistance(), but depending on simulator state
        it may be simplified.
        """
        try:
            distance = self.rover.getDistance()
        except Exception as exc:
            print(f"Distance read failed: {exc}")
            return None

        if distance is None:
            return None

        try:
            distance = float(distance)
        except ValueError:
            return None

        return distance

    def battery_v(self):
        """
        Return battery voltage if backend supports it.
        The original rover.py may not expose this reliably for all versions.
        """
        if not hasattr(self.rover, "getBattery"):
            return None

        try:
            return float(self.rover.getBattery())
        except Exception:
            return None

    def scan_mast(self, motion, left_angle=45, right_angle=45, settle_s=0.4):
        """
        Scan left, center, right using mast servo and ultrasonic sensor.
        Returns dict with distances.
        """
        result = {}

        motion.mast_left(left_angle)
        time.sleep(settle_s)
        result["left"] = self.distance_cm()

        motion.mast_center()
        time.sleep(settle_s)
        result["center"] = self.distance_cm()

        motion.mast_right(right_angle)
        time.sleep(settle_s)
        result["right"] = self.distance_cm()

        motion.mast_center()
        time.sleep(settle_s)

        return result