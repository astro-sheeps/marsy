"""
Marsy motion controller.

High-level movement layer for Marsy.

This module does not know whether it talks to:
- the real 4tronix rover.py backend
- the external simulator backend
- a future custom Marsy simulator

It only expects a rover-like object with methods such as:
    forward()
    reverse()
    stop()
    brake()
    spinLeft()
    spinRight()
    setServo()

Optional backend method:
    setServos()

If setServos() exists, wheel steering is sent as one batch command.
This is useful for the simulator because it avoids several slow HTTP requests.
"""


# ---------------------------------------------------------------------
# Servo mapping for 4tronix M.A.R.S. Rover
# ---------------------------------------------------------------------

SERVO_MAST = 0

SERVO_FL = 9    # Front left wheel steering servo
SERVO_RL = 11   # Rear left wheel steering servo
SERVO_RR = 13   # Rear right wheel steering servo
SERVO_FR = 15   # Front right wheel steering servo


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def clamp_speed(speed):
    """
    Clamp motor speed to 0..100.
    """
    return int(max(0, min(100, speed)))


def clamp_angle(angle):
    """
    Clamp servo angle to -90..90.
    """
    return int(max(-90, min(90, angle)))


# ---------------------------------------------------------------------
# Motion controller
# ---------------------------------------------------------------------

class MarsyMotion:
    def __init__(self, rover):
        self.rover = rover

    # -----------------------------------------------------------------
    # Low-level wheel servo control
    # -----------------------------------------------------------------

    def set_wheel_servos(self, fl, fr, rl, rr):
        """
        Set all four wheel steering servos.

        If backend supports setServos(), send all wheel angles in one batch.
        Otherwise fall back to four standard setServo() calls.

        Args:
            fl: front-left wheel angle
            fr: front-right wheel angle
            rl: rear-left wheel angle
            rr: rear-right wheel angle
        """
        fl = clamp_angle(fl)
        fr = clamp_angle(fr)
        rl = clamp_angle(rl)
        rr = clamp_angle(rr)

        if hasattr(self.rover, "setServos"):
            self.rover.setServos({
                SERVO_FL: fl,
                SERVO_FR: fr,
                SERVO_RL: rl,
                SERVO_RR: rr,
            })
            return

        self.rover.setServo(SERVO_FL, fl)
        self.rover.setServo(SERVO_FR, fr)
        self.rover.setServo(SERVO_RL, rl)
        self.rover.setServo(SERVO_RR, rr)

    def wheels_straight(self):
        """
        Set all steering wheels straight.
        """
        self.set_wheel_servos(0, 0, 0, 0)

    def steer_left(self, angle=20):
        """
        Steer wheels for left arc movement.

        Same convention as the original 4tronix driveRover.py:
            front wheels: -angle
            rear wheels:  +angle
        """
        angle = clamp_angle(angle)
        self.set_wheel_servos(
            fl=-angle,
            fr=-angle,
            rl=angle,
            rr=angle,
        )

    def steer_right(self, angle=20):
        """
        Steer wheels for right arc movement.

        Same convention as the original 4tronix driveRover.py:
            front wheels: +angle
            rear wheels:  -angle
        """
        angle = clamp_angle(angle)
        self.set_wheel_servos(
            fl=angle,
            fr=angle,
            rl=-angle,
            rr=-angle,
        )

    # -----------------------------------------------------------------
    # Motor movement
    # -----------------------------------------------------------------

    def forward(self, speed=60, straighten=True):
        """
        Move forward.

        By default, straighten wheels before driving.
        For manual driving, you may want straighten=False so Marsy keeps
        the current steering angle.
        """
        speed = clamp_speed(speed)

        if straighten:
            self.wheels_straight()

        self.rover.forward(speed)

    def reverse(self, speed=60, straighten=True):
        """
        Move backward.

        By default, straighten wheels before reversing.
        """
        speed = clamp_speed(speed)

        if straighten:
            self.wheels_straight()

        self.rover.reverse(speed)

    def stop(self):
        """
        Coast stop.
        """
        self.rover.stop()

    def brake(self):
        """
        Quick brake.
        """
        self.rover.brake()

    def spin_left(self, speed=50):
        """
        Spin left in place using opposite wheel directions.

        This does not visibly steer the wheel servos.
        """
        speed = clamp_speed(speed)
        self.rover.spinLeft(speed)

    def spin_right(self, speed=50):
        """
        Spin right in place using opposite wheel directions.

        This does not visibly steer the wheel servos.
        """
        speed = clamp_speed(speed)
        self.rover.spinRight(speed)

    def turn_forward(self, left_speed, right_speed):
        """
        Drive forward with different left/right motor speeds.
        """
        left_speed = clamp_speed(left_speed)
        right_speed = clamp_speed(right_speed)
        self.rover.turnForward(left_speed, right_speed)

    def turn_reverse(self, left_speed, right_speed):
        """
        Drive backward with different left/right motor speeds.
        """
        left_speed = clamp_speed(left_speed)
        right_speed = clamp_speed(right_speed)
        self.rover.turnReverse(left_speed, right_speed)

    # -----------------------------------------------------------------
    # Combined arc helpers
    # -----------------------------------------------------------------

    def forward_left(self, speed=50, angle=20):
        """
        Steer left and move forward.
        """
        self.steer_left(angle)
        self.rover.forward(clamp_speed(speed))

    def forward_right(self, speed=50, angle=20):
        """
        Steer right and move forward.
        """
        self.steer_right(angle)
        self.rover.forward(clamp_speed(speed))

    def reverse_left(self, speed=50, angle=20):
        """
        Steer left and reverse.
        """
        self.steer_left(angle)
        self.rover.reverse(clamp_speed(speed))

    def reverse_right(self, speed=50, angle=20):
        """
        Steer right and reverse.
        """
        self.steer_right(angle)
        self.rover.reverse(clamp_speed(speed))

    # -----------------------------------------------------------------
    # Mast servo
    # -----------------------------------------------------------------

    def mast_center(self):
        """
        Center mast/head servo.
        """
        self.rover.setServo(SERVO_MAST, 0)

    def mast_left(self, angle=45):
        """
        Rotate mast/head left.
        """
        self.rover.setServo(SERVO_MAST, -clamp_angle(angle))

    def mast_right(self, angle=45):
        """
        Rotate mast/head right.
        """
        self.rover.setServo(SERVO_MAST, clamp_angle(angle))

    def mast_to(self, angle):
        """
        Set mast/head to an explicit angle.
        """
        self.rover.setServo(SERVO_MAST, clamp_angle(angle))

    # -----------------------------------------------------------------
    # Safe reset helpers
    # -----------------------------------------------------------------

    def reset_pose(self):
        """
        Reset steering wheels and mast to neutral positions.
        """
        self.wheels_straight()
        self.mast_center()

    def stop_and_reset(self):
        """
        Stop motors and reset servos to neutral positions.
        """
        self.stop()
        self.reset_pose()

    def brake_and_reset(self):
        """
        Brake motors and reset servos to neutral positions.
        """
        self.brake()
        self.reset_pose()