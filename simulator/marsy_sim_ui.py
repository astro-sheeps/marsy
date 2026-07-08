# 4tronix M.A.R.S. Rover Simulator UI
#
# This displays a window representing the rover. It keeps track of its position
# and speed, and the orientation of its wheels. It models this in real time -
# if the virtual rover is set in motion, it will continue to move until further
# instructions are sent telling it to stop.
#
# This receives incoming HTTP requests to control the rover. Incoming messages
# are in JSON form. The following properties may be set in the top-level
# message:
#   wheelMotors
#   servos
#   rgbLeds
#
# A request setting both might look like this:
# {
#   "wheelMotors": {
#     "l": [ 100, 0 ],
#     "r": [ 0, 100 ]
#   },
#   "servos": {
#     "0": 0,
#     "9": -10
#     "10": -10  
#   },
#   "rgbLeds": {
#     "0": [255, 0, 0],
#     "3": [0, 255, 255]
#   }
# }
#
# The wheelMotors settings reflect the way the board itself is designed: it
# seems that there are separate PWM outputs for each direction. The duty
# cycle is set directly to the speed. More cryptically, the frequency is also
# adjusted with the speed. Maximum speed is 100, and the frequency is the speed
# divided by 2 in Hz. So at maximum speed, the PWM frequency is 50Hz. At half
# speed it's 25Hz, and so on until we get to a floor of 10Hz. However, we're
# not simulating at that level of detail. We're just accepting the speeds in
# each direction, and setting the speed to 100 in both directions actively
# brakes the wheels (whereas setting both speeds to zero lets the motor coast
# naturally to a halt, which it does pretty quickly).
# There are 16 servo outputs, although with a standard setup only 5 are used.
# Any servos not specified in the message will not have their positions
# changed.
#
# The response is always of the same format (even if the request is empty):
# {
#   "ultrasonicRange": 80
# }
#
# This reports the detected range from the ultrasonic sensor.
   
import sys
import math
import os
import subprocess
import threading
from pathlib import Path
from time import time
from datetime import datetime
import json
import signal
import logging

from PyQt6.QtCore import QThread, QObject, QTimer, pyqtSignal, QRectF, Qt
from PyQt6.QtWidgets import QApplication, QLabel, QWidget, QFrame, QPushButton, QSlider, QGraphicsScene, QGraphicsView, QGraphicsRectItem, QGraphicsItemGroup, QGraphicsPixmapItem, QPlainTextEdit
from PyQt6.QtGui import QPixmap, QTransform, QColor, QPen, QBrush, QPainter

from flask import Flask, request, jsonify

BUILD_STAMP = "mission_launcher_v7_2026_07_08"

# Keep the simulator terminal readable. Mission telemetry appears in the UI panel,
# not as Flask request spam.
logging.getLogger("werkzeug").setLevel(logging.ERROR)
logging.getLogger("werkzeug").disabled = True

# Servo assignments
servo_FL = 9
servo_FR = 15
servo_RL = 11
servo_RR = 13
servo_MA = 0

showSteeringCalcs = False

# ---------------------------------------------------------------------
# Marsy simulated world
# ---------------------------------------------------------------------
# Coordinates are in centimeters, using the same world coordinate system
# as the rover:
#   +X is to the right
#   +Y is upward
#
# The PyQt scene uses +Y downward, so drawing code converts y -> -y.
MARSY_OBSTACLES = [
    # Starting layout for obstacle-avoidance demos:
    # the rover starts near the bottom of the field and points upward.
    # This first obstacle is directly in front of it.
    {"type": "circle", "x": 0, "y": -55, "r": 18, "label": "box"},

    # Extra obstacles higher in the field, so there is something to avoid
    # after the first turn.
    {"type": "circle", "x": 55, "y": -5, "r": 18, "label": "rock"},
    {"type": "circle", "x": -70, "y": 20, "r": 20, "label": "crater"},
    {"type": "circle", "x": 20, "y": 75, "r": 14, "label": "stone"},
]


SONAR_MAX_RANGE_CM = 200.0
SONAR_FOV_DEG = 35.0

# The real ultrasonic sensor is not a mathematical laser line.
# Give it a small beam width by inflating circular obstacles for raycast.
# This makes Marsy detect the 'volume' of obstacles instead of only a thin center line.
SONAR_BEAM_RADIUS_CM = 8.0

# Approximate body clearance used for collision checks.
ROVER_BODY_RADIUS_CM = 13.0

# Extra clearance so Marsy stops just before visually entering an object.
COLLISION_MARGIN_CM = 2.0

# Manual UI control defaults.
SIM_MANUAL_SPEED = 45
SIM_STEER_ANGLE_DEG = 22
UI_SAFE_DISTANCE_CM = 30.0

CURRENT_ROVER = None

# Shared lightweight simulator control state.
# The Flask /state endpoint exposes this to mission code, so UI buttons can
# request a graceful stop of an autonomous mission running in another process.
SIM_CONTROL_STATE = {
    "mode": "manual",
    "control_source": "UI / API",
    "mission_name": None,
    "mission_stop_requested": False,
    "mission_stop_reason": None,
}



def sonar_origin_and_direction(rover):
    """
    Return ultrasonic ray origin and direction in world coordinates.

    Coordinate convention:
      +X is right
      +Y is upward
      heading 0 means rover points toward +Y

    The mast servo is added to the rover heading. Negative mast angles look
    left, positive mast angles look right, matching MarsyMotion.mast_left/right.
    """
    angle_deg = rover.vehicleHeadingDegrees + rover.servos[servo_MA]
    angle_rad = math.radians(angle_deg)

    dx = math.sin(angle_rad)
    dy = math.cos(angle_rad)

    # Approximate the ultrasonic sensor as being on the front of the rover.
    ox = rover.vehicleXcm + dx * (rover.vehicleHeightCm / 2.0)
    oy = rover.vehicleYcm + dy * (rover.vehicleHeightCm / 2.0)

    return ox, oy, dx, dy, angle_deg


def ray_circle_distance(origin_x, origin_y, dir_x, dir_y, center_x, center_y, radius):
    """
    Return distance from ray origin to first intersection with a circle.

    Returns None if the ray does not hit the circle in front of the sensor.
    """
    fx = origin_x - center_x
    fy = origin_y - center_y

    # Direction is already unit length, so a = 1.
    b = 2.0 * (fx * dir_x + fy * dir_y)
    c = fx * fx + fy * fy - radius * radius

    discriminant = b * b - 4.0 * c
    if discriminant < 0:
        return None

    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / 2.0
    t2 = (-b + sqrt_disc) / 2.0

    candidates = [t for t in (t1, t2) if t >= 0]
    if not candidates:
        return None

    return min(candidates)


def cone_circle_distance(origin_x, origin_y, dir_x, dir_y, center_x, center_y, radius):
    """
    Approximate a wide ultrasonic cone hitting a circular obstacle.

    The old simulator used a thin ray. This model checks whether the obstacle
    center falls inside the sonar cone after accounting for obstacle radius.
    It returns the approximate distance along the sonar centerline to the
    obstacle front surface, or None if there is no hit.
    """
    vx = center_x - origin_x
    vy = center_y - origin_y

    forward = vx * dir_x + vy * dir_y
    if forward < 0:
        return None

    center_dist_sq = vx * vx + vy * vy
    perpendicular_sq = max(0.0, center_dist_sq - forward * forward)
    perpendicular = math.sqrt(perpendicular_sq)

    half_fov_rad = math.radians(SONAR_FOV_DEG / 2.0)
    cone_half_width = max(
        SONAR_BEAM_RADIUS_CM,
        math.tan(half_fov_rad) * max(0.0, forward),
    )

    if perpendicular > radius + cone_half_width:
        return None

    # Approximate distance to the front of the obstacle along the beam.
    return max(0.0, forward - radius)


def compute_ultrasonic_distance(rover):
    """
    Simulate the ultrasonic distance sensor using circular obstacles.

    Returns:
      distance_cm: nearest hit distance in cm, or 0.0 if there is no echo
      hit: optional hit metadata
    """
    ox, oy, dx, dy, angle_deg = sonar_origin_and_direction(rover)

    nearest_distance = None
    nearest_obstacle = None

    for obstacle in MARSY_OBSTACLES:
        if obstacle.get("type") != "circle":
            continue

        obstacle_radius = float(obstacle["r"])

        distance = cone_circle_distance(
            origin_x=ox,
            origin_y=oy,
            dir_x=dx,
            dir_y=dy,
            center_x=float(obstacle["x"]),
            center_y=float(obstacle["y"]),
            radius=obstacle_radius,
        )

        if distance is None:
            continue

        if distance > SONAR_MAX_RANGE_CM:
            continue

        if nearest_distance is None or distance < nearest_distance:
            nearest_distance = distance
            nearest_obstacle = obstacle

    if nearest_distance is None:
        return 0.0, None

    hit_x = ox + dx * nearest_distance
    hit_y = oy + dy * nearest_distance

    return round(nearest_distance, 2), {
        "x": round(hit_x, 2),
        "y": round(hit_y, 2),
        "obstacle": nearest_obstacle.get("label", "obstacle"),
        "sonar_fov_deg": SONAR_FOV_DEG,
        "sonar_beam_radius_cm": SONAR_BEAM_RADIUS_CM,
        "visual_radius_cm": nearest_obstacle.get("r"),
    }


def collision_obstacle_at(x_cm, y_cm, body_radius_cm=ROVER_BODY_RADIUS_CM):
    """
    Return the obstacle that collides with the rover body at the given position.

    The simulator represents the rover body as a circle for collision purposes.
    Obstacles are circular too, so collision is a simple circle-circle test.
    """
    for obstacle in MARSY_OBSTACLES:
        if obstacle.get("type") != "circle":
            continue

        dx = float(x_cm) - float(obstacle["x"])
        dy = float(y_cm) - float(obstacle["y"])
        distance = math.sqrt(dx * dx + dy * dy)
        min_distance = body_radius_cm + float(obstacle["r"]) + COLLISION_MARGIN_CM

        if distance < min_distance:
            return obstacle

    return None


def is_collision_at(x_cm, y_cm):
    return collision_obstacle_at(x_cm, y_cm) is not None


def build_state_dict(rover):
    """
    Build simulator state JSON for /state and POST responses.
    """
    if rover is None:
        return {
            "ready": False,
            "distance_cm": 0.0,
            "ultrasonicRange": 0.0,
            "obstacles": MARSY_OBSTACLES,
            "collision": False,
            "collision_obstacle": None,
        }

    distance_cm, hit = compute_ultrasonic_distance(rover)
    ox, oy, dx, dy, sonar_angle_deg = sonar_origin_and_direction(rover)

    return {
        "ready": True,
        "x_cm": round(rover.vehicleXcm, 2),
        "y_cm": round(rover.vehicleYcm, 2),
        "heading_deg": round(rover.vehicleHeadingDegrees, 2),
        "speed_left": rover.speedL,
        "speed_right": rover.speedR,
        "speed_m_s": round(((abs(rover.speedL) + abs(rover.speedR)) / 2.0) / 100.0 * fullSpeedCmPerSecond / 100.0, 3),
        "battery_percent": 87,
        "collision": bool(getattr(rover, "last_collision_label", None)),
        "collision_obstacle": getattr(rover, "last_collision_label", None),
        "mast_deg": rover.servos[servo_MA],
        "distance_cm": distance_cm,
        "ultrasonicRange": distance_cm,
        "sonar": {
            "origin_x_cm": round(ox, 2),
            "origin_y_cm": round(oy, 2),
            "direction_x": round(dx, 4),
            "direction_y": round(dy, 4),
            "angle_deg": round(sonar_angle_deg, 2),
            "max_range_cm": SONAR_MAX_RANGE_CM,
            "fov_deg": SONAR_FOV_DEG,
            "beam_radius_cm": SONAR_BEAM_RADIUS_CM,
            "hit": hit,
        },
        "obstacles": MARSY_OBSTACLES,
        "control": dict(SIM_CONTROL_STATE),
        "mission_stop_requested": bool(SIM_CONTROL_STATE.get("mission_stop_requested", False)),
        "mode": SIM_CONTROL_STATE.get("mode", "manual"),
    }

# Receives requests
class ServerWorker(QObject):
    mysignal = pyqtSignal(str)
    telemetry_signal = pyqtSignal(str)
    http_server = Flask("RoverSimUi")

    def run(self):
        self.http_server.route('/', methods=['POST'])(self.result)
        self.http_server.route('/state', methods=['GET'])(self.state)
        self.http_server.route('/distance', methods=['GET'])(self.distance)
        self.http_server.route('/telemetry', methods=['POST'])(self.telemetry)
        self.http_server.route('/log', methods=['POST'])(self.telemetry)
        self.http_server.route('/control', methods=['GET', 'POST'])(self.control)
        self.http_server.run(port=8523, debug=False, use_reloader=False)

    def result(self): #, *args, **kwargs):
        data = request.get_json(silent=True) or {}
        bodyText = json.dumps(data)
        self.mysignal.emit(bodyText)
        return jsonify(build_state_dict(CURRENT_ROVER))

    def state(self):
        return jsonify(build_state_dict(CURRENT_ROVER))

    def distance(self):
        state = build_state_dict(CURRENT_ROVER)
        return jsonify({
            "distance_cm": state["distance_cm"],
            "ultrasonicRange": state["ultrasonicRange"],
        })

    def control(self):
        if request.method == 'POST':
            data = request.get_json(silent=True) or {}
            action = str(data.get("action", "")).strip().lower()

            if action in {"stop", "mission_stop", "emergency_stop"}:
                SIM_CONTROL_STATE["mission_stop_requested"] = True
                SIM_CONTROL_STATE["mission_stop_reason"] = data.get("reason", "ui_stop")
                SIM_CONTROL_STATE["mode"] = "stopped"
                SIM_CONTROL_STATE["control_source"] = "UI STOP"
            elif action in {"reset", "clear_stop", "manual"}:
                SIM_CONTROL_STATE["mission_stop_requested"] = False
                SIM_CONTROL_STATE["mission_stop_reason"] = None
                SIM_CONTROL_STATE["mode"] = "manual"
                SIM_CONTROL_STATE["control_source"] = "UI / API"

        return jsonify({"ok": True, "control": dict(SIM_CONTROL_STATE)})

    def telemetry(self):
        data = request.get_json(silent=True)

        if data is None:
            data = {
                "source": "external",
                "level": "info",
                "message": request.data.decode("utf-8", errors="replace"),
            }
        elif isinstance(data, str):
            data = {
                "source": "external",
                "level": "info",
                "message": data,
            }
        elif not isinstance(data, dict):
            data = {
                "source": "external",
                "level": "info",
                "message": str(data),
            }

        if "message" not in data:
            data["message"] = json.dumps(data, ensure_ascii=False)

        self.telemetry_signal.emit(json.dumps(data, ensure_ascii=False))
        return jsonify({"ok": True})

fullSpeedCmPerSecond = 9

class Rover:
    vehicleWidthCm = 16
    vehicleHeightCm = 18
    distanceBetweenWheelPairsCm = 8
    timeOfLastUpdate = time()
    # Start near the bottom of the field, facing upward.
    # Coordinate convention: +Y is upward, while QGraphics draws +Y downward.
    vehicleXcm = 0
    vehicleYcm = -115
    vehicleHeadingDegrees = 0
    speedL = 0
    speedR = 0
    servos = [0] * 16
    rgbLeds = [[0,0,0]] * 4
    last_collision_label = None

    def reset(self):
        self.timeOfLastUpdate = time()
        self.vehicleXcm = 0
        self.vehicleYcm = -115
        self.vehicleHeadingDegrees = 0
        self.speedL = 0
        self.speedR = 0
        self.servos = [0] * 16
        self.rgbLeds = [[0, 0, 0]] * 4
        self.last_collision_label = None

    def setServo(self, servoId, value):
        self.servos[servoId] = value

    def setWheelMotorLeft(self, fwd, rev):
        if fwd > 0 and rev > 0:
            self.speedL = 0
        else:
            self.speedL = fwd - rev

    def setWheelMotorRight(self, fwd, rev):
        if fwd > 0 and rev > 0:
            self.speedR = 0
        else:
            self.speedR = fwd - rev

    def setRgbLed(self, ledId, rgbValues):
        self.rgbLeds[ledId] = rgbValues
    
    def updateState(self):
        currentTime = time()
        timeSinceLastUpdate = currentTime - self.timeOfLastUpdate
        self.timeOfLastUpdate = currentTime

        old_x_cm = self.vehicleXcm
        old_y_cm = self.vehicleYcm
        old_heading_degrees = self.vehicleHeadingDegrees
        self.last_collision_label = None

        # Working out the direction and distance of travel is surprisingly
        # complex, not least because there's no guarantee that all 4 steerable
        # wheels are working together - they could be fighting one another.
        # There are a few ways we could try to work it out:
        #   1. an idealised model in which we presume the wheels cannot slip
        #       sideways and work out the rotation and direction of travel
        #   2. determine the resultant force and moment on the rover by
        #       considering the forces from all 6 driven wheels.
        #   3. work out where each wheel is trying to travel and by how much,
        #       and then average these to work out the net motion and
        #       separately calculate the rotation
        # With 1, we can do this one steerable wheel at a time. If the
        # steerable wheel is pointing dead ahead, then it won't be attempting
        # to turn - it will just be trying to push forwards or backwards. But
        # if it is not dead ahead, then it will be attempting to steer. The two
        # fixed middle wheels constrain it to turn around a point somewhere
        # along the imaginary line joining those two wheels together. We need
        # to calculate the size of that turning circle, because from that, we
        # can calculate the rate of turn for a given speed.
        # We have the angle and also the length of the 'opposite' (the distance
        # between the middle and front wheel), and we want the radius.
        # r*sin(a) = opp, so r = opp/(sin(a))

        def calculateSteeredPosition(left, front, wheelAngleRelativeToVehicleDegrees, wheelSpeed, dt):
            # The motor speed is just a number from 0 (not moving)
            # to 100 (full speed). We need to convert that to an
            # actual speed:
            wheelSpeedCmPerSecond = wheelSpeed / 100.0 * fullSpeedCmPerSecond

            if wheelAngleRelativeToVehicleDegrees == 0:
                # We're moving in a straight line, so we just need to work
                # out what that means given the way we're facing
                headingInRadians = (self.vehicleHeadingDegrees / 180.0) * math.pi

                distanceMovedCmSinceLastUpdate = wheelSpeedCmPerSecond * dt 
                xChangeCm = distanceMovedCmSinceLastUpdate * math.sin(headingInRadians)
                yChangeCm = distanceMovedCmSinceLastUpdate * math.cos(headingInRadians)
                headingChangeDegrees = 0

                updatedVehicleX = self.vehicleXcm + xChangeCm
                updatedVehicleY = self.vehicleYcm + yChangeCm
            else:
                # Trying to steer
                wheelDistanceFromCentreX = self.vehicleWidthCm / 2
                steerablePosRelativeToRoverX = -wheelDistanceFromCentreX if left else wheelDistanceFromCentreX
                steerablePosRelativeToRoverY = self.distanceBetweenWheelPairsCm if front else self.distanceBetweenWheelPairsCm
                distanceBetweenWheelsCm = steerablePosRelativeToRoverY

                wheelAngleRelativeToVehicleRadians = (wheelAngleRelativeToVehicleDegrees / 180.0) * math.pi
                turningRadiusToSteerableWheelCm = distanceBetweenWheelsCm / math.sin(wheelAngleRelativeToVehicleRadians)
                circumferenceCm = 2*math.pi*turningRadiusToSteerableWheelCm

                # Now work out the rate of turn, then the amount of turn given the time difference
                revolutionsPerSecond = wheelSpeedCmPerSecond / circumferenceCm
                revolutionsTurned = revolutionsPerSecond * dt
                headingChangeDegrees = revolutionsTurned * 360
                headingChangeRadians = revolutionsTurned * 2 * math.pi

                # Now work out the turning circle centre.
                turningCircleCentreDistanceFromVehicleCentre = math.cos(wheelAngleRelativeToVehicleRadians) * turningRadiusToSteerableWheelCm - steerablePosRelativeToRoverX
                vehicleHeadingRadians = math.radians(self.vehicleHeadingDegrees)
                turningCircleRelativeToVehicleX = turningCircleCentreDistanceFromVehicleCentre * math.cos(-vehicleHeadingRadians)
                turningCircleRelativeToVehicleY = turningCircleCentreDistanceFromVehicleCentre * math.sin(-vehicleHeadingRadians)
                turningCircleX = turningCircleRelativeToVehicleX + self.vehicleXcm
                turningCircleY = turningCircleRelativeToVehicleY + self.vehicleYcm

                if showSteeringCalcs:
                    print("Turning circle centre: " + str([int(turningCircleX),int(turningCircleY)]))
                

                # Work out where vehicle will go as it moves around the turning circle
                currentAngleOnTurningCircleRadians = math.atan2(self.vehicleYcm - turningCircleY, self.vehicleXcm - turningCircleX)
                updatedAngleOnTurningCircleRadians = currentAngleOnTurningCircleRadians - headingChangeRadians
                updatedVehicleX = turningCircleX + abs(turningCircleCentreDistanceFromVehicleCentre) * math.cos(updatedAngleOnTurningCircleRadians)
                updatedVehicleY = turningCircleY + abs(turningCircleCentreDistanceFromVehicleCentre) * math.sin(updatedAngleOnTurningCircleRadians)

            return [updatedVehicleX, updatedVehicleY, self.vehicleHeadingDegrees + headingChangeDegrees]

        # We'll work out where each steerable wheel is attempting to push the rover.
        # For spinning in place, we need to handle opposite wheel directions specially
        if (self.speedL > 0 and self.speedR < 0) or (self.speedL < 0 and self.speedR > 0):
            # Spinning in place
            spinSpeed = max(abs(self.speedL), abs(self.speedR))
            spinSpeedRadiansPerSecond = (spinSpeed / 100.0) * (36.0 * math.pi / 180.0)  # Full speed = 36 degrees per second
            headingChange = spinSpeedRadiansPerSecond * timeSinceLastUpdate * (180.0 / math.pi)
            if self.speedL < 0:  # Spinning left
                headingChange = -headingChange
            
            self.vehicleHeadingDegrees += headingChange
            # When spinning in place, position doesn't change
            return

        # Simplified Marsy steering model for normal movement.
        #
        # The original simulator tried to calculate the vehicle motion from
        # each steerable wheel independently and then average the result. For
        # Marsy-style steering this can cancel out: front wheels are turned one
        # way and rear wheels the opposite way, so the rover graphic shows
        # steering but the body still moves almost straight.
        #
        # This model treats the rover as a small 4-wheel-steered vehicle:
        #   - equal front/rear opposite angles create an arc turn
        #   - equal left/right motor speeds move along that arc
        #   - different left/right motor speeds also add yaw
        # It is deliberately approximate, but visually matches manual driving.

        averageSpeed = (self.speedL + self.speedR) / 2.0
        linearSpeedCmPerSecond = averageSpeed / 100.0 * fullSpeedCmPerSecond

        frontSteerDeg = (self.servos[servo_FL] + self.servos[servo_FR]) / 2.0
        rearSteerDeg = (self.servos[servo_RL] + self.servos[servo_RR]) / 2.0

        # Marsy manual steering convention:
        #   right: front +angle, rear -angle
        #   left:  front -angle, rear +angle
        effectiveSteerDeg = (frontSteerDeg - rearSteerDeg) / 2.0

        oldHeadingDegrees = self.vehicleHeadingDegrees
        headingChangeDegrees = 0.0

        # Steering yaw from wheel angle.
        if abs(linearSpeedCmPerSecond) > 0.001 and abs(effectiveSteerDeg) > 0.1:
            wheelBaseCm = max(1.0, self.distanceBetweenWheelPairsCm * 2.0)
            steeringRadians = math.radians(effectiveSteerDeg)
            headingRateRadiansPerSecond = (
                linearSpeedCmPerSecond / wheelBaseCm * math.tan(steeringRadians)
            )
            headingChangeDegrees += math.degrees(
                headingRateRadiansPerSecond * timeSinceLastUpdate
            )

        # Extra yaw from left/right motor speed difference.
        # This makes turnForward(left, right) and turnReverse(left, right)
        # visibly arc even when the steering servos are straight.
        if abs(self.speedL - self.speedR) > 0.1:
            trackWidthCm = max(1.0, self.vehicleWidthCm)
            leftSpeedCmPerSecond = self.speedL / 100.0 * fullSpeedCmPerSecond
            rightSpeedCmPerSecond = self.speedR / 100.0 * fullSpeedCmPerSecond
            differentialYawRateRadiansPerSecond = (
                (leftSpeedCmPerSecond - rightSpeedCmPerSecond) / trackWidthCm
            )
            headingChangeDegrees += math.degrees(
                differentialYawRateRadiansPerSecond * timeSinceLastUpdate
            )

        # Move using the midpoint heading for smoother arcs.
        distanceMovedCmSinceLastUpdate = linearSpeedCmPerSecond * timeSinceLastUpdate
        midHeadingDegrees = oldHeadingDegrees + headingChangeDegrees / 2.0
        midHeadingRadians = math.radians(midHeadingDegrees)

        self.vehicleXcm += distanceMovedCmSinceLastUpdate * math.sin(midHeadingRadians)
        self.vehicleYcm += distanceMovedCmSinceLastUpdate * math.cos(midHeadingRadians)
        self.vehicleHeadingDegrees = oldHeadingDegrees + headingChangeDegrees

        # Solid obstacle collision.
        #
        # The previous version simply reverted every step that ended inside an
        # obstacle. That made the rover feel "stuck": after touching an object,
        # even a reverse command could still end inside the collision radius for
        # a few frames, so it would be reverted too.
        #
        # This version blocks only movement that goes deeper into an obstacle.
        # If the rover is already touching/overlapping and the next command
        # moves it away from the obstacle center, we allow that movement so the
        # user can reverse or steer out.
        old_collision = collision_obstacle_at(old_x_cm, old_y_cm)
        new_collision = collision_obstacle_at(self.vehicleXcm, self.vehicleYcm)

        if new_collision is not None:
            obstacle = old_collision or new_collision
            obstacle_x = float(obstacle["x"])
            obstacle_y = float(obstacle["y"])
            label = obstacle.get("label", "obstacle")

            old_dx = old_x_cm - obstacle_x
            old_dy = old_y_cm - obstacle_y
            new_dx = self.vehicleXcm - obstacle_x
            new_dy = self.vehicleYcm - obstacle_y

            old_distance = math.sqrt(old_dx * old_dx + old_dy * old_dy)
            new_distance = math.sqrt(new_dx * new_dx + new_dy * new_dy)

            moving_away = old_collision is not None and new_distance > old_distance + 0.05

            if moving_away:
                # Allow escaping movement. Keep a short status note until the
                # rover is fully clear, but do not stop the motors.
                self.last_collision_label = f"clearing {label}"
            else:
                # Block the step and put the rover just outside the obstacle
                # boundary, on the side it came from. This prevents visual
                # penetration and avoids permanent overlap.
                min_distance = (
                    ROVER_BODY_RADIUS_CM
                    + float(obstacle["r"])
                    + COLLISION_MARGIN_CM
                    + 0.5
                )

                if old_distance > 0.001:
                    push_x = old_dx / old_distance
                    push_y = old_dy / old_distance
                elif new_distance > 0.001:
                    push_x = new_dx / new_distance
                    push_y = new_dy / new_distance
                else:
                    # Fallback direction if the rover center is exactly on the
                    # obstacle center.
                    push_x = 0.0
                    push_y = -1.0

                self.vehicleXcm = obstacle_x + push_x * min_distance
                self.vehicleYcm = obstacle_y + push_y * min_distance
                self.vehicleHeadingDegrees = old_heading_degrees
                self.speedL = 0
                self.speedR = 0
                self.last_collision_label = label
        elif old_collision is not None:
            # We just left a collision zone.
            self.last_collision_label = None

        # # This is a bit too basic. We need to take into
        # # account wheel servo orientation to work out how
        # # much each side moves, and in which direction,
        # # and to deduce the rotation of the rover from that.
        # # But it will do for now.
        # averageSpeed = (self.speedL + self.speedR) / 2

        # # The motor speed is just a number from 0 (not moving)
        # # to 100 (full speed). We need to convert that to an
        # # actual speed:
        # averageSpeedCmPerSecond = averageSpeed / 100.0 * fullSpeedCmPerSecond

        # # The program probably won't manage to update at exactly the
        # # same interval - when the computer's busy or running slowly for
        # # some reason, there might be longer gaps between updates. So we
        # # need to work out how far the rover will have travelled based
        # # not just on its speed, but also on how long it has been since the
        # # last update.
        # distanceMovedCmSinceLastUpdate = averageSpeedCmPerSecond * timeSinceLastUpdate 
        
        # # Of course, the rover probably isn't heading exactly up/down/left/right,
        # # so we can't just add the distance moved to vehicleXcm or vehicleYcm. We need to
        # # work out how to split the distance between those two directions based on
        # # the direction the rover is pointing. For this, we use trigonometry! Yay!
        # # First, computers always want things in Radians, not Degrees, because reasons
        # headingInRadians = (self.vehicleHeadingDegrees / 180) * math.pi
        # xChangeCm = -distanceMovedCmSinceLastUpdate * math.sin(headingInRadians)
        # yChangeCm = distanceMovedCmSinceLastUpdate * math.cos(headingInRadians)
        # self.vehicleXcm += xChangeCm
        # self.vehicleYcm += yChangeCm
        if showSteeringCalcs:
            print("X,Y: " + str(self.vehicleXcm) + ", " + str(self.vehicleYcm))
            print("Heading: " + str(self.vehicleHeadingDegrees))



class MainWindow(QWidget):
    rover = Rover()
    updateTimer = QTimer()
    mission_stderr_signal = pyqtSignal(str, str)
    mission_exit_signal = pyqtSignal(str, int)

    # Visualized rover parts
    visRoverGroup = QGraphicsItemGroup()
    visRoverWheelFL = QGraphicsItemGroup()
    visRoverWheelFR = QGraphicsItemGroup()
    visRoverWheelML = QGraphicsItemGroup()
    visRoverWheelMR = QGraphicsItemGroup()
    visRoverWheelBL = QGraphicsItemGroup()
    visRoverWheelBR = QGraphicsItemGroup()

    def __init__(self, parent=None):
        QWidget.__init__(self, parent)

        global CURRENT_ROVER
        CURRENT_ROVER = self.rover

        self._is_shutting_down = False

        self.setWindowTitle("Marsy Rover Simulator — MISSION LAUNCHER v7")
        self.setGeometry(40, 40, 1500, 860)
        self.setMinimumSize(1500, 860)
        self.setMinimumSize(1380, 820)
        self.setStyleSheet("""
            QWidget {
                background: #0f1213;
                color: #e8dfcf;
                font-family: Arial, Helvetica, sans-serif;
                font-size: 13px;
            }
            QLabel {
                background: transparent;
            }
            QPushButton {
                background-color: #171b1d;
                color: #e8dfcf;
                border: 1px solid #303639;
                border-radius: 8px;
                padding: 8px 14px;
                font-weight: 600;
            }
            QPushButton:hover {
                background-color: #202628;
                border-color: #e47b2c;
            }
        """)

        self.accent = QColor(232, 123, 44)
        self.accent_soft = QColor(245, 165, 85)
        self.panel_bg = "#121719"
        self.panel_border = "#2b3336"
        # Mission timer state.
        # This is intentionally separate from the PyQt update timer.
        # The UI can keep repainting while the displayed mission timer is frozen.
        self.start_time = time()  # kept for backward compatibility
        self.mission_timer_running = False
        self.mission_started_at = None
        self.mission_elapsed_s = 0.0

        self.is_paused = False
        self.blocked_reason = None
        self.telemetry_lines = []
        self.mission_process = None
        self.mission_process_name = None

        self.mission_stderr_signal.connect(self.on_mission_process_stderr)
        self.mission_exit_signal.connect(self.on_mission_process_exit)

        self.build_dashboard_shell()
        self.build_rover_graphics()
        self.build_scene()
        self.build_server()

        # Keep the telemetry console visually on top even if the map layout changes.
        self.telemetry_panel.show()
        if hasattr(self, "lbl_missions_title"):
            self.lbl_missions_title.show()
            self.lbl_missions_hint.show()
            self.btn_start_avoid.show()
            self.btn_manual_ui_mode.show()
            self.btn_stop_mission.show()
            self.lbl_mission_status.show()
        self.lbl_telemetry_title.show()
        self.lbl_telemetry_hint.show()
        self.telemetry_box.show()
        self.telemetry_panel.raise_()
        if hasattr(self, "lbl_missions_title"):
            self.lbl_missions_title.raise_()
            self.lbl_missions_hint.raise_()
            self.btn_start_avoid.raise_()
            self.btn_manual_ui_mode.raise_()
            self.btn_stop_mission.raise_()
            self.lbl_mission_status.raise_()
        self.lbl_telemetry_title.raise_()
        self.lbl_telemetry_hint.raise_()
        self.telemetry_box.raise_()

        self.append_telemetry("sim", "MISSION LAUNCHER v7 ready")
        self.append_telemetry("sim", "Routes: /telemetry, /log, /state, /control")

        self.updateTimer.timeout.connect(self.on_update_timer)
        self.updateTimer.start(100)

    # -----------------------------------------------------------------
    # UI shell
    # -----------------------------------------------------------------

    def make_label(self, text, x, y, w, h, *, color="#e8dfcf", size=13, bold=False, parent=None):
        label = QLabel(text, parent or self)
        weight = "700" if bold else "400"
        label.setStyleSheet(
            f"background: transparent; color: {color}; font-size: {size}px; font-weight: {weight};"
        )
        label.setGeometry(x, y, w, h)
        return label

    def make_panel(self, x, y, w, h):
        panel = QFrame(self)
        panel.setGeometry(x, y, w, h)
        panel.setStyleSheet(f"""
            QFrame {{
                background-color: {self.panel_bg};
                border: 1px solid {self.panel_border};
                border-radius: 10px;
            }}
        """)
        return panel

    def make_control_button(self, text, x, y, w, h):
        button = QPushButton(text, self)
        button.setGeometry(x, y, w, h)
        button.setStyleSheet("""
            QPushButton {
                background-color: #171b1d;
                color: #f4eee6;
                border: 1px solid #3a4245;
                border-radius: 8px;
                padding: 4px;
                font-size: 15px;
                font-weight: 700;
            }
            QPushButton:hover {
                background-color: #252b2e;
                border-color: #e87b2c;
            }
            QPushButton:pressed {
                background-color: #e87b2c;
                color: #111;
            }
        """)
        return button

    def slider_style(self):
        return """
            QSlider::groove:horizontal {
                height: 4px;
                background: #384044;
                border-radius: 2px;
            }
            QSlider::sub-page:horizontal {
                background: #e87b2c;
                border-radius: 2px;
            }
            QSlider::handle:horizontal {
                background: #f5a555;
                width: 14px;
                margin: -5px 0;
                border-radius: 7px;
                border: 1px solid #2a170d;
            }
        """

    def on_range_changed(self, value_cm):
        global SONAR_MAX_RANGE_CM
        SONAR_MAX_RANGE_CM = float(value_cm)
        self.lbl_range_value.setText(f"{value_cm / 100.0:.1f} m")

    def on_fov_changed(self, value_deg):
        global SONAR_FOV_DEG, SONAR_BEAM_RADIUS_CM
        SONAR_FOV_DEG = float(value_deg)
        # Keep a minimum physical beam width, but make wider FOV slightly more forgiving.
        SONAR_BEAM_RADIUS_CM = max(4.0, value_deg / 8.0)
        self.lbl_fov_value.setText(f"{value_deg}°")

    def stop_motors(self):
        self.rover.setWheelMotorLeft(0, 0)
        self.rover.setWheelMotorRight(0, 0)

    def set_wheel_servos(self, fl, fr, rl, rr):
        self.rover.setServo(servo_FL, fl)
        self.rover.setServo(servo_FR, fr)
        self.rover.setServo(servo_RL, rl)
        self.rover.setServo(servo_RR, rr)

    # -----------------------------------------------------------------
    # Mission timer
    # -----------------------------------------------------------------

    def reset_mission_timer(self):
        """Reset and pause the mission timer shown in the bottom bar."""
        self.mission_timer_running = False
        self.mission_started_at = None
        self.mission_elapsed_s = 0.0
        self.update_mission_timer_label()

    def start_mission_timer(self, *, reset=False):
        """Start/resume the mission timer. Use reset=True for a new mission."""
        if reset:
            self.mission_elapsed_s = 0.0

        if not self.mission_timer_running:
            self.mission_started_at = time() - self.mission_elapsed_s
            self.mission_timer_running = True

        self.update_mission_timer_label()

    def stop_mission_timer(self):
        """Freeze the mission timer at the current elapsed time."""
        if self.mission_timer_running and self.mission_started_at is not None:
            self.mission_elapsed_s = max(0.0, time() - self.mission_started_at)

        self.mission_timer_running = False
        self.mission_started_at = None
        self.update_mission_timer_label()

    def current_mission_elapsed_s(self):
        if self.mission_timer_running and self.mission_started_at is not None:
            return max(0.0, time() - self.mission_started_at)
        return max(0.0, self.mission_elapsed_s)

    def update_mission_timer_label(self):
        elapsed = int(self.current_mission_elapsed_s())
        hh = elapsed // 3600
        mm = (elapsed % 3600) // 60
        ss = elapsed % 60

        status = "RUN" if self.mission_timer_running else "STOP"

        if hasattr(self, "lbl_sim_time"):
            self.lbl_sim_time.setText(f"MISSION TIME\n{hh:02d}:{mm:02d}:{ss:02d}  {status}")

    def current_effective_steer_deg(self):
        """
        Return the current Marsy-style effective steering angle.

        Marsy turns by steering the front and rear wheel pairs in opposite
        directions. If the wheels are straight, this returns approximately 0.
        If the rover is prepared to drive an arc, this returns a non-zero angle.
        """
        front = (self.rover.servos[servo_FL] + self.rover.servos[servo_FR]) / 2.0
        rear = (self.rover.servos[servo_RL] + self.rover.servos[servo_RR]) / 2.0
        return (front - rear) / 2.0

    def is_prepared_to_turn(self):
        """Return True when the steering wheels are turned enough for an escape arc."""
        return abs(self.current_effective_steer_deg()) >= 5.0

    def set_mode(self, mode, control_source=None, mission_name=None):
        """Update visible mode and the shared /state control block."""
        mode_clean = str(mode or "manual").strip().lower()
        SIM_CONTROL_STATE["mode"] = mode_clean

        if control_source is not None:
            SIM_CONTROL_STATE["control_source"] = control_source
        if mission_name is not None:
            SIM_CONTROL_STATE["mission_name"] = mission_name

        label_map = {
            "manual": "MANUAL",
            "auto": "AUTO",
            "blocked": "BLOCKED",
            "stopped": "STOPPED",
        }
        if hasattr(self, "lbl_mode"):
            self.lbl_mode.setText(f"MODE\n{label_map.get(mode_clean, mode_clean.upper())}")
        if hasattr(self, "lbl_control_source"):
            self.lbl_control_source.setText(f"CONTROL\n{SIM_CONTROL_STATE.get('control_source') or 'UI / API'}")

    def clear_mission_stop_request(self):
        SIM_CONTROL_STATE["mission_stop_requested"] = False
        SIM_CONTROL_STATE["mission_stop_reason"] = None

    def request_mission_stop(self, reason="ui_stop"):
        """Top STOP: stop motors, freeze timer, and ask external missions to exit."""
        self.stop_motors()
        self.stop_mission_timer()
        SIM_CONTROL_STATE["mission_stop_requested"] = True
        SIM_CONTROL_STATE["mission_stop_reason"] = reason
        self.set_mode("stopped", control_source="UI STOP")
        self.append_telemetry("ui", "Mission stop requested", "warning")
        if self.is_mission_process_running():
            self.set_mission_status_label(f"Mission: stopping {self.mission_process_name}...")
        else:
            self.set_mission_status_label("Mission: stop requested")

    def set_manual_mode(self):
        self.is_paused = False
        self.blocked_reason = None
        self.clear_mission_stop_request()
        self.set_mode("manual", control_source="UI / API")

    def set_auto_mode(self, mission_name="mission"):
        self.is_paused = False
        self.blocked_reason = None
        SIM_CONTROL_STATE["mission_stop_requested"] = False
        SIM_CONTROL_STATE["mission_stop_reason"] = None
        self.start_mission_timer(reset=False)
        self.set_mode("auto", control_source="AUTO", mission_name=mission_name)

    def set_blocked_mode(self, reason="obstacle"):
        self.blocked_reason = reason
        self.stop_motors()
        self.set_mode("blocked", control_source="UI / API")

    def clear_blocked_mode_if_safe(self, distance_cm):
        if self.blocked_reason is None:
            return

        # Leave BLOCKED mode when either:
        #   - the sonar no longer sees a close obstacle, or
        #   - the steering is turned, so the next forward command is an escape arc.
        # Reverse / stop / steering buttons also call set_manual_mode() directly.
        if (
            distance_cm <= 0
            or distance_cm >= UI_SAFE_DISTANCE_CM
            or self.is_prepared_to_turn()
        ):
            self.set_manual_mode()

    def on_ui_forward_clicked(self):
        distance_cm, _ = compute_ultrasonic_distance(self.rover)

        # If the obstacle is directly ahead and the wheels are straight, block
        # forward movement. If the wheels are already turned, allow a slower
        # forward arc so the user can steer out of BLOCKED state.
        #
        # Important distinction:
        #   turning the wheels does not rotate the rover body or sonar instantly,
        #   so the sonar may still report a close obstacle until the rover has
        #   moved along the arc.
        if 0 < distance_cm < UI_SAFE_DISTANCE_CM and not self.is_prepared_to_turn():
            self.set_blocked_mode("obstacle")
            self.append_telemetry("ui", f"Forward blocked: obstacle at {distance_cm:.1f} cm", "warning")
            if hasattr(self, "lbl_status_distance"):
                self.lbl_status_distance.setText(f"BLOCKED\n{distance_cm:.1f} cm")
            return

        if 0 < distance_cm < UI_SAFE_DISTANCE_CM and self.is_prepared_to_turn():
            # Cautious escape arc near an obstacle.
            command_speed = min(SIM_MANUAL_SPEED, 28)
        else:
            command_speed = SIM_MANUAL_SPEED

        self.rover.setWheelMotorLeft(command_speed, 0)
        self.rover.setWheelMotorRight(command_speed, 0)
        self.set_manual_mode()
        self.append_telemetry("ui", f"Forward speed={command_speed}")

    def on_ui_reverse_clicked(self):
        self.rover.setWheelMotorLeft(0, SIM_MANUAL_SPEED)
        self.rover.setWheelMotorRight(0, SIM_MANUAL_SPEED)
        self.set_manual_mode()
        self.append_telemetry("ui", f"Reverse speed={SIM_MANUAL_SPEED}")

    def on_ui_stop_clicked(self):
        # Manual-side STOP: stop motors only and remain in manual control.
        self.stop_motors()
        self.set_manual_mode()
        self.append_telemetry("ui", "Manual stop")

    def on_top_stop_clicked(self):
        # Top STOP: emergency/mission stop. This is visible to autonomous
        # missions through /state so avoid_obstacle can exit cleanly.
        self.request_mission_stop("top_stop_button")

    def on_ui_steer_left_clicked(self):
        angle = SIM_STEER_ANGLE_DEG
        self.set_wheel_servos(-angle, -angle, angle, angle)
        self.set_manual_mode()
        self.append_telemetry("ui", f"Steer left angle={angle}")

    def on_ui_steer_right_clicked(self):
        angle = SIM_STEER_ANGLE_DEG
        self.set_wheel_servos(angle, angle, -angle, -angle)
        self.set_manual_mode()
        self.append_telemetry("ui", f"Steer right angle={angle}")

    def on_ui_center_wheels_clicked(self):
        self.set_wheel_servos(0, 0, 0, 0)
        self.set_manual_mode()
        self.append_telemetry("ui", "Center wheels")

    def on_reset_clicked(self):
        self.rover.reset()
        self.set_mission_status_label("Mission: none")
        self.append_telemetry("ui", "Reset rover pose and clear mission stop")
        self.is_paused = False
        self.start_time = time()
        self.reset_mission_timer()
        self.set_manual_mode()
        self.roverIcon.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def build_dashboard_shell(self):
        # Top bar
        self.top_bar = QFrame(self)
        self.top_bar.setGeometry(0, 0, 1500, 62)
        self.top_bar.setStyleSheet("""
            QFrame {
                background-color: #0d1011;
                border-bottom: 1px solid #272d30;
            }
        """)

        self.make_label("☄", 26, 12, 36, 36, color="#e87b2c", size=26, bold=True)
        self.make_label("MARSY", 64, 10, 180, 28, color="#f4eee6", size=22, bold=True)
        self.make_label("ROVER SIMULATOR", 66, 36, 180, 18, color="#e87b2c", size=10, bold=True)
        self.make_label("TOP-DOWN SONAR VIEW", 270, 22, 220, 20, color="#c5b9aa", size=12, bold=True)

        self.btn_stop_top = QPushButton("■  STOP", self)
        self.btn_stop_top.setGeometry(1230, 18, 104, 34)
        self.btn_reset = QPushButton("⟳  RESET", self)
        self.btn_reset.setGeometry(1350, 18, 110, 34)

        self.btn_stop_top.clicked.connect(self.on_top_stop_clicked)
        self.btn_reset.clicked.connect(self.on_reset_clicked)

        # Left sidebar
        self.sidebar = QFrame(self)
        self.sidebar.setGeometry(14, 76, 190, 674)
        self.sidebar.setStyleSheet(f"""
            QFrame {{
                background-color: #101517;
                border: 1px solid {self.panel_border};
                border-radius: 12px;
            }}
        """)

        self.make_label("SENSORS", 28, 94, 150, 22, color="#e87b2c", size=12, bold=True)
        self.make_label("◉  Sonar", 34, 136, 90, 22, color="#e8dfcf", size=13)
        self.make_label("●", 170, 138, 18, 20, color="#e87b2c", size=15)
        self.make_label("Range", 34, 178, 80, 18, color="#c5b9aa", size=12)
        self.lbl_range_value = self.make_label("2.0 m", 154, 178, 45, 18, color="#d8cbb8", size=12)
        self.slider_range = QSlider(Qt.Orientation.Horizontal, self)
        self.slider_range.setGeometry(34, 200, 150, 24)
        self.slider_range.setRange(50, 300)  # centimeters
        self.slider_range.setValue(int(SONAR_MAX_RANGE_CM))
        self.slider_range.setStyleSheet(self.slider_style())
        self.slider_range.valueChanged.connect(self.on_range_changed)

        self.make_label("FOV", 34, 236, 80, 18, color="#c5b9aa", size=12)
        self.lbl_fov_value = self.make_label("35°", 158, 236, 45, 18, color="#d8cbb8", size=12)
        self.slider_fov = QSlider(Qt.Orientation.Horizontal, self)
        self.slider_fov.setGeometry(34, 258, 150, 24)
        self.slider_fov.setRange(5, 75)  # degrees
        self.slider_fov.setValue(int(SONAR_FOV_DEG))
        self.slider_fov.setStyleSheet(self.slider_style())
        self.slider_fov.valueChanged.connect(self.on_fov_changed)

        self.make_label("MANUAL CONTROL", 28, 306, 150, 22, color="#e87b2c", size=12, bold=True)
        self.btn_ui_forward = self.make_control_button("▲", 82, 342, 52, 38)
        self.btn_ui_left = self.make_control_button("◀", 28, 386, 52, 38)
        self.btn_ui_center = self.make_control_button("C", 82, 386, 52, 38)
        self.btn_ui_right = self.make_control_button("▶", 136, 386, 52, 38)
        self.btn_ui_reverse = self.make_control_button("▼", 82, 430, 52, 38)
        self.btn_ui_stop = self.make_control_button("STOP", 34, 482, 148, 30)

        self.btn_ui_forward.clicked.connect(self.on_ui_forward_clicked)
        self.btn_ui_reverse.clicked.connect(self.on_ui_reverse_clicked)
        self.btn_ui_left.clicked.connect(self.on_ui_steer_left_clicked)
        self.btn_ui_right.clicked.connect(self.on_ui_steer_right_clicked)
        self.btn_ui_stop.clicked.connect(self.on_ui_stop_clicked)
        self.btn_ui_center.clicked.connect(self.on_ui_center_wheels_clicked)

        self.make_label("ROVER STATUS", 28, 540, 150, 22, color="#e87b2c", size=12, bold=True)
        self.lbl_position = self.make_label("⌖  Position\n     x: 0.00 m\n     y: 0.00 m", 34, 574, 150, 58, color="#d8cbb8", size=12)
        self.lbl_heading = self.make_label("◴  Heading\n     0°", 34, 632, 150, 38, color="#d8cbb8", size=12)
        self.lbl_speed = self.make_label("◷  Speed\n     0.00 m/s", 34, 676, 150, 38, color="#d8cbb8", size=12)
        self.lbl_battery = self.make_label("▯  Battery  87%", 34, 720, 150, 22, color="#d8cbb8", size=12)

        # Right panel: mission launcher + mission telemetry
        self.telemetry_panel = QFrame(self)
        self.telemetry_panel.setGeometry(1010, 76, 470, 674)
        self.telemetry_panel.setStyleSheet(f"""
            QFrame {{
                background-color: #080c0d;
                border: 2px solid #e87b2c;
                border-radius: 12px;
            }}
        """)

        self.lbl_missions_title = self.make_label("MISSIONS", 1032, 94, 390, 24, color="#ff8a2a", size=15, bold=True)
        self.lbl_missions_hint = self.make_label("start autonomous runs from simulator", 1032, 120, 400, 18, color="#8f9a9e", size=10)

        self.btn_start_avoid = QPushButton("▶  AVOID", self)
        self.btn_start_avoid.setGeometry(1032, 148, 170, 34)
        self.btn_start_avoid.clicked.connect(self.on_start_obstacle_avoidance_clicked)

        self.btn_manual_ui_mode = QPushButton("MANUAL", self)
        self.btn_manual_ui_mode.setGeometry(1216, 148, 104, 34)
        self.btn_manual_ui_mode.clicked.connect(self.on_manual_ui_mode_clicked)

        self.btn_stop_mission = QPushButton("■  STOP", self)
        self.btn_stop_mission.setGeometry(1336, 148, 116, 34)
        self.btn_stop_mission.clicked.connect(self.on_top_stop_clicked)

        self.lbl_mission_status = self.make_label("Mission: none", 1032, 190, 420, 20, color="#d8cbb8", size=11)

        self.lbl_telemetry_title = self.make_label("MISSION TELEMETRY", 1032, 224, 390, 24, color="#ff8a2a", size=15, bold=True)
        self.lbl_telemetry_hint = self.make_label("mission / manual / API logs", 1032, 250, 400, 18, color="#8f9a9e", size=10)
        self.telemetry_box = QPlainTextEdit(self)
        self.telemetry_box.setGeometry(1032, 278, 420, 444)
        self.telemetry_box.setReadOnly(True)
        self.telemetry_box.setMaximumBlockCount(120)
        self.telemetry_box.setStyleSheet("""
            QPlainTextEdit {
                background-color: #070b0c;
                color: #f4eee6;
                border: 2px solid #e87b2c;
                border-radius: 8px;
                padding: 8px;
                font-family: Menlo, Monaco, Consolas, monospace;
                font-size: 11px;
            }
        """)

        # Bottom status bar
        self.status_bar = QFrame(self)
        self.status_bar.setGeometry(0, 760, 1500, 60)
        self.status_bar.setStyleSheet("""
            QFrame {
                background-color: #0d1011;
                border-top: 1px solid #272d30;
            }
        """)
        self.lbl_mode = self.make_label("MODE\nMANUAL", 78, 770, 140, 44, color="#e8dfcf", size=12)
        self.lbl_sim_time = self.make_label("MISSION TIME\n00:00:00  STOP", 330, 770, 205, 44, color="#e8dfcf", size=12)
        self.lbl_status_distance = self.make_label("DISTANCE\n0.0 cm", 610, 770, 190, 44, color="#e8dfcf", size=12)
        self.lbl_control_source = self.make_label("CONTROL\nUI / API", 1245, 770, 140, 44, color="#e87b2c", size=12, bold=True)

    # -----------------------------------------------------------------
    # Scene and rover visuals
    # -----------------------------------------------------------------

    def build_rover_graphics(self):
        vw = self.rover.vehicleWidthCm
        vh = self.rover.vehicleHeightCm

        # Rover body
        body = QGraphicsRectItem(QRectF(-vw / 2, -vh / 2, vw, vh))
        body.setPen(QPen(QColor(30, 30, 30), 1))
        body.setBrush(QBrush(QColor(210, 212, 205)))
        self.visRoverGroup.addToGroup(body)

        # Solar panel / top plate
        panel = QGraphicsRectItem(QRectF(-4, -7, 8, 12))
        panel.setPen(QPen(QColor(60, 70, 72), 1))
        panel.setBrush(QBrush(QColor(70, 80, 82)))
        self.visRoverGroup.addToGroup(panel)

        # Small mast/sensor dot
        mast = QGraphicsRectItem(QRectF(-2, -12, 4, 4))
        mast.setPen(QPen(QColor(245, 165, 85), 1))
        mast.setBrush(QBrush(QColor(245, 165, 85)))
        self.visRoverGroup.addToGroup(mast)

        def makeWheel(front, left, wheelRotatingContainer):
            wheelNonRotatingContainer = QGraphicsItemGroup(self.visRoverGroup)
            wx = vw / 2 + 5
            wy = vh / 2 - 2
            x = -wx if left else wx
            y = -wy if front else wy
            wheel = QGraphicsRectItem(QRectF(-2.5, -4.5, 5, 9))
            wheel.setPen(QPen(QColor(220, 220, 210), 0.8))
            wheel.setBrush(QBrush(QColor(12, 15, 16)))
            wheelRotatingContainer.addToGroup(wheel)
            wheelNonRotatingContainer.addToGroup(wheelRotatingContainer)
            wheelNonRotatingContainer.setTransform(QTransform.fromTranslate(x, y))
            return wheelNonRotatingContainer

        def makeFixedMiddleWheel(left, wheelContainer):
            wheelNonRotatingContainer = QGraphicsItemGroup(self.visRoverGroup)
            wx = vw / 2 + 5
            x = -wx if left else wx
            y = 0
            wheel = QGraphicsRectItem(QRectF(-2.5, -4.5, 5, 9))
            wheel.setPen(QPen(QColor(220, 220, 210), 0.8))
            wheel.setBrush(QBrush(QColor(12, 15, 16)))
            wheelContainer.addToGroup(wheel)
            wheelNonRotatingContainer.addToGroup(wheelContainer)
            wheelNonRotatingContainer.setTransform(QTransform.fromTranslate(x, y))
            return wheelNonRotatingContainer

        makeWheel(True, True, self.visRoverWheelFL)
        makeWheel(True, False, self.visRoverWheelFR)
        makeFixedMiddleWheel(True, self.visRoverWheelML)
        makeFixedMiddleWheel(False, self.visRoverWheelMR)
        makeWheel(False, True, self.visRoverWheelBL)
        makeWheel(False, False, self.visRoverWheelBR)

    def build_scene(self):
        self.scene = QGraphicsScene()
        self.scene.setSceneRect(QRectF(-200, -150, 400, 300))

        # Outer terrain panel
        self.scene.addRect(
            QRectF(-200, -150, 400, 300),
            QPen(QColor(58, 43, 34), 2),
            QBrush(QColor(143, 69, 29)),
        )

        # Terrain texture: deterministic pebbles and subtle speckles.
        # No random module needed, so every run looks the same.
        for i in range(170):
            x = -195 + ((i * 37) % 390)
            y = -145 + ((i * 73) % 290)
            radius = 0.7 + ((i * 11) % 22) / 18.0
            color = QColor(105 + (i % 35), 55 + (i % 25), 28 + (i % 15), 95)
            self.scene.addEllipse(
                QRectF(x - radius, y - radius, radius * 2, radius * 2),
                QPen(QColor(80, 43, 25, 70), 0.3),
                QBrush(color),
            ).setZValue(0.1)

        self.add_world_obstacles(self.scene)

        # Sonar ray visualization. It is updated in on_update_timer().
        self.visSonarRay = self.scene.addLine(
            0, 0, 0, 0,
            QPen(QColor(245, 165, 85, 180), 1.2),
        )
        self.visSonarRay.setZValue(2)

        self.visRoverGroup.setZValue(3)
        self.scene.addItem(self.visRoverGroup)

        self.roverIcon = QGraphicsView(self.scene, parent=self)
        self.roverIcon.setGeometry(220, 76, 770, 674)
        self.roverIcon.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.roverIcon.setStyleSheet("""
            QGraphicsView {
                background-color: #0c0f10;
                border: 1px solid #303639;
                border-radius: 12px;
            }
        """)
        self.roverIcon.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def add_world_obstacles(self, scene):
        """
        Draw static circular obstacles in the simulated world.
        """
        obstacle_pen = QPen(QColor(110, 52, 22), 2)
        obstacle_brush = QBrush(QColor(164, 88, 36))
        label_color = QColor(245, 230, 205)
        halo_pen = QPen(QColor(232, 123, 44, 165), 1.4)

        for obstacle in MARSY_OBSTACLES:
            if obstacle.get("type") != "circle":
                continue

            x_cm = obstacle["x"]
            y_cm = obstacle["y"]
            r_cm = obstacle["r"]
            x_scene = x_cm
            y_scene = -y_cm

            halo_r_cm = r_cm + SONAR_BEAM_RADIUS_CM
            halo = scene.addEllipse(
                QRectF(x_scene - halo_r_cm, y_scene - halo_r_cm, 2 * halo_r_cm, 2 * halo_r_cm),
                halo_pen,
                QBrush(Qt.BrushStyle.NoBrush),
            )
            halo.setZValue(1.0)

            # Shadow
            shadow = scene.addEllipse(
                QRectF(x_scene - r_cm + 2, y_scene - r_cm + 3, 2 * r_cm, 2 * r_cm),
                QPen(Qt.PenStyle.NoPen),
                QBrush(QColor(35, 20, 10, 90)),
            )
            shadow.setZValue(0.8)

            # Main obstacle body
            body = scene.addEllipse(
                QRectF(x_scene - r_cm, y_scene - r_cm, 2 * r_cm, 2 * r_cm),
                obstacle_pen,
                obstacle_brush,
            )
            body.setZValue(1.2)

            # Simple crater effect for crater label
            if obstacle.get("label") == "crater":
                inner = scene.addEllipse(
                    QRectF(x_scene - r_cm * 0.55, y_scene - r_cm * 0.55, 1.1 * r_cm, 1.1 * r_cm),
                    QPen(QColor(85, 42, 22), 1),
                    QBrush(QColor(95, 45, 24)),
                )
                inner.setZValue(1.3)

            label = obstacle.get("label")
            if label:
                text_item = scene.addText(label)
                text_item.setDefaultTextColor(label_color)
                text_item.setScale(0.72)
                text_item.setPos(x_scene - r_cm * 0.55, y_scene - r_cm - 18)
                text_item.setZValue(1.8)

    # -----------------------------------------------------------------
    # Mission launcher
    # -----------------------------------------------------------------

    def project_root(self):
        """Return project root for python -m mission launches."""
        try:
            return Path(__file__).resolve().parents[1]
        except Exception:
            return Path.cwd()

    def is_mission_process_running(self):
        return self.mission_process is not None and self.mission_process.poll() is None

    def set_mission_status_label(self, text):
        if hasattr(self, "lbl_mission_status"):
            self.lbl_mission_status.setText(str(text))

    def start_mission_process(self, mission_key):
        """Launch a simulator mission as a child process."""
        mission_key = str(mission_key).strip().lower()

        missions = {
            "avoid_obstacle": {
                "label": "Obstacle avoidance",
                "module": "missions.avoid_obstacle",
                "mode": "auto",
            },
        }

        if mission_key not in missions:
            self.append_telemetry("ui", f"Unknown mission: {mission_key}", "warning")
            return

        if self.is_mission_process_running():
            self.append_telemetry(
                "ui",
                f"Mission already running: {self.mission_process_name}",
                "warning",
            )
            return

        mission = missions[mission_key]
        self.clear_mission_stop_request()
        self.start_mission_timer(reset=True)
        self.set_mode(mission["mode"], control_source="UI MISSION", mission_name=mission_key)
        self.set_mission_status_label(f"Mission: {mission['label']} starting...")

        env = os.environ.copy()
        env["MARSY_MODE"] = "sim"
        env["PYTHONUNBUFFERED"] = "1"
        env.setdefault("MARSY_TELEMETRY_TO_SIM", "1")

        cmd = [sys.executable, "-m", mission["module"]]

        try:
            self.mission_process = subprocess.Popen(
                cmd,
                cwd=str(self.project_root()),
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
        except Exception as exc:
            self.mission_process = None
            self.mission_process_name = None
            self.stop_mission_timer()
            self.set_mode("stopped", control_source="MISSION ERROR", mission_name=mission_key)
            self.set_mission_status_label("Mission: failed to start")
            self.append_telemetry("ui", f"Failed to start {mission['label']}: {exc}", "error")
            return

        self.mission_process_name = mission_key
        self.set_mission_status_label(f"Mission: {mission['label']} running")
        self.append_telemetry("ui", f"Started mission: {mission['label']}")

        if self.mission_process.stderr is not None:
            threading.Thread(
                target=self._read_mission_stderr,
                args=(mission_key, self.mission_process.stderr),
                daemon=True,
            ).start()

        threading.Thread(
            target=self._watch_mission_process,
            args=(mission_key, self.mission_process),
            daemon=True,
        ).start()

    def _read_mission_stderr(self, mission_key, stream):
        try:
            for line in stream:
                line = line.strip()
                if line:
                    self.mission_stderr_signal.emit(mission_key, line)
        except Exception as exc:
            self.mission_stderr_signal.emit(mission_key, f"stderr reader failed: {exc}")

    def _watch_mission_process(self, mission_key, process):
        try:
            return_code = process.wait()
        except Exception:
            return_code = -999
        self.mission_exit_signal.emit(mission_key, int(return_code))

    def on_mission_process_stderr(self, mission_key, line):
        self.append_telemetry(mission_key, line, "error")

    def on_mission_process_exit(self, mission_key, return_code):
        if self.mission_process_name != mission_key:
            return

        self.mission_process = None
        self.mission_process_name = None
        self.stop_mission_timer()

        if return_code == 0:
            self.append_telemetry("ui", f"Mission finished: {mission_key}")
            self.set_mission_status_label("Mission: none")
            self.set_mode("stopped", control_source="MISSION DONE", mission_name=mission_key)
        else:
            self.append_telemetry("ui", f"Mission exited with code {return_code}: {mission_key}", "warning")
            self.set_mission_status_label(f"Mission: exited code {return_code}")
            self.set_mode("stopped", control_source="MISSION EXIT", mission_name=mission_key)

    def on_start_obstacle_avoidance_clicked(self):
        self.start_mission_process("avoid_obstacle")

    def on_manual_ui_mode_clicked(self):
        if self.is_mission_process_running():
            self.request_mission_stop(reason="switch_to_manual_ui")
            self.append_telemetry("ui", "Requested mission stop before switching to manual UI", "warning")
            return

        self.set_manual_mode()
        self.stop_mission_timer()
        self.set_mission_status_label("Mission: manual UI control")
        self.append_telemetry("ui", "Manual UI control active")

    # -----------------------------------------------------------------
    # Telemetry panel
    # -----------------------------------------------------------------

    def append_telemetry(self, source, message, level="info"):
        """Append one log line to the in-window telemetry console."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        source = str(source or "mission")
        level = str(level or "info").upper()
        message = str(message or "")

        line = f"[{timestamp}] {source} | {level:<5} | {message}"
        self.telemetry_lines.append(line)
        self.telemetry_lines = self.telemetry_lines[-120:]

        if hasattr(self, "telemetry_box"):
            self.telemetry_box.setPlainText("\n".join(self.telemetry_lines))
            scrollbar = self.telemetry_box.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())

    def on_telemetry(self, s):
        """Receive mission logs posted to /telemetry or /log."""
        try:
            data = json.loads(s)
        except Exception:
            self.append_telemetry("external", s, "info")
            return

        source = data.get("source", "mission")
        event = data.get("event")
        level = data.get("level", "info")
        message = data.get("message", "")

        self.append_telemetry(
            source=source,
            message=message,
            level=level,
        )

        # Mission telemetry is also the mode/timer signal for the simulator UI.
        if source == "avoid_obstacle":
            if event == "mission_started":
                self.start_mission_timer(reset=True)
                if not SIM_CONTROL_STATE.get("mission_stop_requested", False):
                    self.set_auto_mode("avoid_obstacle")
                    self.set_mission_status_label("Mission: Obstacle avoidance running")
                self.set_mission_status_label("Mission: Obstacle avoidance running")
            elif event in {"telemetry_active", "mission_config", "path_clear", "path_clear_no_echo", "obstacle_detected", "scan_started", "scan_result", "direction_chosen", "turn_left", "turn_right", "reverse_briefly", "maneuver_interrupted"}:
                if not SIM_CONTROL_STATE.get("mission_stop_requested", False):
                    self.set_auto_mode("avoid_obstacle")
            elif event in {"mission_stop_requested", "mission_stopped_by_ui", "mission_interrupted", "run_finished", "mission_cleaned_up", "shutdown_failed"}:
                self.stop_mission_timer()
                self.stop_motors()
                self.set_mode("stopped", control_source="AUTO STOP", mission_name="avoid_obstacle")
                self.set_mission_status_label("Mission: none")

        elif source == "manual_drive":
            if event == "manual_started":
                self.start_mission_timer(reset=True)
                self.set_mode("manual", control_source="MANUAL DRIVE", mission_name="manual_drive")
                self.set_mission_status_label("Mission: manual_drive running")
            elif event in {"manual_ui_stop_requested", "keyboard_interrupt", "quit", "cleanup_done", "cleanup_failed"}:
                self.stop_mission_timer()
                self.stop_motors()
                self.set_mode("stopped", control_source="MANUAL STOP", mission_name="manual_drive")
                self.set_mission_status_label("Mission: none")
            elif not SIM_CONTROL_STATE.get("mission_stop_requested", False):
                # Do not call set_manual_mode() here: it clears the top STOP
                # request. Regular manual telemetry should update the label
                # without cancelling an already-requested mission stop.
                self.set_mode("manual", control_source="MANUAL DRIVE", mission_name="manual_drive")

    # -----------------------------------------------------------------
    # Server and runtime behavior
    # -----------------------------------------------------------------

    def build_server(self):
        self.server = ServerWorker()
        self.serverThread = QThread()
        self.server.moveToThread(self.serverThread)
        self.serverThread.started.connect(self.server.run)
        self.server.mysignal.connect(self.on_change)
        self.server.telemetry_signal.connect(self.on_telemetry)
        self.serverThread.start()

    def shutdown(self):
        """
        Stop the simulator UI timer and Flask server thread.
        """
        if getattr(self, "_is_shutting_down", False):
            return

        self._is_shutting_down = True
        try:
            self.updateTimer.stop()
        except Exception:
            pass

        try:
            if self.serverThread.isRunning():
                self.serverThread.terminate()
                self.serverThread.wait(1000)
        except Exception:
            pass

    def closeEvent(self, event):
        self.shutdown()
        event.accept()

    def on_change(self, s):
        data = json.loads(s)

        if 'servos' in data:
            servos = data['servos']
            for servo in servos:
                servoId = int(servo)
                self.rover.setServo(servoId, servos[servo])

        if 'wheelMotors' in data:
            if SIM_CONTROL_STATE.get("mission_stop_requested", False):
                # Top STOP has priority over mission/API motor commands.
                self.stop_motors()
                self.set_mode("stopped", control_source="UI STOP")
                return

            wheelMotors = data['wheelMotors']
            if 'l' in wheelMotors:
                [fwd, rev] = wheelMotors['l']
                self.rover.setWheelMotorLeft(fwd, rev)
            if 'r' in wheelMotors:
                [fwd, rev] = wheelMotors['r']
                self.rover.setWheelMotorRight(fwd, rev)
            self.is_paused = False
            if SIM_CONTROL_STATE.get("mode") != "auto":
                self.set_mode("manual", control_source="API")

        if 'rgbLeds' in data:
            rgbLeds = data['rgbLeds']
            for led in rgbLeds:
                ledId = int(led)
                self.rover.setRgbLed(ledId, rgbLeds[led])

    def on_update_timer(self):
        self.rover.updateState()

        tx = QTransform()
        tx.translate(self.rover.vehicleXcm, -self.rover.vehicleYcm)
        tx.rotate(self.rover.vehicleHeadingDegrees)
        self.visRoverGroup.setTransform(tx)

        self.visRoverWheelFL.setTransform(QTransform().rotate(self.rover.servos[servo_FL]))
        self.visRoverWheelFR.setTransform(QTransform().rotate(self.rover.servos[servo_FR]))
        self.visRoverWheelBL.setTransform(QTransform().rotate(self.rover.servos[servo_RL]))
        self.visRoverWheelBR.setTransform(QTransform().rotate(self.rover.servos[servo_RR]))

        # Update sonar ray visualization.
        distance_cm, _ = compute_ultrasonic_distance(self.rover)
        ox, oy, dx, dy, _ = sonar_origin_and_direction(self.rover)
        ray_length = distance_cm if distance_cm > 0 else SONAR_MAX_RANGE_CM
        end_x = ox + dx * ray_length
        end_y = oy + dy * ray_length
        self.visSonarRay.setLine(ox, -oy, end_x, -end_y)

        if distance_cm > 0:
            self.visSonarRay.setPen(QPen(QColor(245, 165, 85, 220), 1.6))
        else:
            self.visSonarRay.setPen(QPen(QColor(150, 110, 65, 160), 1.0))

        # Update sidebar/bottom telemetry.
        avg_speed = (abs(self.rover.speedL) + abs(self.rover.speedR)) / 2.0
        speed_m_s = avg_speed / 100.0 * fullSpeedCmPerSecond / 100.0
        # Mission timer is controlled by mission telemetry / top STOP.
        # Keep repainting the simulator, but do not advance the label when
        # the mission timer is stopped.
        self.update_mission_timer_label()

        self.lbl_position.setText(
            f"⌖  Position\n     x: {self.rover.vehicleXcm / 100.0: .2f} m\n     y: {self.rover.vehicleYcm / 100.0: .2f} m"
        )
        self.lbl_heading.setText(f"◴  Heading\n     {self.rover.vehicleHeadingDegrees % 360:.0f}°")
        self.lbl_speed.setText(f"◷  Speed\n     {speed_m_s:.2f} m/s")
        if getattr(self.rover, "last_collision_label", None):
            self.lbl_status_distance.setText(
                f"COLLISION\n{self.rover.last_collision_label}"
            )
        elif self.blocked_reason is not None and 0 < distance_cm < UI_SAFE_DISTANCE_CM:
            self.lbl_status_distance.setText(f"BLOCKED\n{distance_cm:.1f} cm")
        else:
            self.lbl_status_distance.setText(f"DISTANCE\n{distance_cm:.1f} cm")
            self.clear_blocked_mode_if_safe(distance_cm)
        self.update_mission_timer_label()

app = QApplication([])

window = MainWindow()

def handle_sigint(signum, frame):
    """Handle Ctrl+C without leaving the simulator server on port 8523."""
    print("\nCtrl+C received. Closing Marsy simulator...", flush=True)
    app.quit()

signal.signal(signal.SIGINT, handle_sigint)
app.aboutToQuit.connect(window.shutdown)

window.show()

try:
    sys.exit(app.exec())
except KeyboardInterrupt:
    print("\nKeyboardInterrupt. Closing Marsy simulator...", flush=True)
    window.shutdown()
    sys.exit(0)
