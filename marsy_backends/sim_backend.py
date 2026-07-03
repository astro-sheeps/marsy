"""
Marsy simulator backend.

This backend adapts the external 4tronix rover simulator to the Marsy API.

External simulator:
    external/4tronix-rover-simulator/roversimui.py

Important:
    The original roversimulator.py can hang because it waits for HTTP responses.
    This backend sends fire-and-forget HTTP POST commands through a raw socket.
"""

import importlib
import json
import os
import socket
import sys
from pathlib import Path


# ---------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SIM_PATH = PROJECT_ROOT / "external" / "4tronix-rover-simulator"

SIM_HOST = "127.0.0.1"
SIM_PORT = 8523

if str(SIM_PATH) not in sys.path:
    sys.path.insert(0, str(SIM_PATH))


# Import external simulator module only as a namespace holder.
# We patch its functions below.
rover = importlib.import_module("roversimulator")


# ---------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------

SIM_VERBOSE = os.getenv("MARSY_SIM_VERBOSE", "0") == "1"
SIM_SOCKET_TIMEOUT = float(os.getenv("MARSY_SIM_SOCKET_TIMEOUT", "0.2"))

_num_pixels = 4
_leds = [[0, 0, 0] for _ in range(_num_pixels)]

_l_dir = 0
_r_dir = 0


# ---------------------------------------------------------------------
# Low-level simulator transport
# ---------------------------------------------------------------------

def _send_to_simulator(message, timeout=None):
    """
    Send one command to roversimui.py.

    This is intentionally fire-and-forget:
    - open TCP socket
    - send minimal HTTP POST
    - close socket
    - do not wait for HTTP response

    This avoids hangs caused by the external simulator not responding quickly.
    """
    if timeout is None:
        timeout = SIM_SOCKET_TIMEOUT

    try:
        body = json.dumps(message).encode("utf-8")

        request = (
            b"POST / HTTP/1.1\r\n"
            b"Host: 127.0.0.1:8523\r\n"
            b"Content-Type: application/json\r\n"
            + f"Content-Length: {len(body)}\r\n".encode("utf-8")
            + b"Connection: close\r\n"
            + b"\r\n"
            + body
        )

        with socket.create_connection((SIM_HOST, SIM_PORT), timeout=timeout) as sock:
            sock.sendall(request)

        return True

    except OSError as exc:
        if SIM_VERBOSE:
            print(f"[SIM WARNING] simulator socket error: {exc}", flush=True)
        return False


def _clamp_speed(speed):
    return int(max(0, min(100, speed)))


def _clamp_degrees(degrees):
    return int(max(-90, min(90, degrees)))


# ---------------------------------------------------------------------
# General functions
# ---------------------------------------------------------------------

def sim_init(brightness=0, PiBit=False):
    """
    Simulator init.

    Does not touch real GPIO. Keeps same signature as real rover.init().
    """
    print("Initialized", flush=True)


def sim_cleanup():
    """
    Simulator cleanup.

    Do not call the original roversimulator.cleanup(), because it may hang.
    """
    sim_stop()
    if SIM_VERBOSE:
        print("Simulator cleanup done", flush=True)


def sim_version():
    return 4


# ---------------------------------------------------------------------
# Motor functions
# ---------------------------------------------------------------------

def sim_stop():
    global _l_dir, _r_dir

    _l_dir = 0
    _r_dir = 0

    return _send_to_simulator({
        "wheelMotors": {
            "l": [0, 0],
            "r": [0, 0],
        }
    })


def sim_brake():
    global _l_dir, _r_dir

    _l_dir = 0
    _r_dir = 0

    return _send_to_simulator({
        "wheelMotors": {
            "l": [100, 100],
            "r": [100, 100],
        }
    })


def sim_forward(speed):
    global _l_dir, _r_dir

    speed = _clamp_speed(speed)
    _l_dir = 1
    _r_dir = 1

    return _send_to_simulator({
        "wheelMotors": {
            "l": [speed, 0],
            "r": [speed, 0],
        }
    })


def sim_reverse(speed):
    global _l_dir, _r_dir

    speed = _clamp_speed(speed)
    _l_dir = -1
    _r_dir = -1

    return _send_to_simulator({
        "wheelMotors": {
            "l": [0, speed],
            "r": [0, speed],
        }
    })


def sim_spin_left(speed):
    global _l_dir, _r_dir

    speed = _clamp_speed(speed)
    _l_dir = -1
    _r_dir = 1

    return _send_to_simulator({
        "wheelMotors": {
            "l": [0, speed],
            "r": [speed, 0],
        }
    })


def sim_spin_right(speed):
    global _l_dir, _r_dir

    speed = _clamp_speed(speed)
    _l_dir = 1
    _r_dir = -1

    return _send_to_simulator({
        "wheelMotors": {
            "l": [speed, 0],
            "r": [0, speed],
        }
    })


def sim_turn_forward(left_speed, right_speed):
    global _l_dir, _r_dir

    left_speed = _clamp_speed(left_speed)
    right_speed = _clamp_speed(right_speed)

    _l_dir = 1
    _r_dir = 1

    return _send_to_simulator({
        "wheelMotors": {
            "l": [left_speed, 0],
            "r": [right_speed, 0],
        }
    })


def sim_turn_reverse(left_speed, right_speed):
    global _l_dir, _r_dir

    left_speed = _clamp_speed(left_speed)
    right_speed = _clamp_speed(right_speed)

    _l_dir = -1
    _r_dir = -1

    return _send_to_simulator({
        "wheelMotors": {
            "l": [0, left_speed],
            "r": [0, right_speed],
        }
    })


# ---------------------------------------------------------------------
# Servo functions
# ---------------------------------------------------------------------

def sim_set_servo(servo, degrees):
    """
    Set one servo angle.

    Servo numbers follow the original 4tronix mapping:
        mast: 0
        front-left: 9
        rear-left: 11
        rear-right: 13
        front-right: 15
    """
    servo = int(servo)
    degrees = _clamp_degrees(degrees)

    return _send_to_simulator({
        "servos": {
            str(servo): degrees,
        }
    })


def sim_set_servos(servo_angles):
    """
    Set several servos in one command.

    Example:
        rover.setServos({
            9: 20,
            15: 20,
            11: -20,
            13: -20,
        })

    This is much faster than four separate setServo() calls.
    """
    payload = {
        "servos": {
            str(int(servo)): _clamp_degrees(degrees)
            for servo, degrees in servo_angles.items()
        }
    }

    return _send_to_simulator(payload)


def sim_stop_servos():
    """
    Simulator has no independent servo PWM stop state.
    Keep current angles.
    """
    return True


# ---------------------------------------------------------------------
# Fake sensors
# ---------------------------------------------------------------------

_distance_sequence_text = os.getenv("MARSY_SIM_DISTANCE_SEQUENCE", "").strip()
_distance_default_cm = float(os.getenv("MARSY_SIM_DISTANCE_CM", "100"))

_distance_sequence = []
_distance_index = 0

if _distance_sequence_text:
    try:
        _distance_sequence = [
            float(value.strip())
            for value in _distance_sequence_text.split(",")
            if value.strip()
        ]
    except ValueError:
        print(
            "Invalid MARSY_SIM_DISTANCE_SEQUENCE. Using default distance.",
            flush=True,
        )
        _distance_sequence = []


def sim_get_distance():
    """
    Fake ultrasonic distance.

    Modes:

    Constant:
        MARSY_SIM_DISTANCE_CM=20

    Sequence:
        MARSY_SIM_DISTANCE_SEQUENCE=100,80,60,40,20,100

    After sequence ends, last value is repeated.
    """
    global _distance_index

    if _distance_sequence:
        if _distance_index < len(_distance_sequence):
            value = _distance_sequence[_distance_index]
            _distance_index += 1
            return value

        return _distance_sequence[-1]

    return _distance_default_cm


def sim_get_battery():
    return 7.8


# ---------------------------------------------------------------------
# Keypad compatibility
# ---------------------------------------------------------------------

def sim_get_key():
    return 0


def sim_get_switch():
    return False


# ---------------------------------------------------------------------
# RGB LED compatibility
# ---------------------------------------------------------------------

def sim_from_rgb(red, green, blue):
    return (int(red) << 16) + (int(green) << 8) + int(blue)


def sim_to_rgb(color):
    return (
        (color & 0xFF0000) >> 16,
        (color & 0x00FF00) >> 8,
        color & 0x0000FF,
    )


def sim_set_pixel(pixel_id, color):
    pixel_id = int(pixel_id)

    if 0 <= pixel_id < _num_pixels:
        red, green, blue = sim_to_rgb(color)
        _leds[pixel_id] = [red, green, blue]


def sim_set_color(color):
    for i in range(_num_pixels):
        sim_set_pixel(i, color)


def sim_clear():
    for i in range(_num_pixels):
        _leds[i] = [0, 0, 0]


def sim_show():
    rgb_leds = {
        str(i): _leds[i]
        for i in range(_num_pixels)
    }

    return _send_to_simulator({
        "rgbLeds": rgb_leds,
    })


def sim_rainbow():
    # Minimal compatibility placeholder.
    # We can implement full rainbow later if needed.
    pass


def sim_wheel(pos):
    pos = int(pos) % 256

    if pos < 85:
        return sim_from_rgb(255 - pos * 3, pos * 3, 0)

    if pos < 170:
        pos -= 85
        return sim_from_rgb(0, 255 - pos * 3, pos * 3)

    pos -= 170
    return sim_from_rgb(pos * 3, 0, 255 - pos * 3)


# ---------------------------------------------------------------------
# Patch external rover module
# ---------------------------------------------------------------------

rover.init = sim_init
rover.cleanup = sim_cleanup
rover.version = sim_version

rover.stop = sim_stop
rover.brake = sim_brake
rover.forward = sim_forward
rover.reverse = sim_reverse
rover.spinLeft = sim_spin_left
rover.spinRight = sim_spin_right
rover.turnForward = sim_turn_forward
rover.turnReverse = sim_turn_reverse

rover.setServo = sim_set_servo
rover.setServos = sim_set_servos
rover.stopServos = sim_stop_servos

rover.getDistance = sim_get_distance
rover.getBattery = sim_get_battery
rover.getKey = sim_get_key
rover.getSwitch = sim_get_switch

rover.fromRGB = sim_from_rgb
rover.toRGB = sim_to_rgb
rover.setPixel = sim_set_pixel
rover.setColor = sim_set_color
rover.clear = sim_clear
rover.show = sim_show
rover.rainbow = sim_rainbow
rover.wheel = sim_wheel
rover.numPixels = _num_pixels