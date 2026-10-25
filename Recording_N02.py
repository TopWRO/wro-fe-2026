#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Iman_Dynamixel_Merged.py – DonkeyCar recording/driving + PS4/Dynamixel control

Flags (choose exactly one):
    --recording   Use PS4 controller, store images + user controls in a Tub
    --driving     Load a trained `.h5` model, predict angle/throttle, drive

This version keeps Iman11_N04.py's DonkeyCar camera / tub / model pipeline,
but replaces the BuildHAT LEGO motor control with the Dynamixel motor control
style from ControlRCcarwithPSControler.py.

N02 adds color-area detection while recording:
    --color {red,yellow,blue,green,black,white}   (default: red)
Every frame, the pixels matching the chosen color are counted on the
160x120 image (the same image saved to the Tub). The area (pixels) and the
percentage of the frame are printed and stored in the Tub as
"color/area" and "color/area_pct".

Example:
    python3 Recording_N02.py --recording --color green
"""

# ---------------------------------------------------------------------------
# Standard library
# ---------------------------------------------------------------------------
import os
import sys
import time
import signal
from pathlib import Path
from typing import Tuple

# Third-party
import cv2
import numpy as np
import donkeycar as dk
import pygame
from dynamixel_sdk import *
from donkeycar.parts.transform import Lambda
from donkeycar.parts.tub_v2 import TubWriter

try:
    from donkeycar.parts.keras import KerasInterpreter, KerasLinear
except ImportError:
    sys.exit("Keras/TensorFlow not available – install DonkeyCar with AI support.")

# ---------------------------------------------------------------------------
# DonkeyCar / camera configuration
# ---------------------------------------------------------------------------
CAPTURE_W, CAPTURE_H = (320, 240)      # camera capture
CROP_W, CROP_H       = (160, 120)      # image sent to network / tub

DATA_PATH          = os.path.expanduser("~/Top WRO FE 2026/data")
MODEL_PATH_DEFAULT = os.path.expanduser("~/Top WRO FE 2026/ccwmodel/mypilot.h5")

DRIVE_LOOP_HZ      = 20
RECORD_THRESHOLD   = 0.05              # raw joystick throttle threshold for recording

# Recording-speed limiter:
# Raw PS4 throttle (-1..+1) is STILL stored in the Tub unchanged.
# Only the throttle sent to the Dynamixel drive motor is scaled by this value.
# Example: +1.0 joystick * 0.40 => +0.40 motor command => about +40 RPM
# because TARGET_RPM is 100 RPM.
RECORD_MAX_SPEED    = 0.40              # 40% of the 100 RPM software full-scale

TUB_INPUTS = [
    "cam/image_array",
    "user/angle",
    "user/throttle",
    "user/mode",
    "color/area",
    "color/area_pct",
]
TUB_TYPES  = ["image_array", "float", "float", "str", "float", "float"]

# ---------------------------------------------------------------------------
# Color-area detection
# ---------------------------------------------------------------------------
DEFAULT_COLOR = "red"

# HSV ranges (OpenCV: H 0-179, S 0-255, V 0-255).
# AWB is off, so colors depend on PICAMERA_AWB_GAINS and the venue lighting –
# check on the real field and fine-tune if needed.
# Red wraps around H=0/179, so it needs two ranges.
COLOR_RANGES = {
    "red":    [((0,   100,  70), (10,  255, 255)),
               ((170, 100,  70), (179, 255, 255))],
    "yellow": [((20,   80,  80), (35,  255, 255))],
    "blue":   [((100,  80,  50), (130, 255, 255))],
    "green":  [((40,   60,  40), (85,  255, 255))],
    "black":  [((0,     0,   0), (179, 255,  50))],   # dark = low V
    "white":  [((0,     0, 180), (179,  40, 255))],   # bright + low saturation
}

COLOR_KERNEL = np.ones((3, 3), np.uint8)

# ---------------------------------------------------------------------------
# PiCamera manual settings
# ---------------------------------------------------------------------------
CAMERA_FRAMERATE               = 30
PICAMERA_AWB_MODE              = 'off'
PICAMERA_EXPOSURE_MODE         = 'off'
PICAMERA_ISO                   = 100
PICAMERA_SHUTTER_SPEED         = 15000
PICAMERA_AWB_GAINS             = (1.2, 1.5)
PICAMERA_EXPOSURE_COMPENSATION = 0

# ---------------------------------------------------------------------------
# PS4 / Dynamixel configuration from ControlRCcarwithPSControler.py
# ---------------------------------------------------------------------------
PORT = "/dev/ttyUSB0"
BAUD = 57600
ID_THROTTLE = 1
ID_STEER = 2

ADDR_MODE = 11
ADDR_TORQUE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112

VELOCITY_MODE = 1
POSITION_MODE = 3
TORQUE_ON = 1
TORQUE_OFF = 0

TARGET_RPM = 100
VELOCITY_UNIT_RPM = 0.229
MAX_STEER_DEG = 40
DEADZONE = 0.18
STEP = 0.10


def to_uint32(value):
    value = int(value)
    if value < 0:
        value += 4294967296
    return value


def deg_to_unit(deg):
    return int(deg * 4096 / 360)


def quantize(val):
    """Copy the second program's joystick deadzone + 10% step control."""
    if abs(val) < DEADZONE:
        return 0.0
    sign = 1 if val >= 0 else -1
    q = round(abs(val) / STEP) * STEP
    return sign * max(0.0, min(1.0, q))


def center_crop(img, tw=CROP_W, th=CROP_H):
    # The original Iman program actually resized instead of cropping.
    return cv2.resize(img, (tw, th))

# ---------------------------------------------------------------------------
# Hardware parts
# ---------------------------------------------------------------------------
class DynamixelDrive:
    """One DonkeyCar part that controls both Dynamixel throttle and steering.

    max_speed scales ONLY the physical throttle command. It does not change
    the user/throttle value stored in the training Tub.
    """

    def __init__(self, max_speed: float = 1.0):
        self.max_speed = max(0.0, min(1.0, float(max_speed)))
        self.port = PortHandler(PORT)
        self.packet = PacketHandler(2.0)

        if not self.port.openPort():
            sys.exit(f"Could not open Dynamixel port: {PORT}")
        if not self.port.setBaudRate(BAUD):
            sys.exit(f"Could not set Dynamixel baud rate: {BAUD}")

        # Throttle motor: velocity mode
        self.packet.write1ByteTxRx(self.port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
        self.packet.write1ByteTxRx(self.port, ID_THROTTLE, ADDR_MODE, VELOCITY_MODE)
        self.packet.write4ByteTxRx(self.port, ID_THROTTLE, ADDR_PROFILE_ACCEL, 100)
        self.packet.write4ByteTxRx(self.port, ID_THROTTLE, ADDR_GOAL_VELOCITY, 0)
        self.packet.write1ByteTxRx(self.port, ID_THROTTLE, ADDR_TORQUE, TORQUE_ON)

        # Steering motor: position mode
        self.packet.write1ByteTxRx(self.port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
        self.packet.write1ByteTxRx(self.port, ID_STEER, ADDR_MODE, POSITION_MODE)
        self.packet.write4ByteTxRx(self.port, ID_STEER, ADDR_PROFILE_ACCEL, 100)
        self.packet.write4ByteTxRx(self.port, ID_STEER, ADDR_PROFILE_VELOCITY, 500)
        self.packet.write1ByteTxRx(self.port, ID_STEER, ADDR_TORQUE, TORQUE_ON)

        # Steering center matched to the current competition-car calibration.
        # Mechanical left/right limits measured around this center are nearly symmetric.
        _present, _res, _err = self.packet.read4ByteTxRx(
            self.port, ID_STEER, ADDR_PRESENT_POSITION
        )
        self.zero_pos = 2016

        self.last_vel = None
        self.last_pos = None
        print("Dynamixel drive ready")

    def run(self, angle: float, throttle: float):
        # Clamp AI/user values to DonkeyCar's normal -1 to +1 range.
        angle = max(min(float(angle or 0.0), 1.0), -1.0)
        throttle = max(min(float(throttle or 0.0), 1.0), -1.0)

        # Use second program's control conversion.
        q_steer = quantize(angle)
        q_throttle = quantize(throttle)

        # Limit only the real drive speed. Example: q_throttle=+1.0 and
        # max_speed=0.40 -> +40 RPM when TARGET_RPM=100.
        drive_throttle = q_throttle * self.max_speed
        target_rpm = drive_throttle * TARGET_RPM
        vel_val = int(target_rpm / VELOCITY_UNIT_RPM)

        steer_deg = q_steer * MAX_STEER_DEG
        pos_val = int(self.zero_pos + deg_to_unit(steer_deg))

        if vel_val != self.last_vel:
            self.packet.write4ByteTxRx(
                self.port, ID_THROTTLE, ADDR_GOAL_VELOCITY, to_uint32(vel_val)
            )
            self.last_vel = vel_val

        if pos_val != self.last_pos:
            self.packet.write4ByteTxRx(
                self.port, ID_STEER, ADDR_GOAL_POSITION, pos_val
            )
            self.last_pos = pos_val

    def shutdown(self):
        print("Stopping Dynamixel motors...")
        try:
            self.packet.write4ByteTxRx(self.port, ID_THROTTLE, ADDR_GOAL_VELOCITY, 0)
            self.packet.write4ByteTxRx(self.port, ID_STEER, ADDR_GOAL_POSITION, self.zero_pos)
            time.sleep(0.5)
            self.packet.write1ByteTxRx(self.port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
            self.packet.write1ByteTxRx(self.port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
        finally:
            self.port.closePort()


class PS4Joystick:
    """PS4 joystick copied to match the second program's axis logic."""

    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No PS4 controller detected.")
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        print(f"Connected joystick: {self.js.get_name()}")

    def run(self) -> Tuple[float, float, str]:
        pygame.event.pump()

        # Same axes as ControlRCcarwithPSControler.py
        raw_steer = self.js.get_axis(0)
        raw_throttle = -self.js.get_axis(4)

        angle = quantize(raw_steer)
        throttle = quantize(raw_throttle)

        # Triangle button: delete recent bad training records.
        if self.js.get_button(2):
            return angle, throttle, "erase"

        # L1 button: exit program.
        if self.js.get_button(4):
            raise KeyboardInterrupt

        return angle, throttle, "user"

    def shutdown(self):
        pygame.quit()


class ConsoleDisplay:
    def __init__(self):
        self.last_t = 0

    def run(self, angle: float, throttle: float):
        t = time.monotonic()
        if t - self.last_t >= 1.0:
            print(f"Pred → angle {angle:+.2f}  thr {throttle:+.2f}")
            self.last_t = t


class ColorAreaDetector:
    """Count the pixels of one color in the camera image.

    Input is the DonkeyCar camera image (RGB order).
    Outputs: area in pixels, and area as % of the whole frame.
    """

    def __init__(self, color: str = DEFAULT_COLOR):
        if color not in COLOR_RANGES:
            raise ValueError(f"Unknown color: {color}")
        self.color = color
        self.ranges = [(np.array(lo, np.uint8), np.array(hi, np.uint8))
                       for lo, hi in COLOR_RANGES[color]]

    def run(self, img):
        if img is None:
            return 0.0, 0.0
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        mask = None
        for lo, hi in self.ranges:
            m = cv2.inRange(hsv, lo, hi)
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        # Remove single-pixel noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, COLOR_KERNEL)
        area = float(cv2.countNonZero(mask))
        pct = 100.0 * area / (mask.shape[0] * mask.shape[1])
        return area, pct


class ControlDisplay:
    def __init__(self, color: str = DEFAULT_COLOR):
        self.last_t = 0
        self.color = color

    def run(self, angle: float, throttle: float, area: float = 0.0, pct: float = 0.0):
        t = time.monotonic()
        if t - self.last_t >= 0.2:
            print(f"Steer: {angle*100:+.0f}% | Throttle: {throttle*100:+.0f}% | "
                  f"{self.color}: {area:5.0f} px ({pct:4.1f}%)   ", end='\r')
            self.last_t = t

# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
def add_camera(car: dk.vehicle.Vehicle):
    from donkeycar.parts.camera import PiCamera

    cam = PiCamera(image_w=CAPTURE_W, image_h=CAPTURE_H, image_d=3)
    time.sleep(2)

    cam.camera.framerate = CAMERA_FRAMERATE
    cam.camera.exposure_mode = PICAMERA_EXPOSURE_MODE
    cam.camera.awb_mode = PICAMERA_AWB_MODE
    cam.camera.iso = PICAMERA_ISO
    cam.camera.shutter_speed = PICAMERA_SHUTTER_SPEED
    cam.camera.exposure_compensation = PICAMERA_EXPOSURE_COMPENSATION
    cam.camera.awb_gains = PICAMERA_AWB_GAINS

    car.add(cam, outputs=["cam/raw"], threaded=True)
    car.add(Lambda(center_crop), inputs=["cam/raw"], outputs=["cam/image_array"])

# ---------------------------------------------------------------------------
# Vehicle builders
# ---------------------------------------------------------------------------
def build_vehicle_recording(color: str = DEFAULT_COLOR) -> dk.vehicle.Vehicle:
    car = dk.vehicle.Vehicle()
    add_camera(car)

    car.add(PS4Joystick(), outputs=["user/angle", "user/throttle", "user/mode"])

    # Color area on the same 160x120 image that goes into the Tub.
    car.add(ColorAreaDetector(color), inputs=["cam/image_array"],
            outputs=["color/area", "color/area_pct"])
    print(f"Color detection: {color}")

    car.add(Lambda(lambda t: abs(t) > RECORD_THRESHOLD),
            inputs=["user/throttle"], outputs=["recording"])

    # Keep the ORIGINAL joystick throttle in the Tub for training consistency.
    tub = TubWriter(base_path=DATA_PATH, inputs=TUB_INPUTS, types=TUB_TYPES)
    car.add(tub, inputs=TUB_INPUTS, outputs=["tub/num_records"], run_condition="recording")

    # Record user/throttle unchanged, but scale ONLY the physical drive motor.
    # user/throttle=+1.0 is still saved as +1.0, while the motor receives
    # +1.0 * RECORD_MAX_SPEED (e.g. 0.40 -> about +40 RPM).
    car.add(DynamixelDrive(max_speed=RECORD_MAX_SPEED),
            inputs=["user/angle", "user/throttle"])
    car.add(ControlDisplay(color),
            inputs=["user/angle", "user/throttle", "color/area", "color/area_pct"],
            outputs=[])

    class RecordCounter:
        def __init__(self):
            self.last_ten = -1
        def run(self, n):
            if n is None:
                return
            ten = n // 10
            if ten != self.last_ten:
                print(f"\nRecorded samples: {n}")
                self.last_ten = ten

    car.add(RecordCounter(), inputs=["tub/num_records"], outputs=[])

    class PromptWiper:
        def __init__(self, tub_obj, num_records=100):
            self.tub = tub_obj
            self.num = num_records
            self.erased = False

        def run(self, mode):
            if mode == "erase" and not self.erased:
                self.tub.delete_last_n_records(self.num)
                print(f"\nDeleted last {self.num} records.")
                self.erased = True
            if mode != "erase":
                self.erased = False

    car.add(PromptWiper(tub.tub, num_records=100), inputs=["user/mode"], outputs=[])
    return car


def build_vehicle_driving(model_path: str) -> dk.vehicle.Vehicle:
    car = dk.vehicle.Vehicle()
    add_camera(car)

    interpreter = KerasInterpreter()
    pilot = KerasLinear(interpreter=interpreter, input_shape=(CROP_H, CROP_W, 3))
    pilot.load(model_path)

    car.add(pilot, inputs=["cam/image_array"], outputs=["pilot/angle", "pilot/throttle"])
    car.add(ConsoleDisplay(), inputs=["pilot/angle", "pilot/throttle"], outputs=[])
    car.add(DynamixelDrive(), inputs=["pilot/angle", "pilot/throttle"])

    return car

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Record data or drive autonomously with Dynamixel motors")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--recording", action="store_true", help="Manual PS4 driving & data logging")
    group.add_argument("--driving", action="store_true", help="Autonomous driving with model")
    parser.add_argument("--model", default=MODEL_PATH_DEFAULT, help=".h5 model path for --driving mode")
    parser.add_argument("--color", default=DEFAULT_COLOR, choices=list(COLOR_RANGES.keys()),
                        help="Color to measure while recording (default: red)")
    args = parser.parse_args()

    if args.recording:
        vehicle = build_vehicle_recording(args.color)
    else:
        mpath = Path(args.model).expanduser()
        if not mpath.is_file():
            sys.exit(f"Model file not found: {mpath}")
        vehicle = build_vehicle_driving(str(mpath))

    def _sigterm(_s, _f):
        print("\nSIGTERM – shutting down…")
        (vehicle.shutdown() if hasattr(vehicle, "shutdown") else vehicle.stop())
        sys.exit(0)

    signal.signal(signal.SIGTERM, _sigterm)

    try:
        vehicle.start(rate_hz=DRIVE_LOOP_HZ)
    except KeyboardInterrupt:
        print("\nCtrl-C / L1 – shutting down…")
    finally:
        (vehicle.shutdown() if hasattr(vehicle, "shutdown") else vehicle.stop())
