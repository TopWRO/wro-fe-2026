#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Iman_Dynamixel_Merged.py – DonkeyCar recording/driving + PS4/Dynamixel control

Flags (choose exactly one):
    --recording   Use PS4 controller, store images + user controls in a Tub
    --driving     Load a trained `.h5` model, predict angle/throttle, drive

This version keeps Iman11_N04.py's DonkeyCar camera / tub / model pipeline,
but replaces the BuildHAT LEGO motor control with the Dynamixel motor control
style from ControlRCcarwithPSControler.py.
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

DRIVE_LOOP_HZ      = 10
RECORD_THRESHOLD   = 0.05              # throttle magnitude to start recording

TUB_INPUTS = [
    "cam/image_array",
    "user/angle",
    "user/throttle",
    "user/mode",
]
TUB_TYPES  = ["image_array", "float", "float", "str"]

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

TARGET_RPM = 40
VELOCITY_UNIT_RPM = 0.229
MAX_STEER_DEG = 50
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
    """One DonkeyCar part that controls both Dynamixel throttle and steering."""

    def __init__(self):
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
        self.packet.write1ByteTxRx(self.port, ID_THROTTLE, ADDR_TORQUE, TORQUE_ON)

        # Steering motor: position mode
        self.packet.write1ByteTxRx(self.port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
        self.packet.write1ByteTxRx(self.port, ID_STEER, ADDR_MODE, POSITION_MODE)
        self.packet.write4ByteTxRx(self.port, ID_STEER, ADDR_PROFILE_ACCEL, 100)
        self.packet.write4ByteTxRx(self.port, ID_STEER, ADDR_PROFILE_VELOCITY, 500)
        self.packet.write1ByteTxRx(self.port, ID_STEER, ADDR_TORQUE, TORQUE_ON)

        # Same as the second program: force steering center to 2047.
        # If you want to use the live present position instead, replace this
        # with the read result from ADDR_PRESENT_POSITION.
        _present, _res, _err = self.packet.read4ByteTxRx(
            self.port, ID_STEER, ADDR_PRESENT_POSITION
        )
        self.zero_pos = 2010

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

        target_rpm = q_throttle * TARGET_RPM
        vel_val = int(target_rpm / VELOCITY_UNIT_RPM)

        steer_deg = q_steer * MAX_STEER_DEG
        pos_val = int(self.zero_pos + deg_to_unit(steer_deg))

        if vel_val != self.last_vel:
            self.packet.write4ByteTxRx(
                self.port, ID_THROTTLE, ADDR_GOAL_VELOCITY, to_uint32(vel_val)
            )
            self.last_vel = vel_val

        if  True:#pos_val != self.last_pos
            print("pos_val=",pos_val)
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


class ControlDisplay:
    def __init__(self):
        self.last_t = 0

    def run(self, angle: float, throttle: float):
        t = time.monotonic()
        if t - self.last_t >= 0.2:
            print(f"Steer: {angle*100:+.0f}% | Throttle: {throttle*100:+.0f}%", end='\r')
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
def build_vehicle_recording() -> dk.vehicle.Vehicle:
    car = dk.vehicle.Vehicle()
    add_camera(car)

    car.add(PS4Joystick(), outputs=["user/angle", "user/throttle", "user/mode"])

    car.add(Lambda(lambda t: abs(t) > RECORD_THRESHOLD),
            inputs=["user/throttle"], outputs=["recording"])

    tub = TubWriter(base_path=DATA_PATH, inputs=TUB_INPUTS, types=TUB_TYPES)
    car.add(tub, inputs=TUB_INPUTS, outputs=["tub/num_records"], run_condition="recording")

    car.add(DynamixelDrive(), inputs=["user/angle", "user/throttle"])
    car.add(ControlDisplay(), inputs=["user/angle", "user/throttle"], outputs=[])

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
    args = parser.parse_args()

    if args.recording:
        vehicle = build_vehicle_recording()
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
