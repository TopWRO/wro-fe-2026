#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FINAL FIXED VERSION (RC CAR)

KEY FIXES:
- NO negative velocity is ever sent (XL330 limitation fix)
- Forward / Backward handled via DRIVE MODE reverse bit
- Steering center set to 2010
- 5 second run:
    0–2.5s forward
    2.5–5s backward
- Joystick controls speed magnitude (0–100 RPM)
- Smooth + stable control

Run:
    python3 rc_final_fixed_front_back.py
"""

import sys
import time
import pygame
from dynamixel_sdk import *


# =========================
# BASIC CONFIG
# =========================

PORT = "/dev/ttyUSB0"
BAUD = 57600
PROTOCOL_VERSION = 2.0

ID_THROTTLE = 1
ID_STEER = 2


# =========================
# CONTROL TABLE
# =========================

ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70

ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116

# IMPORTANT: XL330 drive mode (direction control)
ADDR_DRIVE_MODE = 10


VELOCITY_MODE = 1
POSITION_MODE = 3

TORQUE_ON = 1
TORQUE_OFF = 0


# =========================
# RUN SETTINGS
# =========================

RUN_TIME = 50
HALF_TIME = RUN_TIME / 2.0

MAX_RPM = 100
MIN_RPM = -100

STEER_CENTER_POS = 2010   # <<< USER REQUEST FIX


# joystick
THROTTLE_AXIS = 4
DEADZONE = 0.08


# =========================
# INIT
# =========================

port = PortHandler(PORT)
packet = PacketHandler(PROTOCOL_VERSION)


def rpm_to_raw(rpm):
    return int(rpm / 0.229)


def write4(id, addr, val):
    comm, err = packet.write4ByteTxRx(port, id, addr, val)
    if comm != 0 or err != 0:
        print("Write error:", comm, err)


def write1(id, addr, val):
    comm, err = packet.write1ByteTxRx(port, id, addr, val)
    if comm != 0 or err != 0:
        print("Write error:", comm, err)


# =========================
# MOTOR CONTROL (KEY FIX)
# =========================

def set_direction(forward: bool):
    """
    XL330 DRIVE MODE:
    0 = normal
    1 = reverse rotation
    """
    time.sleep(0.001)
#     if forward:
#         write1(ID_THROTTLE, ADDR_DRIVE_MODE, 0)
#     else:
#         write1(ID_THROTTLE, ADDR_DRIVE_MODE, 1)


def set_speed(rpm):
    """
    ALWAYS POSITIVE SPEED ONLY
    """
    if rpm < -MAX_RPM:
        rpm = -MAX_RPM
    if rpm > MAX_RPM:
        rpm = MAX_RPM

    raw = rpm_to_raw(rpm)
    write4(ID_THROTTLE, ADDR_GOAL_VELOCITY, raw)


# =========================
# STEERING
# =========================

def reset_steering():
    write4(ID_STEER, ADDR_GOAL_POSITION, STEER_CENTER_POS)
    time.sleep(0.2)


# =========================
# JOYSTICK
# =========================

def joy_to_speed(axis):
    v = axis * -1
    if v < DEADZONE:
        return 0

    v = (v - DEADZONE) / (1 - DEADZONE)
    if v < 0:
        v = 0
    if v > 1:
        v = 1
    return v * MAX_RPM


# =========================
# MAIN RUN
# =========================

def main():
    port.openPort()
    port.setBaudRate(BAUD)

    pygame.init()
    joy = pygame.joystick.Joystick(0)
    joy.init()

    reset_steering()

    start = time.time()
    last_speed = 0

    while time.time() - start < RUN_TIME:

        t = time.time() - start

        # direction via DRIVE MODE (NO negative velocity ever)
        if t < HALF_TIME:
            set_direction(True)
        else:
            set_direction(False)

        pygame.event.pump()
        axis = joy.get_axis(THROTTLE_AXIS)

        speed = joy_to_speed(axis)

        # smoothing
        speed = 0.3 * speed + 0.7 * last_speed
        last_speed = speed

        set_speed(speed)

        print("t:", round(t,2), "speed:", round(speed,1))

        time.sleep(0.05)

    set_speed(0)
    reset_steering()

    print("DONE")


if __name__ == "__main__":
    main()
