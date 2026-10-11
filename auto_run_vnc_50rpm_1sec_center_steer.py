#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Auto run for Raspberry Pi / VNC, NOT Pico Thonny.

Based on the previous RC file:
- ID 1 = throttle motor, Velocity Mode
- ID 2 = steering motor, Position Mode
- Steering center position = 2047

Behavior:
1. Open /dev/ttyUSB0
2. Configure throttle motor in Velocity Mode
3. Configure steering motor in Position Mode
4. Reset steering to center every time before the car runs
5. Move forward at 50 RPM for 1 second
6. Stop throttle and reset steering to center again
7. Turn torque off and close port

Run in VNC Terminal:
    python3 auto_run_vnc_50rpm_1sec_center_steer.py
"""

import sys
import time
from dynamixel_sdk import *


# ============================================================
# Basic settings
# ============================================================

PORT = "/dev/ttyUSB0"
BAUD = 57600
PROTOCOL_VERSION = 2.0

ID_THROTTLE = 1
ID_STEER = 2


# ============================================================
# DYNAMIXEL XL330 / Protocol 2.0 control table
# ============================================================

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70

ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116

ADDR_PRESENT_POSITION = 132

VELOCITY_MODE = 1
POSITION_MODE = 3

TORQUE_OFF = 0
TORQUE_ON = 1

DXL_ALERT = 0x80


# ============================================================
# Car settings
# ============================================================

# Requested test values
AUTO_RPM = 50
AUTO_RUN_TIME_SECONDS = 1

# Positive RPM = forward, negative RPM = backward
# If the car moves backward when using +50, change this to -1.
THROTTLE_DIRECTION = 1

VELOCITY_UNIT_RPM = 0.229

# From the first RC file
STEER_CENTER_POS = 2047

# Keep speed safe
MAX_RPM = 200


# ============================================================
# DYNAMIXEL objects
# ============================================================

port = PortHandler(PORT)
packet = PacketHandler(PROTOCOL_VERSION)


# ============================================================
# Utility functions
# ============================================================

def to_uint32(value):
    """
    DYNAMIXEL SDK write4ByteTxRx needs negative velocity values converted
    to unsigned 32-bit.
    """
    value = int(value)
    if value < 0:
        value += 4294967296
    return value


def limit_rpm(rpm):
    if rpm > MAX_RPM:
        return MAX_RPM
    if rpm < -MAX_RPM:
        return -MAX_RPM
    return rpm


def rpm_to_raw_velocity(rpm):
    return int(rpm / VELOCITY_UNIT_RPM)


def check_comm(label, comm_result, dxl_error, allow_alert=False):
    if comm_result != COMM_SUCCESS:
        print("[COMM ERROR]", label, packet.getTxRxResult(comm_result))
        return False

    if dxl_error != 0:
        if allow_alert and (dxl_error & DXL_ALERT):
            return True

        print("[DXL ERROR]", label, packet.getRxPacketError(dxl_error))
        return False

    return True


def write1(motor_id, addr, value, label, allow_alert=False):
    comm, err = packet.write1ByteTxRx(port, motor_id, addr, value)
    if not check_comm(label, comm, err, allow_alert=allow_alert):
        raise RuntimeError(label)


def write4(motor_id, addr, value, label, allow_alert=False):
    comm, err = packet.write4ByteTxRx(port, motor_id, addr, value)
    if not check_comm(label, comm, err, allow_alert=allow_alert):
        raise RuntimeError(label)


def read1(motor_id, addr, label, allow_alert=False):
    value, comm, err = packet.read1ByteTxRx(port, motor_id, addr)
    if not check_comm(label, comm, err, allow_alert=allow_alert):
        return None
    return value


def decode_hw_error(hw):
    errors = []

    if hw & 0x01:
        errors.append("Input Voltage Error")
    if hw & 0x04:
        errors.append("Overheating Error")
    if hw & 0x08:
        errors.append("Motor Encoder Error")
    if hw & 0x10:
        errors.append("Electrical Shock / Insufficient Power")
    if hw & 0x20:
        errors.append("Overload Error")

    if not errors:
        errors.append("No hardware error")

    return errors


def check_hardware_error(motor_id, name):
    hw = read1(
        motor_id,
        ADDR_HARDWARE_ERROR,
        name + " Hardware Error Status(70)",
        allow_alert=True
    )

    if hw is None:
        raise RuntimeError(name + " hardware error read failed")

    if hw != 0:
        print("[FATAL]", name, "hardware error =", hw, "hex =", hex(hw))
        print("[FATAL]", name, "decode =", decode_hw_error(hw))
        raise RuntimeError(name + " hardware error")

    return hw


# ============================================================
# Motor configuration
# ============================================================

def configure_throttle_motor():
    """
    Throttle motor should be Velocity Mode, same idea as the first RC file.
    """
    print("[CONFIG] THROTTLE motor: velocity mode")

    write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_OFF, "throttle torque off", allow_alert=True)
    time.sleep(0.05)

    write1(ID_THROTTLE, ADDR_OPERATING_MODE, VELOCITY_MODE, "throttle velocity mode", allow_alert=True)
    write4(ID_THROTTLE, ADDR_PROFILE_ACCEL, 200, "throttle profile accel", allow_alert=True)
    write4(ID_THROTTLE, ADDR_GOAL_VELOCITY, 0, "throttle velocity zero", allow_alert=True)

    write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_ON, "throttle torque on", allow_alert=True)
    time.sleep(0.05)

    check_hardware_error(ID_THROTTLE, "THROTTLE")
    print("[CONFIG] THROTTLE ready")


def configure_steer_motor():
    """
    Steering motor should be Position Mode, not Velocity Mode.
    This is the correction from the first RC file.
    """
    print("[CONFIG] STEER motor: position mode")

    write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_OFF, "steer torque off", allow_alert=True)
    time.sleep(0.05)

    write1(ID_STEER, ADDR_OPERATING_MODE, POSITION_MODE, "steer position mode", allow_alert=True)
    write4(ID_STEER, ADDR_PROFILE_ACCEL, 100, "steer profile accel", allow_alert=True)
    write4(ID_STEER, ADDR_PROFILE_VELOCITY, 500, "steer profile velocity", allow_alert=True)

    write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_ON, "steer torque on", allow_alert=True)
    time.sleep(0.05)

    reset_steering()
    check_hardware_error(ID_STEER, "STEER")
    print("[CONFIG] STEER ready")


# ============================================================
# Movement functions
# ============================================================

def reset_steering():
    """
    Reset steering to center.
    This is called:
    - during setup
    - before every auto run
    - after every auto run
    - during safe stop
    """
    print("[STEER] Reset to center:", STEER_CENTER_POS)
    write4(ID_STEER, ADDR_GOAL_POSITION, STEER_CENTER_POS, "steer center position", allow_alert=True)
    time.sleep(0.30)


def set_throttle_rpm(rpm):
    rpm = limit_rpm(rpm)
    rpm = rpm * THROTTLE_DIRECTION

    raw_velocity = rpm_to_raw_velocity(rpm)
    print("[THROTTLE] RPM:", rpm, "Raw:", raw_velocity)

    write4(
        ID_THROTTLE,
        ADDR_GOAL_VELOCITY,
        to_uint32(raw_velocity),
        "write throttle goal velocity",
        allow_alert=True
    )


def stop_throttle():
    write4(ID_THROTTLE, ADDR_GOAL_VELOCITY, 0, "stop throttle", allow_alert=True)


def safe_stop():
    print("")
    print("[SAFE STOP] Stop throttle and center steering")

    try:
        stop_throttle()
    except Exception as e:
        print("[SAFE STOP] throttle stop failed:", e)

    try:
        reset_steering()
    except Exception as e:
        print("[SAFE STOP] steering reset failed:", e)

    time.sleep(0.2)

    try:
        write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_OFF, "throttle torque off", allow_alert=True)
    except Exception as e:
        print("[SAFE STOP] throttle torque off failed:", e)

    try:
        write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_OFF, "steer torque off", allow_alert=True)
    except Exception as e:
        print("[SAFE STOP] steer torque off failed:", e)

    print("[SAFE STOP] Done")


def autonomous_run(speed_rpm=AUTO_RPM, run_time_seconds=AUTO_RUN_TIME_SECONDS):
    """
    Positive speed_rpm = forward.
    Negative speed_rpm = backward.

    Requested default:
    - speed_rpm = 50
    - run_time_seconds = 1
    """
    speed_rpm = limit_rpm(speed_rpm)

    print("")
    print("================================")
    print("[AUTO RUN] Started")
    print("[AUTO RUN] Speed:", speed_rpm, "RPM")
    print("[AUTO RUN] Time:", run_time_seconds, "second(s)")
    print("[AUTO RUN] Positive = forward, negative = backward")
    print("================================")

    # Important correction:
    # Steering is reset every time before moving.
    reset_steering()

    check_hardware_error(ID_THROTTLE, "THROTTLE")
    check_hardware_error(ID_STEER, "STEER")

    set_throttle_rpm(speed_rpm)

    start_time = time.time()

    while time.time() - start_time < run_time_seconds:
        # keep checking steering/motor health while running
        check_hardware_error(ID_THROTTLE, "THROTTLE")
        check_hardware_error(ID_STEER, "STEER")
        time.sleep(0.05)

    stop_throttle()

    # Important correction:
    # Steering is reset again after the run.
    reset_steering()

    print("[AUTO RUN] Finished")


# ============================================================
# Main
# ============================================================

def main():
    print("Opening port:", PORT)

    if not port.openPort():
        print("[FATAL] Failed to open port:", PORT)
        sys.exit(1)

    if not port.setBaudRate(BAUD):
        print("[FATAL] Failed to set baudrate:", BAUD)
        port.closePort()
        sys.exit(1)

    print("[OK] Port opened and baudrate set")

    try:
        configure_throttle_motor()
        configure_steer_motor()

        # Requested auto run:
        # speed = 50 RPM
        # time = 1 second
        autonomous_run(50, 1)

    except KeyboardInterrupt:
        print("")
        print("[KEYBOARD] Ctrl-C detected")

    except RuntimeError as e:
        print("")
        print("[RUNTIME EXIT]", e)

    finally:
        safe_stop()
        port.closePort()
        print("[END] Program exited")


if __name__ == "__main__":
    main()
