#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
VNC / Raspberry Pi normal Python code.

Goal:
- NOT Pico / NOT Thonny
- Robot still runs by time
- Run time = 1 second
- Speed is controlled by joystick while it is running
- Speed range = 0 to 100 RPM
- Steering motor resets to calibrated straight position every run
- Positive RPM should move forward

Based on your previous RC structure:
- /dev/ttyUSB0 = U2D2
- ID 1 = throttle motor, Velocity Mode
- ID 2 = steering motor, Position Mode
- PS controller through pygame joystick

Run:
    python3 auto_run_vnc_joystick_speed_0_100rpm_1sec.py
'''

import sys
import time
import pygame
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

RUN_TIME_SECONDS = 1.0

# Joystick speed range
MIN_RPM = 0
MAX_RPM = 100

# Your car previously went backward, so throttle direction is reversed.
# Keep this as -1 so positive calculated RPM moves forward.
THROTTLE_DIRECTION = -1

VELOCITY_UNIT_RPM = 0.229

# Same joystick axis as your RC file used for throttle.
# In your original RC code:
#     raw_throttle = joy.get_axis(4)
THROTTLE_AXIS = 4

# Many PS controllers give negative value when pushing stick upward.
# This setting makes pushing forward/up become positive speed.
# If your joystick works opposite, change this to 1.
JOYSTICK_FORWARD_SIGN = -1

DEADZONE = 0.08

# ============================================================
# Steering calibration
# ============================================================
# Your measured note:
#   -300 to +300 = steering 0 / straight
#   700 to 1300 = steering -90
#
# So reset steering to 0 for straight.
STEER_CENTER_POS = 0

STEER_ZERO_MIN = -300
STEER_ZERO_MAX = 300

STEER_NEG90_MIN = 700
STEER_NEG90_MAX = 1300


# ============================================================
# DYNAMIXEL objects
# ============================================================

port = PortHandler(PORT)
packet = PacketHandler(PROTOCOL_VERSION)


# ============================================================
# Utility functions
# ============================================================

def to_uint32(value):
    value = int(value)
    if value < 0:
        value += 4294967296
    return value


def s32(value):
    value = int(value)
    if value & 0x80000000:
        return value - 4294967296
    return value


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


def read4(motor_id, addr, label, allow_alert=False):
    value, comm, err = packet.read4ByteTxRx(port, motor_id, addr)
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


def rpm_to_raw_velocity(rpm):
    return int(rpm / VELOCITY_UNIT_RPM)


# ============================================================
# Steering helpers
# ============================================================

def read_steer_position():
    pos_raw = read4(
        ID_STEER,
        ADDR_PRESENT_POSITION,
        "steer present position",
        allow_alert=True
    )

    if pos_raw is None:
        return None

    return s32(pos_raw)


def print_steer_status():
    pos = read_steer_position()

    if pos is None:
        print("[STEER] Present position read failed")
        return

    if STEER_ZERO_MIN <= pos <= STEER_ZERO_MAX:
        zone = "0 degree / straight zone"
    elif STEER_NEG90_MIN <= pos <= STEER_NEG90_MAX:
        zone = "-90 degree zone"
    else:
        zone = "outside calibrated zones"

    print("[STEER] Present position:", pos, "|", zone)


def reset_steering():
    print("[STEER] Reset to calibrated center:", STEER_CENTER_POS)

    write4(
        ID_STEER,
        ADDR_GOAL_POSITION,
        to_uint32(STEER_CENTER_POS),
        "steer center",
        allow_alert=True
    )

    time.sleep(0.30)
    print_steer_status()


# ============================================================
# Motor configuration
# ============================================================

def configure_throttle_motor():
    print("[CONFIG] THROTTLE motor: velocity mode")

    write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_OFF, "throttle torque off", allow_alert=True)
    time.sleep(0.05)

    write1(ID_THROTTLE, ADDR_OPERATING_MODE, VELOCITY_MODE, "throttle velocity mode", allow_alert=True)
    write4(ID_THROTTLE, ADDR_PROFILE_ACCEL, 200, "throttle profile accel", allow_alert=True)
    write4(ID_THROTTLE, ADDR_GOAL_VELOCITY, 0, "throttle zero", allow_alert=True)

    write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_ON, "throttle torque on", allow_alert=True)
    time.sleep(0.05)

    check_hardware_error(ID_THROTTLE, "THROTTLE")
    print("[CONFIG] THROTTLE ready")


def configure_steer_motor():
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
# Joystick functions
# ============================================================

def setup_joystick():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("[FATAL] No joystick found.")
        return None

    joy = pygame.joystick.Joystick(0)
    joy.init()

    print("[OK] Joystick ready:", joy.get_name())
    return joy


def joystick_axis_to_speed_rpm(raw_axis):
    forward_value = raw_axis * JOYSTICK_FORWARD_SIGN

    if forward_value < DEADZONE:
        return 0

    speed_ratio = (forward_value - DEADZONE) / (1.0 - DEADZONE)

    if speed_ratio < 0:
        speed_ratio = 0
    if speed_ratio > 1:
        speed_ratio = 1

    return speed_ratio * MAX_RPM


# ============================================================
# Movement functions
# ============================================================

def set_throttle_rpm(rpm):
    if rpm < MIN_RPM:
        rpm = MIN_RPM
    if rpm > MAX_RPM:
        rpm = MAX_RPM

    command_rpm = rpm * THROTTLE_DIRECTION
    raw_velocity = rpm_to_raw_velocity(command_rpm)

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

    pygame.quit()
    print("[SAFE STOP] Done")


def autonomous_time_run_with_joystick_speed(joy, run_time_seconds=RUN_TIME_SECONDS):
    print("")
    print("================================")
    print("[AUTO RUN] Started")
    print("[AUTO RUN] Time:", run_time_seconds, "second(s)")
    print("[AUTO RUN] Speed controlled by joystick:", MIN_RPM, "to", MAX_RPM, "RPM")
    print("[AUTO RUN] Push throttle joystick forward to increase speed")
    print("================================")

    reset_steering()
    check_hardware_error(ID_THROTTLE, "THROTTLE")
    check_hardware_error(ID_STEER, "STEER")

    start_time = time.time()
    last_print_time = 0.0

    while time.time() - start_time < run_time_seconds:
        pygame.event.pump()

        raw_throttle = joy.get_axis(THROTTLE_AXIS)
        target_rpm = joystick_axis_to_speed_rpm(raw_throttle)

        set_throttle_rpm(target_rpm)

        now = time.time()
        if now - last_print_time >= 0.10:
            last_print_time = now
            elapsed = now - start_time
            print(
                "[RUN] elapsed={:.2f}s | raw_axis={:+.2f} | target_speed={:.1f} rpm".format(
                    elapsed,
                    raw_throttle,
                    target_rpm
                )
            )

        check_hardware_error(ID_THROTTLE, "THROTTLE")
        check_hardware_error(ID_STEER, "STEER")

        time.sleep(0.05)

    stop_throttle()
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

        joy = setup_joystick()
        if joy is None:
            raise RuntimeError("no joystick")

        autonomous_time_run_with_joystick_speed(joy, RUN_TIME_SECONDS)

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
