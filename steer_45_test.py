#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Swing the steering motor (DYNAMIXEL ID 2) 45 deg left and 45 deg right, 3 times.

Only the steering motor moves; the drive motor (ID 1) is not touched.
Keep fingers away from the steering linkage while it runs.

Steps:
  1. Check for hardware error (abort if any)
  2. Torque off -> position mode -> torque on, go to center
     (same as configure_steer_motor() in rc_with_dxl_error_check_recover.py)
  3. Repeat 3 times: left 45 deg -> right 45 deg
  4. Return to center, leave torque on

Ctrl+C at any time returns the steering to center.
"""

import sys
import time
from dynamixel_sdk import *

PORT = "/dev/ttyUSB0"
BAUD = 57600

ID_STEER = 2

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

POSITION_MODE = 3

TORQUE_OFF = 0
TORQUE_ON = 1

# Same values as rc_with_dxl_error_check_recover.py
STEER_CENTER_POS = 2010
MAX_STEER_DEG = 50

TURN_DEG = 45
REPEATS = 3
HOLD_SECONDS = 1.0

# In the RC script, joystick right (+) adds position units.
# If the wheels turn the wrong way, change this to -1.
RIGHT_SIGN = 1

port = PortHandler(PORT)
packet = PacketHandler(2.0)


def s32(value):
    value = int(value)
    if value & 0x80000000:
        return value - 4294967296
    return value


def deg_to_unit(deg):
    return int(deg * 4096 / 360)


def check_comm(label, comm_result, dxl_error):
    if comm_result != COMM_SUCCESS:
        print("[COMM ERROR]", label, packet.getTxRxResult(comm_result))
        return False
    if dxl_error != 0:
        print("[DXL ERROR]", label, packet.getRxPacketError(dxl_error))
        return False
    return True


def read1(motor_id, addr, label):
    value, comm, err = packet.read1ByteTxRx(port, motor_id, addr)
    if not check_comm(label, comm, err):
        return None
    return value


def read4(motor_id, addr, label):
    value, comm, err = packet.read4ByteTxRx(port, motor_id, addr)
    if not check_comm(label, comm, err):
        return None
    return value


def write1(motor_id, addr, value, label):
    comm, err = packet.write1ByteTxRx(port, motor_id, addr, value)
    return check_comm(label, comm, err)


def write4(motor_id, addr, value, label):
    comm, err = packet.write4ByteTxRx(port, motor_id, addr, value)
    return check_comm(label, comm, err)


def start_position_mode():
    print("[CONFIG] STEER -> position mode")

    write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_OFF, "steer torque off")
    time.sleep(0.05)

    ok = write1(ID_STEER, ADDR_OPERATING_MODE, POSITION_MODE, "steer position mode")
    ok = write4(ID_STEER, ADDR_PROFILE_ACCEL, 100, "steer profile accel") and ok
    ok = write4(ID_STEER, ADDR_PROFILE_VELOCITY, 500, "steer profile velocity") and ok
    ok = write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_ON, "steer torque on") and ok
    ok = write4(ID_STEER, ADDR_GOAL_POSITION, STEER_CENTER_POS, "steer center position") and ok
    time.sleep(0.5)

    return ok


def steer_to(deg, label):
    deg = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, deg))
    goal = STEER_CENTER_POS + RIGHT_SIGN * deg_to_unit(deg)
    print("[STEER] %-6s %+d deg -> goal %d" % (label, deg, goal))

    write4(ID_STEER, ADDR_GOAL_POSITION, goal, "steer goal position")
    time.sleep(HOLD_SECONDS)

    raw = read4(ID_STEER, ADDR_PRESENT_POSITION, "steer present position")
    if raw is not None:
        print("         present position %d" % s32(raw))


def center():
    print("[CENTER] STEER goal position", STEER_CENTER_POS)
    write4(ID_STEER, ADDR_GOAL_POSITION, STEER_CENTER_POS, "steer center position")


def swing():
    for i in range(1, REPEATS + 1):
        print("--- round %d/%d ---" % (i, REPEATS))
        steer_to(-TURN_DEG, "left")
        steer_to(TURN_DEG, "right")
    print("[SWING] done")


def main():
    print("Opening port:", PORT)

    if not port.openPort():
        print("[FATAL] Failed to open port:", PORT)
        sys.exit(1)

    if not port.setBaudRate(BAUD):
        print("[FATAL] Failed to set baudrate:", BAUD)
        port.closePort()
        sys.exit(1)

    print("[OK] Port opened")

    hw = read1(ID_STEER, ADDR_HARDWARE_ERROR, "steer hardware error")
    if hw is None or hw != 0:
        print("[FATAL] Cannot read ID", ID_STEER, "or it has a hardware error:", hw)
        print("[HINT] Run dxl_recover_test.py first.")
        port.closePort()
        sys.exit(1)

    try:
        if start_position_mode():
            swing()
        else:
            print("[FAILED] Could not switch to position mode")
    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C")
    finally:
        center()
        time.sleep(0.5)
        port.closePort()
        print("[END] Port closed")


if __name__ == "__main__":
    main()
