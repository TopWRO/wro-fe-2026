#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Spin DYNAMIXEL ID 2 (steering motor) in velocity mode for 3 seconds.

WARNING: only run this with the steering linkage DETACHED from ID 2.

Steps:
  1. Check for hardware error (abort if any)
  2. Torque off -> velocity mode -> torque on
  3. Spin at ~50 rpm for 3 seconds, printing present velocity
  4. Stop, switch back to position mode and center at 2047
     (same as configure_steer() in dxl_recover_test.py)

Ctrl+C during the spin stops the motor and still restores position mode.
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
ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_VELOCITY = 128

VELOCITY_MODE = 1
POSITION_MODE = 3

TORQUE_OFF = 0
TORQUE_ON = 1

SPIN_SECONDS = 3.0
SPIN_RPM = 50
VELOCITY_UNIT_RPM = 0.229
SPIN_VELOCITY = int(round(SPIN_RPM / VELOCITY_UNIT_RPM))   # 218 -> 49.9 rpm

STEER_CENTER = 2047

port = PortHandler(PORT)
packet = PacketHandler(2.0)


def s32(value):
    value = int(value)
    if value & 0x80000000:
        return value - 4294967296
    return value


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


def start_velocity_mode():
    print("[CONFIG] STEER -> velocity mode")

    write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_OFF, "steer torque off")
    time.sleep(0.05)

    ok = write1(ID_STEER, ADDR_OPERATING_MODE, VELOCITY_MODE, "steer velocity mode")
    ok = write4(ID_STEER, ADDR_PROFILE_ACCEL, 200, "steer profile accel") and ok
    ok = write4(ID_STEER, ADDR_GOAL_VELOCITY, 0, "steer goal velocity zero") and ok
    ok = write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_ON, "steer torque on") and ok
    time.sleep(0.05)

    return ok


def restore_position_mode():
    print("[CONFIG] STEER -> position mode, center", STEER_CENTER)

    write4(ID_STEER, ADDR_GOAL_VELOCITY, 0, "steer goal velocity zero")
    time.sleep(0.3)

    write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_OFF, "steer torque off")
    time.sleep(0.05)

    write1(ID_STEER, ADDR_OPERATING_MODE, POSITION_MODE, "steer position mode")
    write4(ID_STEER, ADDR_PROFILE_ACCEL, 100, "steer profile accel")
    write4(ID_STEER, ADDR_PROFILE_VELOCITY, 500, "steer profile velocity")
    write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_ON, "steer torque on")
    write4(ID_STEER, ADDR_GOAL_POSITION, STEER_CENTER, "steer center position")
    time.sleep(1.0)

    print("[CONFIG] STEER restored")


def spin():
    print("[SPIN] ID", ID_STEER, "goal velocity", SPIN_VELOCITY,
          "(" + str(round(SPIN_VELOCITY * VELOCITY_UNIT_RPM, 1)) + " rpm) for",
          SPIN_SECONDS, "s")

    write4(ID_STEER, ADDR_GOAL_VELOCITY, SPIN_VELOCITY, "steer goal velocity")

    t0 = time.monotonic()
    while time.monotonic() - t0 < SPIN_SECONDS:
        raw = read4(ID_STEER, ADDR_PRESENT_VELOCITY, "steer present velocity")
        if raw is not None:
            print("  t=%.1fs  velocity=%.1f rpm"
                  % (time.monotonic() - t0, s32(raw) * VELOCITY_UNIT_RPM))
        time.sleep(0.25)

    write4(ID_STEER, ADDR_GOAL_VELOCITY, 0, "steer goal velocity zero")
    print("[SPIN] done")


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
        if start_velocity_mode():
            spin()
        else:
            print("[FAILED] Could not switch to velocity mode")
    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C")
    finally:
        restore_position_mode()
        port.closePort()
        print("[END] Port closed")


if __name__ == "__main__":
    main()
