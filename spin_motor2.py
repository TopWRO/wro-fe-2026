#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Spin DYNAMIXEL ID 1 (drive / throttle motor) in velocity mode for 3 seconds.

WARNING: lift the car so the wheels are off the ground before running.

Steps:
  1. Check for hardware error (abort if any)
  2. Torque off -> velocity mode -> torque on
     (same as configure_throttle() in dxl_recover_test.py)
  3. Spin at ~50 rpm for 3 seconds, printing present velocity
  4. Stop (goal velocity 0), leave in velocity mode

Ctrl+C during the spin still stops the motor.
"""

import sys
import time
from dynamixel_sdk import *

PORT = "/dev/ttyUSB0"
BAUD = 57600

ID_THROTTLE = 1

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCEL = 108
ADDR_PRESENT_VELOCITY = 128

VELOCITY_MODE = 1

TORQUE_OFF = 0
TORQUE_ON = 1

SPIN_SECONDS = 3.0
SPIN_RPM = 50
VELOCITY_UNIT_RPM = 0.229
SPIN_VELOCITY = int(round(SPIN_RPM / VELOCITY_UNIT_RPM))   # 218 -> 49.9 rpm

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
    print("[CONFIG] THROTTLE -> velocity mode")

    write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_OFF, "throttle torque off")
    time.sleep(0.05)

    ok = write1(ID_THROTTLE, ADDR_OPERATING_MODE, VELOCITY_MODE, "throttle velocity mode")
    ok = write4(ID_THROTTLE, ADDR_PROFILE_ACCEL, 200, "throttle profile accel") and ok
    ok = write4(ID_THROTTLE, ADDR_GOAL_VELOCITY, 0, "throttle goal velocity zero") and ok
    ok = write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_ON, "throttle torque on") and ok
    time.sleep(0.05)

    return ok


def stop_motor():
    print("[STOP] THROTTLE goal velocity 0")
    write4(ID_THROTTLE, ADDR_GOAL_VELOCITY, 0, "throttle goal velocity zero")


def spin():
    print("[SPIN] ID", ID_THROTTLE, "goal velocity", SPIN_VELOCITY,
          "(" + str(round(SPIN_VELOCITY * VELOCITY_UNIT_RPM, 1)) + " rpm) for",
          SPIN_SECONDS, "s")

    write4(ID_THROTTLE, ADDR_GOAL_VELOCITY, SPIN_VELOCITY, "throttle goal velocity")

    t0 = time.monotonic()
    while time.monotonic() - t0 < SPIN_SECONDS:
        raw = read4(ID_THROTTLE, ADDR_PRESENT_VELOCITY, "throttle present velocity")
        if raw is not None:
            print("  t=%.1fs  velocity=%.1f rpm"
                  % (time.monotonic() - t0, s32(raw) * VELOCITY_UNIT_RPM))
        time.sleep(0.25)

    write4(ID_THROTTLE, ADDR_GOAL_VELOCITY, 0, "throttle goal velocity zero")
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

    hw = read1(ID_THROTTLE, ADDR_HARDWARE_ERROR, "throttle hardware error")
    if hw is None or hw != 0:
        print("[FATAL] Cannot read ID", ID_THROTTLE, "or it has a hardware error:", hw)
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
        stop_motor()
        port.closePort()
        print("[END] Port closed")


if __name__ == "__main__":
    main()
