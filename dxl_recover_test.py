#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
from dynamixel_sdk import *

PORT = "/dev/ttyUSB0"
BAUD = 57600

ID_THROTTLE = 1
ID_STEER = 2

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VOLTAGE = 144
ADDR_PRESENT_TEMP = 146

VELOCITY_MODE = 1
POSITION_MODE = 3

TORQUE_OFF = 0
TORQUE_ON = 1

DXL_ALERT = 0x80

port = PortHandler(PORT)
packet = PacketHandler(2.0)


def s16(value):
    value = int(value)
    if value & 0x8000:
        return value - 65536
    return value


def s32(value):
    value = int(value)
    if value & 0x80000000:
        return value - 4294967296
    return value


def decode_hw_error(hw):
    errors = []

    if hw & 0x01:
        errors.append("Input Voltage Error")
    if hw & 0x04:
        errors.append("Overheating Error")
    if hw & 0x10:
        errors.append("Electrical Shock / Insufficient Power")
    if hw & 0x20:
        errors.append("Overload Error")

    if not errors:
        errors.append("No hardware error")

    return errors


def check_comm(label, comm_result, dxl_error, allow_alert=False):
    if comm_result != COMM_SUCCESS:
        print("[COMM ERROR]", label, packet.getTxRxResult(comm_result))
        return False

    if dxl_error != 0:
        # 0x80 means Alert Bit.
        # It tells us hardware error exists, but the read itself can still be valid.
        if allow_alert and (dxl_error & DXL_ALERT):
            print("[ALERT]", label, "Status packet has hardware alert bit 0x80.")
            print("[ALERT]", label, "Continue reading Hardware Error Status(70).")
            return True

        print("[DXL ERROR]", label, packet.getRxPacketError(dxl_error))
        return False

    return True


def read1(motor_id, addr, label, allow_alert=False):
    value, comm, err = packet.read1ByteTxRx(port, motor_id, addr)
    if not check_comm(label, comm, err, allow_alert=allow_alert):
        return None
    return value


def read2(motor_id, addr, label, allow_alert=False):
    value, comm, err = packet.read2ByteTxRx(port, motor_id, addr)
    if not check_comm(label, comm, err, allow_alert=allow_alert):
        return None
    return value


def read4(motor_id, addr, label, allow_alert=False):
    value, comm, err = packet.read4ByteTxRx(port, motor_id, addr)
    if not check_comm(label, comm, err, allow_alert=allow_alert):
        return None
    return value


def write1(motor_id, addr, value, label, allow_alert=False):
    comm, err = packet.write1ByteTxRx(port, motor_id, addr, value)
    if not check_comm(label, comm, err, allow_alert=allow_alert):
        return False
    return True


def write4(motor_id, addr, value, label, allow_alert=False):
    comm, err = packet.write4ByteTxRx(port, motor_id, addr, value)
    if not check_comm(label, comm, err, allow_alert=allow_alert):
        return False
    return True


def read_status(motor_id, name):
    print("")
    print("========== STATUS:", name, "ID", motor_id, "==========")

    torque = read1(
        motor_id,
        ADDR_TORQUE_ENABLE,
        name + " Torque Enable(64)",
        allow_alert=True
    )

    hw = read1(
        motor_id,
        ADDR_HARDWARE_ERROR,
        name + " Hardware Error Status(70)",
        allow_alert=True
    )

    voltage_raw = read2(
        motor_id,
        ADDR_PRESENT_VOLTAGE,
        name + " Present Voltage(144)",
        allow_alert=True
    )

    temp = read1(
        motor_id,
        ADDR_PRESENT_TEMP,
        name + " Present Temperature(146)",
        allow_alert=True
    )

    current_raw = read2(
        motor_id,
        ADDR_PRESENT_CURRENT,
        name + " Present Current(126)",
        allow_alert=True
    )

    velocity_raw = read4(
        motor_id,
        ADDR_PRESENT_VELOCITY,
        name + " Present Velocity(128)",
        allow_alert=True
    )

    position_raw = read4(
        motor_id,
        ADDR_PRESENT_POSITION,
        name + " Present Position(132)",
        allow_alert=True
    )

    if torque is not None:
        print("Torque Enable:", torque)

    if hw is not None:
        print("Hardware Error Status:", hw, "hex =", hex(hw))
        print("Decoded Error:", decode_hw_error(hw))

    if voltage_raw is not None:
        print("Voltage:", voltage_raw / 10.0, "V")

    if temp is not None:
        print("Temperature:", temp, "C")

    if current_raw is not None:
        print("Current:", s16(current_raw), "mA approx")

    if velocity_raw is not None:
        print("Velocity:", round(s32(velocity_raw) * 0.229, 2), "rpm")

    if position_raw is not None:
        print("Position:", s32(position_raw))

    print("========================================")
    print("")

    if torque is None or hw is None:
        return None

    return {
        "torque": torque,
        "hw_error": hw,
    }


def reboot_motor(motor_id, name):
    print("[REBOOT]", name, "ID", motor_id)

    comm, err = packet.reboot(port, motor_id)

    # Reboot command may also return alert bit when the motor is already in hardware error.
    # So allow_alert=True here.
    ok = check_comm(
        "REBOOT " + name,
        comm,
        err,
        allow_alert=True
    )

    if not ok:
        print("[FATAL] Reboot command failed:", name)
        return False

    time.sleep(1.0)

    print("[REBOOT] Done:", name)
    return True


def configure_throttle():
    print("[CONFIG] THROTTLE")

    write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_OFF, "throttle torque off", allow_alert=True)
    time.sleep(0.05)

    write1(ID_THROTTLE, ADDR_OPERATING_MODE, VELOCITY_MODE, "throttle velocity mode")
    write4(ID_THROTTLE, ADDR_PROFILE_ACCEL, 200, "throttle profile accel")
    write4(ID_THROTTLE, ADDR_GOAL_VELOCITY, 0, "throttle goal velocity zero")

    write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_ON, "throttle torque on")
    time.sleep(0.05)

    print("[CONFIG] THROTTLE ready")


def configure_steer():
    print("[CONFIG] STEER")

    write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_OFF, "steer torque off", allow_alert=True)
    time.sleep(0.05)

    write1(ID_STEER, ADDR_OPERATING_MODE, POSITION_MODE, "steer position mode")
    write4(ID_STEER, ADDR_PROFILE_ACCEL, 100, "steer profile accel")
    write4(ID_STEER, ADDR_PROFILE_VELOCITY, 500, "steer profile velocity")
    write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_ON, "steer torque on")
    write4(ID_STEER, ADDR_GOAL_POSITION, 2047, "steer center position")

    time.sleep(0.05)

    print("[CONFIG] STEER ready")


def recover_if_needed(motor_id, name, configure_func):
    status = read_status(motor_id, name)

    if status is None:
        print("[FATAL] Cannot read status:", name)
        return False

    hw = status["hw_error"]
    torque = status["torque"]

    if hw != 0:
        print("[WARNING]", name, "has hardware error before start.")
        print("[WARNING]", name, "error =", decode_hw_error(hw))
        print("[ACTION]", name, "will reboot now.")

        ok = reboot_motor(motor_id, name)

        if not ok:
            return False

        status_after = read_status(motor_id, name)

        if status_after is None:
            print("[FATAL]", name, "cannot read status after reboot.")
            return False

        if status_after["hw_error"] != 0:
            print("[FATAL]", name, "still has hardware error after reboot.")
            print("[FATAL]", name, "error =", decode_hw_error(status_after["hw_error"]))
            return False

        print("[OK]", name, "hardware error cleared after reboot.")

    else:
        print("[OK]", name, "has no hardware error.")

    print("[ACTION]", name, "configure and torque on.")
    configure_func()

    status_final = read_status(motor_id, name)

    if status_final is None:
        return False

    if status_final["hw_error"] != 0:
        print("[FATAL]", name, "hardware error came back after torque on.")
        print("[FATAL]", name, "error =", decode_hw_error(status_final["hw_error"]))
        return False

    if status_final["torque"] != TORQUE_ON:
        print("[FATAL]", name, "torque is still OFF after configure.")
        return False

    print("[OK]", name, "recovered and ready.")
    return True


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

    ok_throttle = recover_if_needed(ID_THROTTLE, "THROTTLE", configure_throttle)
    ok_steer = recover_if_needed(ID_STEER, "STEER", configure_steer)

    if ok_throttle and ok_steer:
        print("")
        print("[SUCCESS] Both motors are ready.")
    else:
        print("")
        print("[FAILED] At least one motor is not ready.")

    port.closePort()
    print("[END] Port closed")


if __name__ == "__main__":
    main()