#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RC car control with Dynamixel hardware-error check and startup recovery.

Behavior:
1. On startup:
   - Read motor status.
   - If Hardware Error Status(70) is not zero, print the error, reboot that motor,
     then reconfigure it and torque on.
   - If no error, configure and torque on directly.

2. While running:
   - Periodically read Torque Enable(64) and Hardware Error Status(70).
   - If hardware error appears, print the full status and exit.
   - It does NOT reboot while the car is moving.
   - Next program start will recover the motor.

Hardware assumption:
- /dev/ttyUSB0 = U2D2
- ID 1 = throttle motor, Velocity Mode
- ID 2 = steering motor, Position Mode
- PS controller through pygame joystick
"""

import sys
import time
import pygame
from dynamixel_sdk import *


# ============================================================
# Basic settings
# ============================================================

PORT = "/dev/ttyUSB0"
BAUD = 57600

ID_THROTTLE = 1
ID_STEER = 2


# ============================================================
# Dynamixel control table for XL330-M288-T / Protocol 2.0
# ============================================================

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70

ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116

ADDR_PRESENT_PWM = 124
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VOLTAGE = 144
ADDR_PRESENT_TEMP = 146

VELOCITY_MODE = 1
POSITION_MODE = 3

TORQUE_OFF = 0
TORQUE_ON = 1

# Dynamixel Protocol 2.0 Alert Bit.
# When this bit appears in dxl_error, it means the packet was returned,
# but the motor is warning that Hardware Error Status(70) should be checked.
DXL_ALERT = 0x80


# ============================================================
# Car control settings
# ============================================================

# Your weak DC converter makes 100 rpm risky.
# Use 50-60 for testing. After power is upgraded, you can try 80 or 100.
TARGET_RPM = 60

VELOCITY_UNIT_RPM = 0.229

MAX_STEER_DEG = 50

# Same as your original program.
# If the car does not drive straight, calibrate this number.
STEER_CENTER_POS = 2010

DEADZONE = 0.18
STEP = 0.10

# Check motor status every 0.25 second while running.
CHECK_INTERVAL = 0.25


# ============================================================
# Global objects
# ============================================================

port = PortHandler(PORT)
packet = PacketHandler(2.0)

last_vel = None
last_pos = None


# ============================================================
# Utility functions
# ============================================================

def to_uint32(value):
    value = int(value)
    if value < 0:
        value += 4294967296
    return value


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


def deg_to_unit(deg):
    return int(deg * 4096 / 360)


def quantize(val):
    if abs(val) < DEADZONE:
        return 0.0

    sign = 1 if val >= 0 else -1
    q = round(abs(val) / STEP) * STEP
    return sign * max(0.0, min(1.0, q))


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


def check_comm(label, comm_result, dxl_error, allow_alert=False):
    if comm_result != COMM_SUCCESS:
        print("[COMM ERROR]", label, packet.getTxRxResult(comm_result))
        return False

    if dxl_error != 0:
        if allow_alert and (dxl_error & DXL_ALERT):
            # This is not a communication failure.
            # The motor is telling us there is a hardware alert.
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
        raise RuntimeError(label)
    return True


def write4(motor_id, addr, value, label, allow_alert=False):
    comm, err = packet.write4ByteTxRx(port, motor_id, addr, value)
    if not check_comm(label, comm, err, allow_alert=allow_alert):
        raise RuntimeError(label)
    return True


# ============================================================
# Motor status
# ============================================================

def read_basic_status(motor_id, name):
    # allow_alert=True is very important.
    # When the motor is in hardware error, reading Torque Enable(64) may return
    # the Alert Bit, but the value itself is still readable.
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

    if torque is None or hw is None:
        return None

    return {
        "torque": torque,
        "hw_error": hw,
    }


def print_full_status(motor_id, name):
    print("")
    print("========== MOTOR STATUS:", name, "ID", motor_id, "==========")

    torque = read1(motor_id, ADDR_TORQUE_ENABLE, name + " Torque Enable(64)", allow_alert=True)
    hw = read1(motor_id, ADDR_HARDWARE_ERROR, name + " Hardware Error Status(70)", allow_alert=True)
    pwm_raw = read2(motor_id, ADDR_PRESENT_PWM, name + " Present PWM(124)", allow_alert=True)
    curr_raw = read2(motor_id, ADDR_PRESENT_CURRENT, name + " Present Current(126)", allow_alert=True)
    vel_raw = read4(motor_id, ADDR_PRESENT_VELOCITY, name + " Present Velocity(128)", allow_alert=True)
    pos_raw = read4(motor_id, ADDR_PRESENT_POSITION, name + " Present Position(132)", allow_alert=True)
    volt_raw = read2(motor_id, ADDR_PRESENT_VOLTAGE, name + " Present Voltage(144)", allow_alert=True)
    temp_raw = read1(motor_id, ADDR_PRESENT_TEMP, name + " Present Temp(146)", allow_alert=True)

    if torque is not None:
        print("Torque Enable:", torque)

    if hw is not None:
        print("Hardware Error Status:", hw, "hex =", hex(hw))
        print("Decoded Error:", decode_hw_error(hw))

    if pwm_raw is not None:
        print("Present PWM:", s16(pwm_raw))

    if curr_raw is not None:
        print("Present Current:", s16(curr_raw), "mA approx")

    if vel_raw is not None:
        print("Present Velocity:", round(s32(vel_raw) * VELOCITY_UNIT_RPM, 2), "rpm")

    if pos_raw is not None:
        print("Present Position:", s32(pos_raw))

    if volt_raw is not None:
        print("Present Voltage:", round(volt_raw / 10.0, 2), "V")

    if temp_raw is not None:
        print("Present Temperature:", temp_raw, "C")

    print("================================================")
    print("")


def reboot_motor(motor_id, name):
    print("[REBOOT]", name, "ID", motor_id)

    comm, err = packet.reboot(port, motor_id)

    # When the motor is currently in hardware error, the reboot command response
    # may also include the alert bit. That is OK.
    if not check_comm("REBOOT " + name, comm, err, allow_alert=True):
        print("[FATAL] Reboot command failed:", name)
        return False

    time.sleep(1.0)
    print("[REBOOT] Done:", name)
    return True


# ============================================================
# Motor configuration
# ============================================================

def configure_throttle_motor():
    print("[CONFIG] THROTTLE motor")

    write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_OFF, "throttle torque off", allow_alert=True)
    time.sleep(0.05)

    write1(ID_THROTTLE, ADDR_OPERATING_MODE, VELOCITY_MODE, "throttle velocity mode", allow_alert=True)
    write4(ID_THROTTLE, ADDR_PROFILE_ACCEL, 200, "throttle profile accel", allow_alert=True)
    write4(ID_THROTTLE, ADDR_GOAL_VELOCITY, 0, "throttle goal velocity zero", allow_alert=True)

    write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_ON, "throttle torque on", allow_alert=True)
    time.sleep(0.05)

    print("[CONFIG] THROTTLE ready")


def configure_steer_motor():
    print("[CONFIG] STEER motor")

    write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_OFF, "steer torque off", allow_alert=True)
    time.sleep(0.05)

    write1(ID_STEER, ADDR_OPERATING_MODE, POSITION_MODE, "steer position mode", allow_alert=True)
    write4(ID_STEER, ADDR_PROFILE_ACCEL, 100, "steer profile accel", allow_alert=True)
    write4(ID_STEER, ADDR_PROFILE_VELOCITY, 500, "steer profile velocity", allow_alert=True)

    write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_ON, "steer torque on", allow_alert=True)
    time.sleep(0.05)

    write4(ID_STEER, ADDR_GOAL_POSITION, STEER_CENTER_POS, "steer center position", allow_alert=True)

    print("[CONFIG] STEER ready")


# ============================================================
# Startup recovery
# ============================================================

def recover_if_needed(motor_id, name, configure_func):
    print("[STARTUP CHECK]", name)

    status = read_basic_status(motor_id, name)

    if status is None:
        print("[FATAL] Cannot read status:", name)
        return False

    hw = status["hw_error"]
    torque = status["torque"]

    print("[STATUS]", name, "Torque =", torque, "HW Error =", hw, "hex =", hex(hw))
    print("[STATUS]", name, "Error Decode =", decode_hw_error(hw))

    if hw != 0:
        print("[WARNING]", name, "has hardware error before start.")
        print_full_status(motor_id, name)

        print("[ACTION]", name, "will reboot now.")
        if not reboot_motor(motor_id, name):
            return False

        status_after = read_basic_status(motor_id, name)

        if status_after is None:
            print("[FATAL]", name, "cannot read status after reboot.")
            return False

        if status_after["hw_error"] != 0:
            print("[FATAL]", name, "still has hardware error after reboot.")
            print_full_status(motor_id, name)
            return False

        print("[OK]", name, "hardware error cleared after reboot.")
    else:
        print("[OK]", name, "no hardware error. No reboot needed.")

    print("[ACTION]", name, "configure and torque on.")
    configure_func()

    final_status = read_basic_status(motor_id, name)

    if final_status is None:
        print("[FATAL]", name, "cannot read final status after config.")
        return False

    if final_status["hw_error"] != 0:
        print("[FATAL]", name, "hardware error came back after config.")
        print_full_status(motor_id, name)
        return False

    if final_status["torque"] != TORQUE_ON:
        print("[FATAL]", name, "torque is still OFF after config.")
        print_full_status(motor_id, name)
        return False

    print("[OK]", name, "ready.")
    return True


# ============================================================
# Runtime monitor and safe stop
# ============================================================

def monitor_motor_or_exit(motor_id, name):
    status = read_basic_status(motor_id, name)

    if status is None:
        print("")
        print("[FATAL]", name, "communication lost while running.")
        print("[FATAL] Program will exit.")
        raise RuntimeError(name + " communication lost")

    torque = status["torque"]
    hw = status["hw_error"]

    if hw != 0:
        print("")
        print("[FATAL]", name, "hardware error detected while running.")
        print("[FATAL]", name, "Error Decode =", decode_hw_error(hw))
        print("[FATAL] Program will exit. Restart the program to recover.")
        print_full_status(motor_id, name)
        raise RuntimeError(name + " hardware error")

    if torque != TORQUE_ON:
        print("")
        print("[FATAL]", name, "torque is OFF while running.")
        print("[FATAL] Program will exit.")
        print_full_status(motor_id, name)
        raise RuntimeError(name + " torque off")


def safe_stop():
    print("")
    print("[SAFE STOP] Sending stop commands...")

    try:
        write4(ID_THROTTLE, ADDR_GOAL_VELOCITY, 0, "safe stop throttle", allow_alert=True)
    except Exception as e:
        print("[SAFE STOP] throttle stop failed:", e)

    try:
        write4(ID_STEER, ADDR_GOAL_POSITION, STEER_CENTER_POS, "safe stop steer", allow_alert=True)
    except Exception as e:
        print("[SAFE STOP] steer center failed:", e)

    time.sleep(0.3)

    try:
        write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_OFF, "safe torque off throttle", allow_alert=True)
    except Exception as e:
        print("[SAFE STOP] throttle torque off failed:", e)

    try:
        write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_OFF, "safe torque off steer", allow_alert=True)
    except Exception as e:
        print("[SAFE STOP] steer torque off failed:", e)

    print("[SAFE STOP] Done")


# ============================================================
# Joystick setup
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


# ============================================================
# Main program
# ============================================================

def main():
    global last_vel, last_pos

    print("Opening port:", PORT)

    if not port.openPort():
        print("[FATAL] Failed to open port:", PORT)
        sys.exit(1)

    if not port.setBaudRate(BAUD):
        print("[FATAL] Failed to set baudrate:", BAUD)
        port.closePort()
        sys.exit(1)

    print("[OK] Port opened")

    try:
        # Startup recovery.
        ok_throttle = recover_if_needed(ID_THROTTLE, "THROTTLE", configure_throttle_motor)
        ok_steer = recover_if_needed(ID_STEER, "STEER", configure_steer_motor)

        if not ok_throttle or not ok_steer:
            print("[FATAL] Startup motor recovery failed.")
            raise RuntimeError("startup recovery failed")

        print_full_status(ID_THROTTLE, "THROTTLE")
        print_full_status(ID_STEER, "STEER")

        # Joystick.
        joy = setup_joystick()
        if joy is None:
            raise RuntimeError("no joystick")

        print("[RUN] RC control started")
        print("[RUN] Runtime error policy:")
        print("      hardware error -> print status -> exit")
        print("      next start -> auto reboot/recover")

        last_check_time = 0.0
        last_vel = None
        last_pos = None

        while True:
            now = time.time()

            if now - last_check_time >= CHECK_INTERVAL:
                last_check_time = now
                monitor_motor_or_exit(ID_THROTTLE, "THROTTLE")
                monitor_motor_or_exit(ID_STEER, "STEER")

            pygame.event.pump()

            raw_steer = joy.get_axis(0)
            raw_throttle = joy.get_axis(4)

            q_steer = quantize(raw_steer)
            q_throttle = quantize(raw_throttle)

            target_rpm = q_throttle * TARGET_RPM
            vel_val = int(target_rpm / VELOCITY_UNIT_RPM)

            steer_deg = q_steer * MAX_STEER_DEG
            pos_val = int(STEER_CENTER_POS + deg_to_unit(steer_deg))

            if vel_val != last_vel:
                write4(
                    ID_THROTTLE,
                    ADDR_GOAL_VELOCITY,
                    to_uint32(vel_val),
                    "write throttle goal velocity",
                    allow_alert=True
                )
                last_vel = vel_val

            if pos_val != last_pos:
                write4(
                    ID_STEER,
                    ADDR_GOAL_POSITION,
                    pos_val,
                    "write steer goal position",
                    allow_alert=True
                )
                last_pos = pos_val

            print(
                "Steer: {:+.0f}% | Throttle: {:+.0f}% | RPM limit: {}".format(
                    q_steer * 100,
                    q_throttle * 100,
                    TARGET_RPM
                ),
                end="\r"
            )

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("")
        print("[KEYBOARD] Ctrl-C detected.")

    except RuntimeError as e:
        print("")
        print("[RUNTIME EXIT]", e)

    finally:
        safe_stop()
        port.closePort()
        pygame.quit()
        print("[END] Program exited.")


if __name__ == "__main__":
    main()
