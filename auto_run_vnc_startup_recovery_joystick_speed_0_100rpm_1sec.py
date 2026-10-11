#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
VNC / Raspberry Pi normal Python code.

This version adds the startup recovery from your RC recovery file BEFORE the timed auto-run.

Goal:
- NOT Pico / NOT Thonny
- On startup, check Dynamixel hardware errors
- If a motor has Hardware Error Status(70), reboot it first, then configure it
- Then run timed auto mode
- Run time = 1 second
- Speed is controlled by joystick during that 1 second
- Speed range = 0 to 100 RPM
- Steering resets to calibrated straight position before and after the run
- Positive speed should move forward

Hardware assumption:
- /dev/ttyUSB0 = U2D2
- ID 1 = throttle motor, Velocity Mode
- ID 2 = steering motor, Position Mode
- PS controller through pygame joystick

Run:
    python3 auto_run_vnc_startup_recovery_joystick_speed_0_100rpm_1sec.py
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
DXL_ALERT = 0x80


# ============================================================
# Car settings
# ============================================================

RUN_TIME_SECONDS = 5.0

# Joystick speed range
MIN_RPM = 0
MAX_RPM = 100

# The car previously went backward, so throttle direction is reversed.
# Keep this as -1 so positive calculated RPM moves forward.
THROTTLE_DIRECTION = -1

VELOCITY_UNIT_RPM = 0.229

# Same joystick axis as your RC file used for throttle.
THROTTLE_AXIS = 4

# Many PS controllers give negative value when pushing stick upward.
# This setting makes pushing forward/up become positive speed.
# If your joystick works opposite, change this to 1.
JOYSTICK_FORWARD_SIGN = -1

DEADZONE = 0.08

# Check motor status every 0.25 second while running.
CHECK_INTERVAL = 0.25


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


def check_comm(label, comm_result, dxl_error, allow_alert=False):
    if comm_result != COMM_SUCCESS:
        print("[COMM ERROR]", label, packet.getTxRxResult(comm_result))
        return False

    if dxl_error != 0:
        if allow_alert and (dxl_error & DXL_ALERT):
            # This is not a communication failure.
            # The motor is telling us Hardware Error Status(70) should be checked.
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


def rpm_to_raw_velocity(rpm):
    return int(rpm / VELOCITY_UNIT_RPM)


# ============================================================
# Motor status and startup recovery
# ============================================================

def read_basic_status(motor_id, name):
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

    if not check_comm("REBOOT " + name, comm, err, allow_alert=True):
        print("[FATAL] Reboot command failed:", name)
        return False

    time.sleep(1.0)
    print("[REBOOT] Done:", name)
    return True


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
    """
    Convert joystick axis to forward speed from 0 to 100 RPM.

    With JOYSTICK_FORWARD_SIGN = -1:
        pushing the stick forward/up usually gives positive speed.

    This version only allows forward speed:
        below deadzone or pulling backward => 0 RPM
        full forward => 100 RPM
    """

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
    monitor_motor_or_exit(ID_THROTTLE, "THROTTLE")
    monitor_motor_or_exit(ID_STEER, "STEER")

    start_time = time.time()
    last_print_time = 0.0
    last_check_time = 0.0

    while time.time() - start_time < run_time_seconds:
        now = time.time()

        if now - last_check_time >= CHECK_INTERVAL:
            last_check_time = now
            monitor_motor_or_exit(ID_THROTTLE, "THROTTLE")
            monitor_motor_or_exit(ID_STEER, "STEER")

        pygame.event.pump()

        raw_throttle = joy.get_axis(THROTTLE_AXIS)
        target_rpm = joystick_axis_to_speed_rpm(raw_throttle)

        set_throttle_rpm(target_rpm)

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

    print("[OK] Port opened")

    try:
        # ====================================================
        # Startup recovery is added here BEFORE running.
        # This comes from your rc_with_dxl_error_check_recover.py logic.
        # ====================================================
        ok_throttle = recover_if_needed(ID_THROTTLE, "THROTTLE", configure_throttle_motor)
        ok_steer = recover_if_needed(ID_STEER, "STEER", configure_steer_motor)

        if not ok_throttle or not ok_steer:
            print("[FATAL] Startup motor recovery failed.")
            raise RuntimeError("startup recovery failed")

        print_full_status(ID_THROTTLE, "THROTTLE")
        print_full_status(ID_STEER, "STEER")

        joy = setup_joystick()
        if joy is None:
            raise RuntimeError("no joystick")

        # Timed run:
        # duration = 1 second
        # speed is controlled by joystick from 0 to 100 RPM
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
