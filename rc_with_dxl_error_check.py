#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import pygame
from dynamixel_sdk import *

# =========================
# Basic settings
# =========================

PORT = "/dev/ttyUSB0"
BAUD = 57600

ID_THROTTLE = 1
ID_STEER = 2

# =========================
# Dynamixel control table
# XL330-M288-T / Protocol 2.0
# =========================

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

# =========================
# Car settings
# =========================

# Safer value for weak power supply.
# You can change it back to 100 after fixing the DC converter.
TARGET_RPM = 60

VELOCITY_UNIT_RPM = 0.229

MAX_STEER_DEG = 50
STEER_CENTER_POS = 2047

DEADZONE = 0.18
STEP = 0.10

CHECK_INTERVAL = 0.25

# =========================
# Global objects
# =========================

port = PortHandler(PORT)
packet = PacketHandler(2.0)

last_vel = None
last_pos = None


# =========================
# Utility functions
# =========================

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


def decode_hw_error(err):
    names = []

    if err & 0x01:
        names.append("Input Voltage Error")
    if err & 0x04:
        names.append("Overheating Error")
    if err & 0x10:
        names.append("Electrical Shock / Insufficient Power")
    if err & 0x20:
        names.append("Overload Error")

    if not names:
        names.append("No hardware error")

    return names


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


def read2(motor_id, addr, label):
    value, comm, err = packet.read2ByteTxRx(port, motor_id, addr)
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
    if not check_comm(label, comm, err):
        raise RuntimeError(label)


def write4(motor_id, addr, value, label):
    comm, err = packet.write4ByteTxRx(port, motor_id, addr, value)
    if not check_comm(label, comm, err):
        raise RuntimeError(label)


# =========================
# Motor status functions
# =========================

def read_basic_status(motor_id, name):
    torque = read1(motor_id, ADDR_TORQUE_ENABLE, name + " torque")
    hw = read1(motor_id, ADDR_HARDWARE_ERROR, name + " hardware error")

    if torque is None or hw is None:
        return None

    return {
        "torque": torque,
        "hw_error": hw,
    }


def print_full_status(motor_id, name):
    print("")
    print("========== MOTOR STATUS ==========")
    print("Name:", name)
    print("ID:", motor_id)

    torque = read1(motor_id, ADDR_TORQUE_ENABLE, name + " torque")
    hw = read1(motor_id, ADDR_HARDWARE_ERROR, name + " hardware error")
    pwm_raw = read2(motor_id, ADDR_PRESENT_PWM, name + " present pwm")
    curr_raw = read2(motor_id, ADDR_PRESENT_CURRENT, name + " present current")
    vel_raw = read4(motor_id, ADDR_PRESENT_VELOCITY, name + " present velocity")
    pos_raw = read4(motor_id, ADDR_PRESENT_POSITION, name + " present position")
    volt_raw = read2(motor_id, ADDR_PRESENT_VOLTAGE, name + " present voltage")
    temp_raw = read1(motor_id, ADDR_PRESENT_TEMP, name + " present temp")

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

    print("==================================")
    print("")


def reboot_motor(motor_id, name):
    print("[REBOOT]", name, "ID", motor_id)

    comm, err = packet.reboot(port, motor_id)

    if not check_comm("reboot " + name, comm, err):
        print("[FATAL] Reboot command failed:", name)
        return False

    time.sleep(1.0)
    print("[REBOOT] Done:", name)
    return True


def startup_check_and_reboot_if_error(motor_id, name):
    print("[STARTUP CHECK]", name)

    status = read_basic_status(motor_id, name)

    if status is None:
        print("[FATAL] Cannot read motor status:", name)
        return False

    hw = status["hw_error"]

    if hw == 0:
        print("[OK]", name, "no hardware error. No reboot needed.")
        return True

    print("[WARNING]", name, "has hardware error before start.")
    print_full_status(motor_id, name)

    ok = reboot_motor(motor_id, name)

    if not ok:
        return False

    status_after = read_basic_status(motor_id, name)

    if status_after is None:
        print("[FATAL] Cannot read motor after reboot:", name)
        return False

    if status_after["hw_error"] != 0:
        print("[FATAL]", name, "still has hardware error after reboot.")
        print_full_status(motor_id, name)
        return False

    print("[OK]", name, "hardware error cleared after reboot.")
    return True


# =========================
# Motor configuration
# =========================

def configure_throttle_motor():
    print("[CONFIG] THROTTLE motor")

    write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_OFF, "throttle torque off")
    time.sleep(0.05)

    write1(ID_THROTTLE, ADDR_OPERATING_MODE, VELOCITY_MODE, "throttle velocity mode")
    write4(ID_THROTTLE, ADDR_PROFILE_ACCEL, 200, "throttle profile accel")
    write4(ID_THROTTLE, ADDR_GOAL_VELOCITY, 0, "throttle goal velocity zero")

    write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_ON, "throttle torque on")
    time.sleep(0.05)

    print("[CONFIG] THROTTLE ready")


def configure_steer_motor():
    print("[CONFIG] STEER motor")

    write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_OFF, "steer torque off")
    time.sleep(0.05)

    write1(ID_STEER, ADDR_OPERATING_MODE, POSITION_MODE, "steer position mode")
    write4(ID_STEER, ADDR_PROFILE_ACCEL, 100, "steer profile accel")
    write4(ID_STEER, ADDR_PROFILE_VELOCITY, 500, "steer profile velocity")

    write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_ON, "steer torque on")
    time.sleep(0.05)

    write4(ID_STEER, ADDR_GOAL_POSITION, STEER_CENTER_POS, "steer center position")

    print("[CONFIG] STEER ready")


# =========================
# Runtime monitor
# =========================

def monitor_motor_or_exit(motor_id, name):
    status = read_basic_status(motor_id, name)

    if status is None:
        print("")
        print("[FATAL]", name, "communication lost.")
        print("[FATAL] Program will exit.")
        raise RuntimeError(name + " communication lost")

    torque = status["torque"]
    hw = status["hw_error"]

    if hw != 0:
        print("")
        print("[FATAL]", name, "hardware error detected while running.")
        print("[FATAL] Program will exit now.")
        print_full_status(motor_id, name)
        raise RuntimeError(name + " hardware error")

    if torque != TORQUE_ON:
        print("")
        print("[FATAL]", name, "torque is OFF while running.")
        print("[FATAL] Hardware error is 0, but motor is no longer active.")
        print_full_status(motor_id, name)
        raise RuntimeError(name + " torque off")


def safe_stop():
    print("")
    print("[SAFE STOP] Sending stop commands...")

    try:
        write4(ID_THROTTLE, ADDR_GOAL_VELOCITY, 0, "safe stop throttle")
    except Exception as e:
        print("[SAFE STOP] throttle stop failed:", e)

    try:
        write4(ID_STEER, ADDR_GOAL_POSITION, STEER_CENTER_POS, "safe stop steer")
    except Exception as e:
        print("[SAFE STOP] steer center failed:", e)

    time.sleep(0.3)

    try:
        write1(ID_THROTTLE, ADDR_TORQUE_ENABLE, TORQUE_OFF, "safe torque off throttle")
    except Exception as e:
        print("[SAFE STOP] throttle torque off failed:", e)

    try:
        write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_OFF, "safe torque off steer")
    except Exception as e:
        print("[SAFE STOP] steer torque off failed:", e)

    print("[SAFE STOP] Done")


# =========================
# Main program
# =========================

def main():
    global last_vel
    global last_pos

    print("Opening port:", PORT)

    if not port.openPort():
        print("[FATAL] Failed to open port:", PORT)
        sys.exit(1)

    if not port.setBaudRate(BAUD):
        print("[FATAL] Failed to set baudrate:", BAUD)
        sys.exit(1)

    print("[OK] Port opened")

    # 1. Check hardware error before start.
    ok1 = startup_check_and_reboot_if_error(ID_THROTTLE, "THROTTLE")
    ok2 = startup_check_and_reboot_if_error(ID_STEER, "STEER")

    if not ok1 or not ok2:
        print("[FATAL] Startup check failed. Program will exit.")
        port.closePort()
        sys.exit(1)

    # 2. Re-initialize motors and torque on.
    configure_throttle_motor()
    configure_steer_motor()

    print_full_status(ID_THROTTLE, "THROTTLE")
    print_full_status(ID_STEER, "STEER")

    # 3. Initialize controller.
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("[FATAL] No joystick found.")
        safe_stop()
        port.closePort()
        pygame.quit()
        sys.exit(1)

    joy = pygame.joystick.Joystick(0)
    joy.init()

    print("[OK] Joystick ready:", joy.get_name())
    print("[RUN] RC control started")
    print("[RUN] If hardware error occurs, program will print error and exit.")

    last_check_time = 0.0
    last_vel = None
    last_pos = None

    try:
        while True:
            now = time.time()

            # Check motor status every CHECK_INTERVAL seconds.
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
                    "write throttle goal velocity"
                )
                last_vel = vel_val

            if pos_val != last_pos:
                write4(
                    ID_STEER,
                    ADDR_GOAL_POSITION,
                    pos_val,
                    "write steer goal position"
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