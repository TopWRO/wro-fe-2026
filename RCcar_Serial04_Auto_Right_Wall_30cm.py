#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RCcar_Serial04_Auto_Right_Wall_30cm.py

Purpose:
- Use the user's original Dynamixel + PS4 + Serial04 structure.
- Read unified serial data from /dev/ttyACM0 at 115200 baud.
- Expected serial line format:
      angle, front, right, back, left
- Use two side distance values to keep the car parallel to the right wall.
- Keep the car about 30 cm from the wall.
- PS4 axis 4 controls throttle.
- Steering is controlled automatically by the wall-following algorithm.
- Print live debug data: two sensor values, average distance, errors, steering, motor commands.

IMPORTANT SENSOR MAPPING:
- By default this code uses:
      right_front = right
      right_rear  = back
- If your serial names are different, change RIGHT_FRONT_SOURCE and RIGHT_REAR_SOURCE below.

Run:
    python3 RCcar_Serial04_Auto_Right_Wall_30cm.py

Stop:
    Ctrl+C
"""

import sys
import time
import math
import serial
import pygame
from dynamixel_sdk import *


# ============================================================
# Dynamixel RC car control settings
# ============================================================

PORT = "/dev/ttyUSB0"
BAUD = 57600

ID_THROTTLE = 1
ID_STEER = 2

ADDR_MODE = 11
ADDR_TORQUE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112

VELOCITY_MODE = 1
POSITION_MODE = 3
TORQUE_ON = 1
TORQUE_OFF = 0

TARGET_RPM = 100
VELOCITY_UNIT_RPM = 0.229
MAX_STEER_DEG = 50

# PS4 throttle settings
DEADZONE = 0.18
STEP = 0.10

LOOP_DELAY = 0.05


# ============================================================
# Serial04 settings
# ============================================================

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200


# ============================================================
# Wall following settings
# ============================================================

TARGET_DISTANCE_CM = 30.0
SENSOR_SPACING_CM = 10.0
DISTANCE_DEAD_ZONE_CM = 1.5

# Serial fields are: angle, front, right, back, left
# Change these two if your two right-side sensors use different names.
RIGHT_FRONT_SOURCE = "right"
RIGHT_REAR_SOURCE = "back"

# Steering tuning.
# If car turns too weakly, increase these.
# If car wiggles, decrease these.
DISTANCE_KP = 0.035
ANGLE_DIFF_KP = 0.080

# Limit final auto steering command before converting to steering servo.
# q_steer range is -1.0 to +1.0.
MAX_AUTO_STEER = 0.85

# Safety stop if the front sensor sees an obstacle.
FRONT_STOP_CM = 18.0
ENABLE_FRONT_SAFETY_STOP = True

# If True, steering direction is reversed.
# Use this if the car corrects in the wrong direction.
REVERSE_STEERING = False

# If True, throttle direction is reversed.
# Use this if pushing PS4 throttle forward makes the car go backward.
REVERSE_THROTTLE = False

# Keep steering centered until valid serial data arrives.
ALLOW_DRIVE_WITHOUT_SENSOR = False


# ============================================================
# Helper functions
# ============================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def to_uint32(value):
    value = int(value)
    if value < 0:
        value = value + 4294967296
    return value


def deg_to_unit(deg):
    return int(deg * 4096 / 360)


def quantize(val):
    if abs(val) < DEADZONE:
        return 0.0

    sign = 1 if val >= 0 else -1
    q = round(abs(val) / STEP) * STEP
    return sign * max(0.0, min(1.0, q))


def parse_serial04_line(line):
    """
    Expected serial line:
        angle, front, right, back, left

    Return a dict or None.
    """
    line = line.strip()

    if not line:
        return None

    if line.startswith("#"):
        return None

    parts = line.split(",")

    if len(parts) != 5:
        return None

    try:
        angle, front, right, back, left = map(float, parts)
        return {
            "angle": angle,
            "front": front,
            "right": right,
            "back": back,
            "left": left,
        }
    except ValueError:
        return None


def get_sensor_value(sensor_data, source_name):
    if sensor_data is None:
        return None
    return sensor_data.get(source_name)


def calculate_wall_steering(sensor_data):
    """
    Uses two right-side sensors.

    distance_error:
        positive = car is too far from wall
        negative = car is too close to wall

    angle_diff = right_front - right_rear:
        positive = front is farther from wall than rear
        negative = front is closer to wall than rear

    For right wall following:
        positive steering means turn right.
        negative steering means turn left.
    """
    if sensor_data is None:
        return 0.0, {
            "status": "NO_SENSOR_DATA",
            "rf": None,
            "rr": None,
            "avg": None,
            "distance_error": None,
            "angle_diff": None,
            "angle_deg": None,
            "distance_steer": 0.0,
            "angle_steer": 0.0,
        }

    rf = get_sensor_value(sensor_data, RIGHT_FRONT_SOURCE)
    rr = get_sensor_value(sensor_data, RIGHT_REAR_SOURCE)
    front = get_sensor_value(sensor_data, "front")

    if rf is None or rr is None:
        return 0.0, {
            "status": "MISSING_RIGHT_SENSOR",
            "rf": rf,
            "rr": rr,
            "avg": None,
            "distance_error": None,
            "angle_diff": None,
            "angle_deg": None,
            "distance_steer": 0.0,
            "angle_steer": 0.0,
        }

    if ENABLE_FRONT_SAFETY_STOP and front is not None and front < FRONT_STOP_CM:
        # Steering can stay centered; throttle will be forced to zero in main loop.
        status = "FRONT_STOP"
    else:
        status = "AUTO_WALL"

    avg_distance = (rf + rr) / 2.0
    distance_error = avg_distance - TARGET_DISTANCE_CM

    if abs(distance_error) <= DISTANCE_DEAD_ZONE_CM:
        distance_error_for_control = 0.0
    else:
        distance_error_for_control = distance_error

    angle_diff = rf - rr
    angle_deg = math.degrees(math.atan2(angle_diff, SENSOR_SPACING_CM))

    distance_steer = DISTANCE_KP * distance_error_for_control
    angle_steer = ANGLE_DIFF_KP * angle_diff

    steering = distance_steer + angle_steer

    if REVERSE_STEERING:
        steering = -steering

    steering = clamp(steering, -MAX_AUTO_STEER, MAX_AUTO_STEER)

    debug = {
        "status": status,
        "rf": rf,
        "rr": rr,
        "avg": avg_distance,
        "distance_error": distance_error,
        "angle_diff": angle_diff,
        "angle_deg": angle_deg,
        "distance_steer": distance_steer,
        "angle_steer": angle_steer,
    }

    return steering, debug


def format_float(value, width=6, decimals=1):
    if value is None:
        return " " * (width - 4) + "None"
    return f"{value:{width}.{decimals}f}"


# ============================================================
# Setup Dynamixel
# ============================================================

port = PortHandler(PORT)
packet = PacketHandler(2.0)

if not port.openPort() or not port.setBaudRate(BAUD):
    print("[DXL] port open or baudrate failed")
    sys.exit(1)

# Throttle motor setup
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_MODE, VELOCITY_MODE)
packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_PROFILE_ACCEL, 100)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_ON)

# Steering motor setup
packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, ID_STEER, ADDR_MODE, POSITION_MODE)
packet.write4ByteTxRx(port, ID_STEER, ADDR_PROFILE_ACCEL, 100)
packet.write4ByteTxRx(port, ID_STEER, ADDR_PROFILE_VELOCITY, 500)
packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_ON)

# Keep original fixed center value.
zero_pos = 2047


# ============================================================
# Setup pygame joystick
# ============================================================

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("[JOY] no joystick detected")
    sys.exit(1)

joy = pygame.joystick.Joystick(0)
joy.init()

print("[JOY] connected:", joy.get_name())
print("[JOY] axis 4 = throttle")
print("[AUTO] steering is controlled by right wall following")


# ============================================================
# Setup Serial04
# ============================================================

try:
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)
    print("[SERIAL04] Opened", ser.name)

    serial_buffer = ""
    serial_timer = time.time()

    ser.write(b"reset gyro\n")
    print("[SERIAL04] reset gyro sent")

except Exception as e:
    print("[SERIAL04] open failed:", e)
    ser = None
    serial_buffer = ""
    serial_timer = time.time()


# ============================================================
# Main loop
# ============================================================

last_vel = None
last_pos = None
latest_sensor_data = None
latest_raw_sensor_text = "sensor: no data yet"
latest_debug = None

if __name__ == "__main__":
    try:
        while True:
            pygame.event.pump()

            # ------------------------------------------------
            # Read PS4 throttle only
            # ------------------------------------------------
            raw_throttle = joy.get_axis(4)
            if REVERSE_THROTTLE:
                raw_throttle = -raw_throttle
            q_throttle = quantize(raw_throttle)

            # ------------------------------------------------
            # Serial04 non-blocking receive
            # ------------------------------------------------
            if ser is not None:
                n = ser.in_waiting

                if n:
                    chunk = ser.read(n).decode("utf-8", errors="ignore")
                    if chunk:
                        serial_buffer += chunk
                        *lines, serial_buffer = serial_buffer.split("\n")

                        for line in lines:
                            data = parse_serial04_line(line)
                            if data is None:
                                continue

                            latest_sensor_data = data
                            latest_raw_sensor_text = (
                                f"A:{data['angle']:6.2f} "
                                f"F:{data['front']:6.1f} "
                                f"R:{data['right']:6.1f} "
                                f"B:{data['back']:6.1f} "
                                f"L:{data['left']:6.1f}"
                            )

                if time.time() > (serial_timer + 10):
                    serial_timer = time.time()

            # ------------------------------------------------
            # Calculate auto steering from two side sensors
            # ------------------------------------------------
            auto_steer, debug = calculate_wall_steering(latest_sensor_data)
            latest_debug = debug

            q_steer = auto_steer

            # Safety behavior
            if debug["status"] == "FRONT_STOP":
                q_throttle = 0.0
            elif debug["status"] != "AUTO_WALL" and not ALLOW_DRIVE_WITHOUT_SENSOR:
                q_throttle = 0.0
                q_steer = 0.0

            # ------------------------------------------------
            # Send Dynamixel commands
            # ------------------------------------------------
            target_rpm = q_throttle * TARGET_RPM
            vel_val = int(target_rpm / VELOCITY_UNIT_RPM)

            steer_deg = q_steer * MAX_STEER_DEG
            pos_val = int(zero_pos + deg_to_unit(steer_deg))

            if vel_val != last_vel:
                packet.write4ByteTxRx(
                    port,
                    ID_THROTTLE,
                    ADDR_GOAL_VELOCITY,
                    to_uint32(vel_val)
                )
                last_vel = vel_val

            if pos_val != last_pos:
                packet.write4ByteTxRx(
                    port,
                    ID_STEER,
                    ADDR_GOAL_POSITION,
                    pos_val
                )
                last_pos = pos_val

            # ------------------------------------------------
            # Real-time debug display
            # ------------------------------------------------
            debug_text = (
                f"{debug['status']:12s} | "
                f"RF:{format_float(debug['rf'])} "
                f"RR:{format_float(debug['rr'])} "
                f"AVG:{format_float(debug['avg'])} "
                f"D_ERR:{format_float(debug['distance_error'], 7, 2)} "
                f"DIFF:{format_float(debug['angle_diff'], 7, 2)} "
                f"ANG:{format_float(debug['angle_deg'], 7, 2)} "
                f"D_STR:{debug['distance_steer']:+6.3f} "
                f"A_STR:{debug['angle_steer']:+6.3f} "
                f"STEER:{q_steer:+5.2f} "
                f"STEER_DEG:{steer_deg:+6.1f} "
                f"THR:{q_throttle*100:+4.0f}% "
                f"VEL:{vel_val:+5d} "
                f"POS:{pos_val:4d}"
            )

            print(debug_text + " | " + latest_raw_sensor_text, end="\r")

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\n[PY] end")

        packet.write4ByteTxRx(
            port,
            ID_THROTTLE,
            ADDR_GOAL_VELOCITY,
            0
        )

        packet.write4ByteTxRx(
            port,
            ID_STEER,
            ADDR_GOAL_POSITION,
            zero_pos
        )

        time.sleep(0.5)

        packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
        packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)

        port.closePort()

        if ser is not None:
            ser.close()

        pygame.quit()
        print("[PY] stopped safely")
