#!/usr/bin/env python3
"""
Two-Sensor Parallel Wall Following RC Car - Raspberry Pi + VL53L4CD

Hardware idea:
- Put one VL53L4CD on the RIGHT FRONT side of the car.
- Put one VL53L4CD on the RIGHT REAR side of the car.
- Both sensors face to the RIGHT wall.
- The two sensors are 10 cm apart along the car body.

Control goal:
- Keep average distance from the right wall at 30 cm.
- Keep the car body parallel to the wall by making front and rear readings equal.

Install:
    pip3 install adafruit-blinka
    pip3 install adafruit-circuitpython-vl53l4cd

Run:
    python3 two_sensor_parallel_wall_following_car.py

Important:
- set_motor_speed() currently prints values only.
- Replace it with your real motor driver code from your car.
"""

import time
import math
import board
import digitalio
import adafruit_vl53l4cd


# ======================================================
# User settings
# ======================================================

TARGET_DISTANCE_CM = 30.0
SENSOR_GAP_CM = 10.0          # distance between right-front and right-rear sensors
DEAD_ZONE_CM = 1.5            # ignore small distance error
ANGLE_DEAD_ZONE_CM = 0.7      # ignore small front-rear difference

BASE_SPEED = 4.0              # normal forward speed, range -10 to +10
MAX_SPEED = 10.0
MAX_STEER = 4.0               # limit steering correction
LOOP_DELAY = 0.05

# Tuning values:
# If car corrects too slowly, increase these.
# If car shakes left-right, decrease these.
DISTANCE_KP = 0.10            # controls 30 cm distance
ANGLE_KP = 0.45               # controls parallel angle

# Optional smoothing. 0.0 = no update, 1.0 = no smoothing.
SMOOTHING_ALPHA = 0.45


# ======================================================
# VL53L4CD address settings
# ======================================================

# Change these pins if your XSHUT wiring is different.
SENSOR_CONFIG = {
    "right_front": {
        "xshut_pin": board.D5,
        "new_address": 0x30,
    },
    "right_rear": {
        "xshut_pin": board.D6,
        "new_address": 0x31,
    },
}


# ======================================================
# Helper functions
# ======================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def setup_xshut_pins():
    """Turn all VL53L4CD sensors off first."""
    xshut_pins = {}

    for name, config in SENSOR_CONFIG.items():
        pin = digitalio.DigitalInOut(config["xshut_pin"])
        pin.direction = digitalio.Direction.OUTPUT
        pin.value = False
        xshut_pins[name] = pin

    time.sleep(0.2)
    return xshut_pins


def setup_vl53l4cd_sensors(i2c):
    """Turn on sensors one by one and give each a unique I2C address."""
    sensors = {}
    xshut_pins = setup_xshut_pins()

    for name, config in SENSOR_CONFIG.items():
        print("Starting", name, "sensor...")

        xshut_pins[name].value = True
        time.sleep(0.2)

        sensor = adafruit_vl53l4cd.VL53L4CD(i2c)
        sensor.set_address(config["new_address"])
        time.sleep(0.1)

        sensor.inter_measurement = 0
        sensor.timing_budget = 50
        sensor.start_ranging()

        sensors[name] = sensor
        print(name, "ready at I2C address", hex(config["new_address"]))

    return sensors


def read_distance_cm(sensor):
    """Read one VL53L4CD sensor. Adafruit library returns cm."""
    if sensor.data_ready:
        sensor.clear_interrupt()
        return float(sensor.distance)
    return None


def smooth_value(old_value, new_value):
    """Simple low-pass filter to reduce noisy steering."""
    if new_value is None:
        return old_value
    if old_value is None:
        return new_value
    return SMOOTHING_ALPHA * new_value + (1.0 - SMOOTHING_ALPHA) * old_value


def read_all_distances(sensors, last_values):
    distances = {}

    for name, sensor in sensors.items():
        raw = read_distance_cm(sensor)
        last_values[name] = smooth_value(last_values[name], raw)
        distances[name] = last_values[name]

    return distances


# ======================================================
# Motor control
# ======================================================

def set_motor_speed(left_speed, right_speed):
    """
    Replace this function with your real motor driver code.

    left_speed and right_speed:
        -10 = full reverse
         0  = stop
        +10 = full forward

    Positive steering in this program means turning RIGHT:
        left motor faster, right motor slower.
    """
    left_speed = clamp(left_speed, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, -MAX_SPEED, MAX_SPEED)

    print("LEFT:", round(left_speed, 2), "RIGHT:", round(right_speed, 2))


def stop_car():
    set_motor_speed(0, 0)


# ======================================================
# Wall following logic
# ======================================================

def calculate_motor_speed(distances):
    """
    Uses two right-side sensors:

    average_distance = (right_front + right_rear) / 2
    distance_error = average_distance - 30

    angle_error_cm = right_front - right_rear
    angle_rad = atan(angle_error_cm / 10)

    For right wall:
    - distance_error > 0: car is too far from wall -> turn right.
    - distance_error < 0: car is too close to wall -> turn left.
    - angle_error_cm > 0: front is farther than rear -> nose points left -> turn right.
    - angle_error_cm < 0: front is closer than rear -> nose points right -> turn left.
    """
    right_front = distances["right_front"]
    right_rear = distances["right_rear"]

    if right_front is None or right_rear is None:
        return BASE_SPEED, BASE_SPEED, "Waiting for both sensors"

    average_distance = (right_front + right_rear) / 2.0
    distance_error = average_distance - TARGET_DISTANCE_CM
    angle_error_cm = right_front - right_rear
    angle_deg = math.degrees(math.atan2(angle_error_cm, SENSOR_GAP_CM))

    if abs(distance_error) <= DEAD_ZONE_CM:
        distance_error = 0.0

    if abs(angle_error_cm) <= ANGLE_DEAD_ZONE_CM:
        angle_error_cm = 0.0

    steer = DISTANCE_KP * distance_error + ANGLE_KP * angle_error_cm
    steer = clamp(steer, -MAX_STEER, MAX_STEER)

    # Differential drive:
    # positive steer = right turn = left motor faster, right motor slower.
    left_speed = BASE_SPEED + steer
    right_speed = BASE_SPEED - steer

    left_speed = clamp(left_speed, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, -MAX_SPEED, MAX_SPEED)

    status = (
        "RF=" + str(round(right_front, 1)) + "cm " +
        "RR=" + str(round(right_rear, 1)) + "cm " +
        "avg=" + str(round(average_distance, 1)) + "cm " +
        "dist_err=" + str(round(distance_error, 1)) + "cm " +
        "angle=" + str(round(angle_deg, 1)) + "deg " +
        "steer=" + str(round(steer, 2))
    )

    return left_speed, right_speed, status


# ======================================================
# Main program
# ======================================================

def main():
    print("Initializing I2C...")
    i2c = board.I2C()

    print("Initializing right-front and right-rear VL53L4CD sensors...")
    sensors = setup_vl53l4cd_sensors(i2c)

    last_values = {
        "right_front": None,
        "right_rear": None,
    }

    print("Sensors ready.")
    print("Target: drive parallel to RIGHT wall at", TARGET_DISTANCE_CM, "cm.")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            distances = read_all_distances(sensors, last_values)
            left_speed, right_speed, status = calculate_motor_speed(distances)

            set_motor_speed(left_speed, right_speed)
            print(status)

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("Stopping car...")
        stop_car()

        for sensor in sensors.values():
            sensor.stop_ranging()

        print("Program stopped.")


if __name__ == "__main__":
    main()
