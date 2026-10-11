#!/usr/bin/env python3
"""
VL53L4CD Wall Following RC Car - Raspberry Pi

Function:
- Uses 4 VL53L4CD distance sensors: front, back, left, right.
- Automatically assigns different I2C addresses using XSHUT pins.
- Keeps the car about 30 cm away from a wall.
- If the car is too close to the wall, it steers away.
- If the car is too far from the wall, it steers toward the wall.
- Uses differential drive output: left motor speed and right motor speed.

Install libraries:
    pip3 install adafruit-blinka
    pip3 install adafruit-circuitpython-vl53l4cd

Wiring:
- All VL53L4CD sensors share:
    VIN -> 3.3V or 5V
    GND -> GND
    SDA -> Raspberry Pi SDA
    SCL -> Raspberry Pi SCL

- Each sensor must have its XSHUT / SHDN pin connected to a different GPIO:
    FRONT sensor XSHUT -> GPIO5
    BACK  sensor XSHUT -> GPIO6
    LEFT  sensor XSHUT -> GPIO13
    RIGHT sensor XSHUT -> GPIO19

You may change these pins below if your wiring is different.

Important:
- The motor control function currently prints the motor values.
- Replace set_motor_speed() with your real motor driver code.
"""

import time
import board
import digitalio
import adafruit_vl53l4cd


# ======================================================
# User settings
# ======================================================

TARGET_DISTANCE_CM = 30.0
DEAD_ZONE_CM = 2.0

# Choose wall to follow: "left" or "right"
FOLLOW_WALL = "right"

# Speed range is -10 to +10
BASE_SPEED = 4.0
MAX_SPEED = 10.0

# Steering strength.
# Increase if correction is too weak.
# Decrease if the car wiggles too much.
DISTANCE_KP = 0.25

# Front safety stop distance
FRONT_STOP_CM = 15.0

# Loop delay
LOOP_DELAY = 0.05


# ======================================================
# VL53L4CD address settings
# ======================================================

DEFAULT_ADDRESS = 0x29

SENSOR_CONFIG = {
    "front": {
        "xshut_pin": board.D5,
        "new_address": 0x30,
    },
    "back": {
        "xshut_pin": board.D6,
        "new_address": 0x31,
    },
    "left": {
        "xshut_pin": board.D13,
        "new_address": 0x32,
    },
    "right": {
        "xshut_pin": board.D19,
        "new_address": 0x33,
    },
}


# ======================================================
# Helper functions
# ======================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def setup_xshut_pins():
    """
    Create XSHUT output pins and turn all sensors off first.
    """
    xshut_pins = {}

    for name, config in SENSOR_CONFIG.items():
        pin = digitalio.DigitalInOut(config["xshut_pin"])
        pin.direction = digitalio.Direction.OUTPUT
        pin.value = False
        xshut_pins[name] = pin

    time.sleep(0.2)
    return xshut_pins


def setup_vl53l4cd_sensors(i2c):
    """
    Turn on each VL53L4CD one at a time and assign a unique I2C address.
    """
    sensors = {}
    xshut_pins = setup_xshut_pins()

    for name, config in SENSOR_CONFIG.items():
        print("Starting", name, "sensor...")

        # Turn on only this sensor.
        xshut_pins[name].value = True
        time.sleep(0.2)

        sensor = adafruit_vl53l4cd.VL53L4CD(i2c)

        # Give this sensor a unique address.
        sensor.set_address(config["new_address"])
        time.sleep(0.1)

        # Optional measurement settings.
        sensor.inter_measurement = 0
        sensor.timing_budget = 50

        sensor.start_ranging()

        sensors[name] = sensor

        print(
            name,
            "sensor ready at I2C address",
            hex(config["new_address"])
        )

    return sensors


def read_distance_cm(sensor):
    """
    Read one VL53L4CD sensor.
    The Adafruit library reports distance in centimeters.
    """
    if sensor.data_ready:
        sensor.clear_interrupt()
        return float(sensor.distance)

    return None


def read_all_distances(sensors, last_values):
    """
    Read all 4 sensors.
    If a sensor has no new data in this loop, keep the last valid value.
    """
    distances = {}

    for name, sensor in sensors.items():
        value = read_distance_cm(sensor)

        if value is not None:
            last_values[name] = value

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

    For now, the code prints motor speed only.
    """

    left_speed = clamp(left_speed, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, -MAX_SPEED, MAX_SPEED)

    print(
        "LEFT:",
        round(left_speed, 2),
        "RIGHT:",
        round(right_speed, 2)
    )


def stop_car():
    set_motor_speed(0, 0)


# ======================================================
# Wall following logic
# ======================================================

def calculate_motor_speed(distances):
    """
    Calculate left and right motor speed based on wall distance.

    If following right wall:
        distance < 30 cm means too close -> steer left
        distance > 30 cm means too far   -> steer right

    If following left wall:
        distance < 30 cm means too close -> steer right
        distance > 30 cm means too far   -> steer left
    """

    front = distances["front"]
    side = distances[FOLLOW_WALL]

    # Safety stop
    if front is not None and front < FRONT_STOP_CM:
        return 0, 0, "STOP: front obstacle"

    if side is None:
        return BASE_SPEED, BASE_SPEED, "No side distance"

    error = side - TARGET_DISTANCE_CM

    # Dead zone
    if abs(error) <= DEAD_ZONE_CM:
        error = 0

    steer = DISTANCE_KP * error

    # For right wall:
    # error > 0 means too far, steer right.
    # In differential drive, positive steer means right turn:
    # left motor faster, right motor slower.
    if FOLLOW_WALL == "right":
        left_speed = BASE_SPEED + steer
        right_speed = BASE_SPEED - steer

    # For left wall, reverse correction.
    else:
        left_speed = BASE_SPEED - steer
        right_speed = BASE_SPEED + steer

    left_speed = clamp(left_speed, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, -MAX_SPEED, MAX_SPEED)

    status = (
        "side=" + str(round(side, 1)) +
        "cm error=" + str(round(error, 1)) +
        " steer=" + str(round(steer, 2))
    )

    return left_speed, right_speed, status


# ======================================================
# Main program
# ======================================================

def main():
    print("Initializing I2C...")
    i2c = board.I2C()

    print("Initializing VL53L4CD sensors...")
    sensors = setup_vl53l4cd_sensors(i2c)

    last_values = {
        "front": None,
        "back": None,
        "left": None,
        "right": None,
    }

    print("All sensors ready.")
    print("Following", FOLLOW_WALL, "wall at", TARGET_DISTANCE_CM, "cm.")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            distances = read_all_distances(sensors, last_values)

            left_speed, right_speed, status = calculate_motor_speed(distances)

            set_motor_speed(left_speed, right_speed)

            print(
                "F:", distances["front"],
                "B:", distances["back"],
                "L:", distances["left"],
                "R:", distances["right"],
                "|",
                status
            )

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("Stopping car...")
        stop_car()

        for sensor in sensors.values():
            sensor.stop_ranging()

        print("Program stopped.")


if __name__ == "__main__":
    main()
