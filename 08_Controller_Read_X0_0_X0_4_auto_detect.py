#!/usr/bin/env python3
"""
Raspberry Pi controller data reader.

This version removes all PCA9685 / PWM output code.
It only reads and displays Controller X0-0 and Controller X0-4.

Improvements:
1. Uses joystick index 0 directly, matching the name X0-0 / X0-4.
2. Re-scans the controller if no joystick is found at program start.
3. Prints all available axis values so you can confirm which physical stick maps to which axis.
4. Uses pygame.event.pump() every loop so axis values refresh continuously.
5. Does not require any PCA9685, servo, or PWM hardware.
"""

import os
import sys
import time
import pygame

# If running without Raspberry Pi desktop GUI / over SSH, this helps pygame initialize.
# It does not connect Bluetooth by itself; the controller still must be paired/connected in Raspberry Pi OS.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")


def init_pygame_joystick():
    """Initialize pygame joystick system and return joystick 0 when available."""
    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        return None, 0

    print("\nJoystick/controller detected.")
    print("Total joystick count:", joystick_count)

    for i in range(joystick_count):
        js = pygame.joystick.Joystick(i)
        js.init()
        print(
            "Joystick index X{}: {} | axes: {} | buttons: {} | hats: {}".format(
                i,
                js.get_name(),
                js.get_numaxes(),
                js.get_numbuttons(),
                js.get_numhats(),
            )
        )

    # Use joystick index 0, because the requested data is X0-0 and X0-4.
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    return joystick, joystick_count


def read_axis_safe(joystick, axis_index):
    """Read one axis safely. Return 0.0 if that axis does not exist."""
    if joystick is None:
        return 0.0
    if joystick.get_numaxes() <= axis_index:
        return 0.0
    return round(joystick.get_axis(axis_index), 2)


if __name__ == "__main__":
    print("Starting controller reader...")
    print("Please make sure the Bluetooth/USB controller is connected before reading data.")
    print("This program will keep scanning until it finds a controller. Press Ctrl+C to stop.\n")

    joystick = None

    try:
        while joystick is None:
            pygame.joystick.quit()
            pygame.joystick.init()
            joystick, count = init_pygame_joystick()
            if joystick is None:
                print("No joystick/controller found. Waiting for connection...")
                time.sleep(1)

        print("\nNow reading Controller X0-0 and Controller X0-4.")
        print("Also showing all axis values for mapping check.")
        print("Move the left stick and right stick to see which axis changes.")
        print("Press Ctrl+C to stop.\n")

        while True:
            pygame.event.pump()

            axis_count = joystick.get_numaxes()
            all_axes = []
            for i in range(axis_count):
                all_axes.append("X0-{}={}".format(i, round(joystick.get_axis(i), 2)))

            x0_0 = read_axis_safe(joystick, 0)
            x0_4 = read_axis_safe(joystick, 4)

            print(
                "Controller X0-0 = {}     Controller X0-4 = {}     All axes: {}".format(
                    x0_0,
                    x0_4,
                    " | ".join(all_axes),
                )
            )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        pygame.quit()
        sys.exit(0)
