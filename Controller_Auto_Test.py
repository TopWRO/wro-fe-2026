#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
May PS4 Controller Auto Test

Purpose:
- Automatically try /dev/input/js0 and /dev/input/js1
- Use the first available joystick device detected by pygame
- Print controller axis data in one simple line
- No PCA9685, no PWM output

Run:
    python3 May_PS4_Controller_Auto_Test.py

If permission problem:
    sudo python3 May_PS4_Controller_Auto_Test.py
"""

import os
import time
import pygame

# ============================================================
# Candidate joystick devices
# ============================================================

CANDIDATE_DEVICES = [
    "/dev/input/js0",
    "/dev/input/js1",
]

PRINT_INTERVAL = 0.10   # seconds
DECIMAL_PLACES = 2     # reduce digital length


def find_available_js_device():
    """
    Find the first existing and readable joystick device.
    """
    for device in CANDIDATE_DEVICES:
        if os.path.exists(device):
            if os.access(device, os.R_OK):
                return device
            else:
                print(f"Found {device}, but no read permission.")
                print("You may need to run with sudo.")
                return device
    return None


def init_pygame_with_device(device):
    """
    Tell SDL/Pygame to use a specific joystick device.
    """
    os.environ["SDL_JOYSTICK_DEVICE"] = device

    pygame.quit()
    pygame.init()
    pygame.joystick.init()

    count = pygame.joystick.get_count()
    if count <= 0:
        return None

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    return joystick


def try_connect_controller():
    """
    Try js0 first, then js1.
    Return selected device and joystick object.
    """
    for device in CANDIDATE_DEVICES:
        if not os.path.exists(device):
            continue

        joystick = init_pygame_with_device(device)
        if joystick is not None:
            return device, joystick

    return None, None


def format_axis(value):
    """
    Format axis value with fewer digits.
    """
    return f"{value:.{DECIMAL_PLACES}f}"


def print_controller_data(device, joystick):
    """
    Print all axis values in one simple line.
    """
    pygame.event.pump()

    num_axes = joystick.get_numaxes()

    x0_0 = joystick.get_axis(0) if num_axes > 0 else 0
    x0_4 = joystick.get_axis(4) if num_axes > 4 else 0

    # Short one-line output
    print(
        f"{device} | "
        f"X0-0:{format_axis(x0_0)} | "
        f"X0-4:{format_axis(x0_4)}"
    )


def main():
    print("====================================")
    print(" May PS4 Controller Auto Test")
    print("====================================")
    print("Trying /dev/input/js0 first, then /dev/input/js1.")
    print("Press Ctrl + C to stop.")
    print("====================================")

    current_device = None
    joystick = None

    try:
        while True:
            # If no joystick connected, try to connect
            if joystick is None:
                current_device, joystick = try_connect_controller()

                if joystick is None:
                    print("No controller detected on js0 or js1. Waiting...")
                    time.sleep(1)
                    continue

                print(f"Connected: {current_device}")
                print(f"Name: {joystick.get_name()}")
                print(f"Axes: {joystick.get_numaxes()}")
                print("------------------------------------")

            try:
                print_controller_data(current_device, joystick)

            except pygame.error:
                # Controller disconnected or changed
                print("Controller disconnected. Trying js0/js1 again...")
                joystick = None
                current_device = None
                time.sleep(1)
                continue

            # If device disappears, reconnect automatically
            if current_device and not os.path.exists(current_device):
                print(f"{current_device} disappeared. Trying js0/js1 again...")
                joystick = None
                current_device = None
                time.sleep(1)
                continue

            time.sleep(PRINT_INTERVAL)

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")

    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
