#!/usr/bin/env python3
"""
PS4 Controller Reader for Raspberry Pi

This program points pygame/SDL to the specific controller device:
    /dev/input/js0

It only keeps PS4 controller reading code.
No PCA9685 / 9685 motor-control code is included.
"""

import os
import time
import pygame


# ======================================================
# Force SDL / pygame to use the specific joystick device
# ======================================================
PS4_DEVICE = "/dev/input/js0"

if not os.path.exists(PS4_DEVICE):
    print("Controller device not found:", PS4_DEVICE)
    print("Please check if your controller is connected.")
    exit()

# Tell SDL to use this joystick device
os.environ["SDL_JOYSTICK_DEVICE"] = PS4_DEVICE


# ======================================================
# Initialize pygame joystick
# ======================================================
pygame.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()

if joystick_count == 0:
    print("No joystick detected by pygame.")
    print("Device exists, but pygame cannot access it:", PS4_DEVICE)
    print("Try running with sudo or check permission for /dev/input/js0.")
    exit()

# Usually /dev/input/js0 is joystick index 0
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("PS4 controller connected.")
print("Device:", PS4_DEVICE)
print("Pygame joystick name:", joystick.get_name())
print("Number of axes:", joystick.get_numaxes())
print("Number of buttons:", joystick.get_numbuttons())


# ======================================================
# Main loop
# ======================================================
while True:
    pygame.event.pump()

    axis_data = {}
    button_data = {}

    for i in range(joystick.get_numaxes()):
        axis_data[f"axis{i}"] = round(joystick.get_axis(i), 2)

    for i in range(joystick.get_numbuttons()):
        button_data[f"button{i}"] = joystick.get_button(i)

    print("Axes:", axis_data, "Buttons:", button_data)

    time.sleep(0.05)
