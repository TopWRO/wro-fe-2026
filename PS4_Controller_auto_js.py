#!/usr/bin/env python3
"""
PS4 Controller Reader for Raspberry Pi

Auto-detects whether controller is:
    /dev/input/js0 OR /dev/input/js1

Then binds pygame to that device.
"""

import os
import time
import pygame


# ======================================================
# Detect controller device
# ======================================================
POSSIBLE_DEVICES = ["/dev/input/js0", "/dev/input/js1"]
PS4_DEVICE = None

for dev in POSSIBLE_DEVICES:
    if os.path.exists(dev):
        PS4_DEVICE = dev
        break

if PS4_DEVICE is None:
    print("No controller device found (js0/js1).")
    exit()

print("Detected controller device:", PS4_DEVICE)

# Force SDL to use detected device
os.environ["SDL_JOYSTICK_DEVICE"] = PS4_DEVICE


# ======================================================
# Initialize pygame joystick
# ======================================================
pygame.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()

if joystick_count == 0:
    print("Device exists but pygame cannot access it:", PS4_DEVICE)
    print("Try: sudo chmod 666 /dev/input/js*")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("PS4 controller connected")
print("Using device:", PS4_DEVICE)
print("Name:", joystick.get_name())


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
