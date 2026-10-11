#!/usr/bin/env python3
"""
PS4 Controller Reader for Raspberry Pi

Auto-detects /dev/input/js0 or js1
Prints only axis0 and axis4
"""

import os
import time
import pygame


# Detect controller device
POSSIBLE_DEVICES = ["/dev/input/js0", "/dev/input/js1"]
PS4_DEVICE = None

for dev in POSSIBLE_DEVICES:
    if os.path.exists(dev):
        PS4_DEVICE = dev
        break

if PS4_DEVICE is None:
    print("No controller device found (js0/js1).")
    exit()

print("Using device:", PS4_DEVICE)

os.environ["SDL_JOYSTICK_DEVICE"] = PS4_DEVICE


# Initialize pygame
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Device exists but pygame cannot access it.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()


# Main loop
while True:
    pygame.event.pump()

    axis0 = round(joystick.get_axis(0), 2)
    axis4 = round(joystick.get_axis(4), 2)

    print("axis0:", axis0, " axis4:", axis4)

    time.sleep(0.05)
