#!/usr/bin/env python3
"""
PS4 Controller Reader (Processed Output)

Features:
1. Auto-detect js0 / js1
2. Reverse axis direction (forward/back + left/right)
3. Quantize speed into 10 levels
4. Dead zone: [-0.1, 0.1] -> 0
5. Output only processed axis0 (left/right) and axis4 (forward/back)
"""

import os
import time
import pygame


# =============================
# Detect controller device
# =============================
POSSIBLE_DEVICES = ["/dev/input/js0", "/dev/input/js1"]
PS4_DEVICE = None

for dev in POSSIBLE_DEVICES:
    if os.path.exists(dev):
        PS4_DEVICE = dev
        break

if PS4_DEVICE is None:
    print("No controller device found.")
    exit()

os.environ["SDL_JOYSTICK_DEVICE"] = PS4_DEVICE


# =============================
# Initialize pygame
# =============================
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Controller not accessible.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Using:", PS4_DEVICE)


# =============================
# Helper: quantize function
# =============================
def quantize(val):
    # reverse direction
    val = -val

    # dead zone
    if -0.1 <= val <= 0.1:
        return 0

    # scale into 10 steps
    step = int(abs(val) * 10)

    # limit to max 10
    if step > 10:
        step = 10

    return step if val > 0 else -step


# =============================
# Main loop
# =============================
while True:
    pygame.event.pump()

    raw_axis0 = joystick.get_axis(0)
    raw_axis4 = joystick.get_axis(4)

    axis0 = quantize(raw_axis0)   # left/right
    axis4 = quantize(raw_axis4)   # forward/back

    print("axis0:", axis0, " axis4:", axis4)

    time.sleep(0.05)
