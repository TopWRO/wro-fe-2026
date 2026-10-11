#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PS4 Controller /dev/input/js0 Pygame Test Program

Purpose:
- Force pygame/SDL to use /dev/input/js0
- Check whether pygame can detect the PS4 controller
- Print all joystick axis data
- Print Controller X0-0 and Controller X0-4 clearly

Run:
    python3 test_ps4_js0_pygame.py

If permission problem:
    sudo python3 test_ps4_js0_pygame.py
"""

import os
import time
import pygame

# ============================================================
# Force pygame / SDL to use /dev/input/js0
# ============================================================

JS_DEVICE = "/dev/input/js0"
os.environ["SDL_JOYSTICK_DEVICE"] = JS_DEVICE

print("======================================")
print(" PS4 Controller /dev/input/js0 Test")
print("======================================")
print("Target joystick device:", JS_DEVICE)

# ============================================================
# Check device existence and permission
# ============================================================

if not os.path.exists(JS_DEVICE):
    print("ERROR: /dev/input/js0 does not exist.")
    print("Please check with:")
    print("  ls /dev/input/js*")
    raise SystemExit(1)

print("OK: /dev/input/js0 exists.")

if not os.access(JS_DEVICE, os.R_OK):
    print("WARNING: Current user may not have read permission for /dev/input/js0.")
    print("Try running:")
    print("  sudo python3 test_ps4_js0_pygame.py")
    print("Or add your user to input group:")
    print("  sudo usermod -a -G input $USER")
    print("Then reboot Raspberry Pi.")
else:
    print("OK: /dev/input/js0 is readable.")

# ============================================================
# Initialize pygame joystick
# ============================================================

pygame.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()
print("--------------------------------------")
print("Pygame joystick count:", joystick_count)

if joystick_count == 0:
    print("ERROR: Pygame did not detect any joystick.")
    print("Even though /dev/input/js0 exists, SDL/Pygame did not connect to it.")
    print()
    print("Please test raw joystick device first:")
    print("  sudo apt install joystick")
    print("  jstest /dev/input/js0")
    pygame.quit()
    raise SystemExit(1)

# Use pygame joystick index 0
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Joystick connected in pygame.")
print("Joystick name:", joystick.get_name())
print("Joystick ID:", joystick.get_id())
print("Number of axes:", joystick.get_numaxes())
print("Number of buttons:", joystick.get_numbuttons())
print("Number of hats:", joystick.get_numhats())
print("--------------------------------------")
print("Move the PS4 controller sticks now.")
print("The program will print all axis values.")
print("Press Ctrl + C to stop.")
print("--------------------------------------")

# ============================================================
# Main loop
# ============================================================

try:
    while True:
        # Important: refresh joystick state
        pygame.event.pump()

        num_axes = joystick.get_numaxes()
        axis_values = []

        for i in range(num_axes):
            value = joystick.get_axis(i)
            axis_values.append(f"X0-{i}={value:.3f}")

        print("All axes:", " | ".join(axis_values))

        # Clearly print X0-0 and X0-4
        x0_0 = joystick.get_axis(0) if num_axes > 0 else None
        x0_4 = joystick.get_axis(4) if num_axes > 4 else None

        print(
            "Controller X0-0 =",
            f"{x0_0:.3f}" if x0_0 is not None else "N/A",
            "     Controller X0-4 =",
            f"{x0_4:.3f}" if x0_4 is not None else "N/A",
        )

        print("--------------------------------------")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nProgram stopped by user.")

finally:
    pygame.quit()
