#!/usr/bin/env python3
"""
Read PS/USB controller joystick data on Raspberry Pi.

This version removes all PCA9685 / PWM output code.
It only displays Controller X0-0 and X0-4 data.

Fix included:
- Uses pygame.event.pump() every loop so joystick axis values refresh continuously.
- Reads joystick data even when pygame.event.get() has no new events.
- Prints controller name and available axis count to help confirm axis mapping.
"""

import time
import sys
import pygame


# ======================================================
# Init joystick
# ======================================================
pygame.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()

if joystick_count == 0:
    print("No joystick/controller connected.")
    pygame.quit()
    sys.exit(1)

# Use the last connected joystick, same behavior as the old program.
joystick = pygame.joystick.Joystick(joystick_count - 1)
joystick.init()
time.sleep(0.2)

axis_count = joystick.get_numaxes()

print("Joystick connected:", joystick.get_name())
print("Joystick number:", joystick_count)
print("Available axis count:", axis_count)
print("Reading Controller X0-0 and X0-4 only. Press Ctrl+C to stop.")

joystick_data = {
    "axis0": 0.0,
    "axis4": 0.0,
}


# ======================================================
# Main loop
# ======================================================
if __name__ == "__main__":
    try:
        while True:
            # Very important: refresh pygame joystick state continuously.
            # Without this, some Raspberry Pi / controller combinations keep showing 0.
            pygame.event.pump()

            if axis_count > 0:
                joystick_data["axis0"] = round(joystick.get_axis(0), 2)
            else:
                joystick_data["axis0"] = 0.0

            if axis_count > 4:
                joystick_data["axis4"] = round(joystick.get_axis(4), 2)
            else:
                joystick_data["axis4"] = 0.0

            print(
                "Controller X0-0 =",
                joystick_data["axis0"],
                "                             ",
                "Controller X0-4 =",
                joystick_data["axis4"],
            )

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        pygame.quit()
