#!/usr/bin/env python3
"""
PS4 Controller Reader
Only keeps PS4 / joystick related code.
Removed PCA9685 / 9685 motor-control related code.
"""

import time
import pygame


# =============================
# Initialize PS4 controller
# =============================
pygame.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()

if joystick_count == 0:
    print("No joystick connected")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Joystick connected:", joystick.get_name())


# =============================
# Main loop
# =============================
while True:
    pygame.event.pump()

    axis0 = round(joystick.get_axis(0), 2)
    axis1 = round(joystick.get_axis(1), 2)
    axis2 = round(joystick.get_axis(2), 2)
    axis3 = round(joystick.get_axis(3), 2)
    axis4 = round(joystick.get_axis(4), 2)
    axis5 = round(joystick.get_axis(5), 2)

    button0 = joystick.get_button(0)
    button1 = joystick.get_button(1)
    button2 = joystick.get_button(2)
    button3 = joystick.get_button(3)

    print(
        "axis0:", axis0,
        "axis1:", axis1,
        "axis2:", axis2,
        "axis3:", axis3,
        "axis4:", axis4,
        "axis5:", axis5,
        "button0:", button0,
        "button1:", button1,
        "button2:", button2,
        "button3:", button3
    )

    time.sleep(0.05)
