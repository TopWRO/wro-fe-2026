#!/usr/bin/env python3

import os
import sys
import time
import pygame
from dynamixel_sdk import *

PORT = "/dev/ttyUSB0"
BAUD = 57600
DXL_ID_STEER = 2

ADDR_MODE = 11
ADDR_TORQUE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112

TORQUE_OFF = 0
TORQUE_ON = 1
POSITION_MODE = 3

MAX_ANGLE = 50
PROFILE_ACCEL = 50
PROFILE_VELOCITY = 200

def degree_to_position_unit(deg):
    return int(deg * 4096 / 360)

port = PortHandler(PORT)
packet = PacketHandler(2.0)

if not port.openPort():
    sys.exit()
if not port.setBaudRate(BAUD):
    sys.exit()

packet.write1ByteTxRx(port, DXL_ID_STEER, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, DXL_ID_STEER, ADDR_MODE, POSITION_MODE)
packet.write4ByteTxRx(port, DXL_ID_STEER, ADDR_PROFILE_ACCEL, PROFILE_ACCEL)
packet.write4ByteTxRx(port, DXL_ID_STEER, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)
packet.write1ByteTxRx(port, DXL_ID_STEER, ADDR_TORQUE, TORQUE_ON)

zero_pos, result, error = packet.read4ByteTxRx(port, DXL_ID_STEER, ADDR_PRESENT_POSITION)
if result != COMM_SUCCESS:
    sys.exit()

pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    sys.exit()
joystick = pygame.joystick.Joystick(0)
joystick.init()

if __name__ == '__main__':
    try:
        while True:
            pygame.event.pump()
            
            axis0 = joystick.get_axis(0)
            
            target_angle = axis0 * MAX_ANGLE
            pos_offset = degree_to_position_unit(target_angle)
            goal_position = zero_pos + pos_offset
            
            if goal_position < 0: goal_position = 0
            if goal_position > 4095: goal_position = 4095
            
            packet.write4ByteTxRx(port, DXL_ID_STEER, ADDR_GOAL_POSITION, int(goal_position))
            
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        packet.write4ByteTxRx(port, DXL_ID_STEER, ADDR_GOAL_POSITION, zero_pos)
        time.sleep(0.5)
        packet.write1ByteTxRx(port, DXL_ID_STEER, ADDR_TORQUE, TORQUE_OFF)
        port.closePort()
        pygame.quit()