#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import pygame
from dynamixel_sdk import *

PORT = "/dev/ttyUSB0"
BAUD = 57600
ID_THROTTLE = 1
ID_STEER = 2

ADDR_MODE = 11
ADDR_TORQUE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112

VELOCITY_MODE = 1
POSITION_MODE = 3
TORQUE_ON = 1
TORQUE_OFF = 0

TARGET_RPM = 100
VELOCITY_UNIT_RPM = 0.229
MAX_STEER_DEG = 50
DEADZONE = 0.18
STEP = 0.10

def to_uint32(value):
    value = int(value)
    if value < 0:
        value = value + 4294967296
    return value

def deg_to_unit(deg):
    return int(deg * 4096 / 360)

def quantize(val):
    if abs(val) < DEADZONE:
        return 0.0
    sign = 1 if val >= 0 else -1
    q = round(abs(val) / STEP) * STEP
    return sign * max(0.0, min(1.0, q))

port = PortHandler(PORT)
packet = PacketHandler(2.0)

if not port.openPort() or not port.setBaudRate(BAUD):
    sys.exit()

packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_MODE, VELOCITY_MODE)
packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_PROFILE_ACCEL, 100)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_ON)

packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, ID_STEER, ADDR_MODE, POSITION_MODE)
packet.write4ByteTxRx(port, ID_STEER, ADDR_PROFILE_ACCEL, 100)
packet.write4ByteTxRx(port, ID_STEER, ADDR_PROFILE_VELOCITY, 500)
packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_ON)

zero_pos, res, err = packet.read4ByteTxRx(port, ID_STEER, ADDR_PRESENT_POSITION)
zero_pos = 2047
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    sys.exit()
joy = pygame.joystick.Joystick(0)
joy.init()

last_vel = None
last_pos = None

if __name__ == '__main__':
    try:
        while True:
            pygame.event.pump()
            
            raw_steer = joy.get_axis(0) 
            raw_throttle = joy.get_axis(4) 
            
            q_steer = quantize(raw_steer)
            q_throttle = quantize(raw_throttle)
            
            target_rpm = q_throttle * TARGET_RPM
            vel_val = int(target_rpm / VELOCITY_UNIT_RPM)
            
            steer_deg = q_steer * MAX_STEER_DEG
            pos_val = int(zero_pos + deg_to_unit(steer_deg))
#             print("zero_pos=",zero_pos,"pos_val=",pos_val)
            
            if vel_val != last_vel:
                packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_GOAL_VELOCITY, to_uint32(vel_val))
                last_vel = vel_val
                
            if pos_val != last_pos:
                packet.write4ByteTxRx(port, ID_STEER, ADDR_GOAL_POSITION, pos_val)
                last_pos = pos_val
                
            print(f"Steer: {q_steer*100:+.0f}% | Throttle: {q_throttle*100:+.0f}%", end='\r')
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_GOAL_VELOCITY, 0)
        packet.write4ByteTxRx(port, ID_STEER, ADDR_GOAL_POSITION, zero_pos)
        time.sleep(0.5)
        packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
        packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
        port.closePort()
        pygame.quit()