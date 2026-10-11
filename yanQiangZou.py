#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import serial
from dynamixel_sdk import *

PORT_SERIAL = "/dev/ttyACM0"
BAUD_SERIAL = 115200
ser = serial.Serial(PORT_SERIAL, BAUD_SERIAL, timeout=0)

PORT_MOTOR = "/dev/ttyUSB0"
BAUD_MOTOR = 57600
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

VELOCITY_UNIT_RPM = 0.229
MAX_STEER_DEG = 50

TARGET_DIST = 30.0    
SENSOR_DIST = 18.0    
CRUISE_RPM = -30       

Kp = 1.5   
Kd = 2.5   

def to_uint32(value):
    value = int(value)
    if value < 0:
        value = value + 4294967296
    return value

def deg_to_unit(deg):
    return int(deg * 4096 / 360)

port = PortHandler(PORT_MOTOR)
packet = PacketHandler(2.0)

if not port.openPort() or not port.setBaudRate(BAUD_MOTOR):
    sys.exit()

packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_MODE, VELOCITY_MODE)
packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_PROFILE_ACCEL, 200)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_ON)

packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, ID_STEER, ADDR_MODE, POSITION_MODE)
packet.write4ByteTxRx(port, ID_STEER, ADDR_PROFILE_ACCEL, 100)
packet.write4ByteTxRx(port, ID_STEER, ADDR_PROFILE_VELOCITY, 500)
packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_ON)

zero_pos, res, err = packet.read4ByteTxRx(port, ID_STEER, ADDR_PRESENT_POSITION)

last_vel = None
last_pos = None
buffer = ""

if __name__ == '__main__':
    try:
        start_time = time.time()
        print("开始 8 秒巡航测试...")
        
        while True:
            # 8秒退出逻辑
            if time.time() - start_time >= 8.0:
                print("\n\n[INFO] 8秒时间到，准备停车。")
                break

            # 驱动前进
            vel_val = int(CRUISE_RPM / VELOCITY_UNIT_RPM)
            if vel_val != last_vel:
                packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_GOAL_VELOCITY, to_uint32(vel_val))
                last_vel = vel_val

            # 读取传感器
            n = ser.in_waiting
            if n:
                chunk = ser.read(n).decode('utf-8', errors='ignore')
                if chunk:
                    buffer += chunk
                    *lines, buffer = buffer.split('\n')

                    for line in lines:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            continue

                        parts = line.split(',')
                        if len(parts) != 5:
                            continue

                        try:
                            angle, front, right, back, left = map(float, parts)
                            
                            current_dist = (right + back) / 2.0
                            error_dist = current_dist - TARGET_DIST
                            error_angle = right - back
                            
                            steer_deg = (Kp * error_dist) + (Kd * error_angle)
                            steer_deg = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer_deg))
                            
                            pos_val = int(zero_pos + deg_to_unit(steer_deg))
                            
                            if pos_val != last_pos:
                                packet.write4ByteTxRx(port, ID_STEER, ADDR_GOAL_POSITION, pos_val)
                                last_pos = pos_val
                                
                            # 同步打印用于 debug 的状态参数
                            print(f"R_Front:{right:5.1f} | R_Back:{back:5.1f} | Diff:{error_angle:5.1f} | DistErr:{error_dist:5.1f} | Steer:{steer_deg:5.1f}", end='\r')

                        except ValueError:
                            pass

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n[PY] 手动中断")
    finally:
        # 安全停止逻辑移至 finally，确保不论如何退出都会执行
        packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_GOAL_VELOCITY, 0)
        packet.write4ByteTxRx(port, ID_STEER, ADDR_GOAL_POSITION, zero_pos)
        time.sleep(0.5)
        packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
        packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
        
        port.closePort()
        ser.close()
        print("[PY] 电机已停止，资源已释放。")