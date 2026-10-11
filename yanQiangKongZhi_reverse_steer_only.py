#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import math
import serial
import pygame
from dynamixel_sdk import *

# ================= 传感器与串口配置 =================
TOF_PORT = "/dev/ttyACM0"
TOF_BAUD = 115200

# ================= 电机与控制配置 =================
DYNAMIXEL_PORT = "/dev/ttyUSB0"
DYNAMIXEL_BAUD = 57600
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

TARGET_RPM = 50
VELOCITY_UNIT_RPM = 0.229
MAX_STEER_DEG = 50
DEADZONE = 0.18
STEP = 0.10

# ================= 自动驾驶算法参数 =================
SENSOR_DIST = 18.0       # 两个传感器之间的距离 (cm)
TARGET_WALL_DIST = 30.0  # 期望距离右墙的距离 (cm)

# 权重参数 (需要根据实际测试时的车体反应速度微调)
K_DIST = 0.5          # 距离误差权重 
K_POSTURE = 1.5         # 姿态误差权重 

# ================= 辅助函数 =================
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

# ================= 初始化 =================
# 1. 初始化 ToF 串口
try:
    ser_tof = serial.Serial(TOF_PORT, TOF_BAUD, timeout=0)
    print("Opened ToF Sensor at", ser_tof.name)
except Exception as e:
    print(f"Failed to open ToF port: {e}")
    sys.exit()

# 2. 初始化 Dynamixel
port = PortHandler(DYNAMIXEL_PORT)
packet = PacketHandler(2.0)

if not port.openPort() or not port.setBaudRate(DYNAMIXEL_BAUD):
    print("Failed to open Dynamixel port.")
    sys.exit()

# 配置油门电机 (Velocity Mode)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_MODE, VELOCITY_MODE)
packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_PROFILE_ACCEL, 100)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_ON)

# 配置转向电机 (Position Mode)
packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, ID_STEER, ADDR_MODE, POSITION_MODE)
packet.write4ByteTxRx(port, ID_STEER, ADDR_PROFILE_ACCEL, 100)
packet.write4ByteTxRx(port, ID_STEER, ADDR_PROFILE_VELOCITY, 500)
packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_ON)

zero_pos = 2047

# 3. 初始化 Pygame (手柄)
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No joystick found.")
    sys.exit()
joy = pygame.joystick.Joystick(0)
joy.init()

# ================= 主循环 =================

ser_tof.write(b"reset gyro\n")
print("[PY] reset gyro sent")
time.sleep(3)

if __name__ == '__main__':
    last_vel = None
    last_pos = None
    buffer = ""
    
    # 初始状态变量设置
    auto_steer = 0.0  
    d_perp = 0.0      
    diff = 0.0        
    right_val = -1.0
    back_val = -1.0

    print("\nStarting Control Loop... Press Ctrl+C to stop.")
    try:
        while True:
            # === 1. 遥控器读取 (只控制前后/油门) ===
            pygame.event.pump()
            raw_throttle = joy.get_axis(4) 
            q_throttle = quantize(raw_throttle)
            
            target_rpm = q_throttle * TARGET_RPM
            vel_val = int(target_rpm / VELOCITY_UNIT_RPM)

            # === 2. 传感器读取与自动转向计算 ===
            n = ser_tof.in_waiting
            if n:
                chunk = ser_tof.read(n).decode('utf-8', errors='ignore')
                if chunk:
                    buffer += chunk
                    *lines, buffer = buffer.split('\n') 
                    
                    latest_valid_parts = None
                    # 只取最后一行有效数据，降低延迟
                    for line in lines:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            continue
                        parts = line.split(',')
                        if len(parts) == 5:
                            latest_valid_parts = parts
                    
                    if latest_valid_parts:
                        try:
                            front_val = float(latest_valid_parts[1])
                            right_val = float(latest_valid_parts[2]) # R
                            back_val  = float(latest_valid_parts[3]) # B
                            
                            # 如果测距有效则更新方向
                            if right_val == -1:
                                right_val =55
                            if back_val == -1:
                                back_val =55
                            
                            diff = right_val - back_val
                            
                            # 计算中点与墙面的垂直距离
                            cos_theta = SENSOR_DIST / math.sqrt(SENSOR_DIST**2 + diff**2)
                            d_mid = (right_val + back_val) / 2.0
                            d_perp = d_mid * cos_theta
                            
                            # 计算误差
                            error_dist = (d_perp - TARGET_WALL_DIST)/TARGET_WALL_DIST
                            error_posture = diff/TARGET_WALL_DIST
                            
                            # 综合计算转向值 (假设向右转为正值，需根据实际舵机方向决定是否乘 -1)
                            raw_steer = (K_DIST * error_dist) + (K_POSTURE * error_posture)

                            # 倒车时，车辆运动方向相反，所以 steering 需要取负号
                            if q_throttle < 0:
                                raw_steer = -raw_steer
                            
                            # 归一化限制在 -1 到 1 之间
                            auto_steer = max(-1.0, min(1.0, raw_steer))


                        except ValueError:
                            pass

            # === 3. 发送指令给电机 ===
            steer_deg = auto_steer * MAX_STEER_DEG
            pos_val = int(zero_pos + deg_to_unit(steer_deg))
            
            if vel_val != last_vel:
                packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_GOAL_VELOCITY, to_uint32(vel_val))
                last_vel = vel_val
                
            if pos_val != last_pos:
                packet.write4ByteTxRx(port, ID_STEER, ADDR_GOAL_POSITION, pos_val)
                last_pos = pos_val
                
            # 实时状态打印
            print(f"垂线: {d_perp:>5.1f}cm | R-B: {diff:>+5.1f}cm | Steer: {auto_steer:>+5.2f} | 油门: {q_throttle*100:>+4.0f}% | R:{right_val:>5.1f} B:{back_val:>5.1f}   ", end='\r')
#             print(right_val, back_val)
            time.sleep(0.01) # 10ms 循环，兼顾串口非阻塞和CPU占用
            
    except KeyboardInterrupt:
        print("\nShutting down safely...")
        packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_GOAL_VELOCITY, 0)
        packet.write4ByteTxRx(port, ID_STEER, ADDR_GOAL_POSITION, zero_pos)
        time.sleep(0.5)
        packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
        packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
        port.closePort()
        ser_tof.close()
        pygame.quit()