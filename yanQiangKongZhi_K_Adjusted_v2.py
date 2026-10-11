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
K_DIST = 0.5          # 距离误差权重：D-pad 上/下 调整
K_POSTURE = 1.5       # 姿态误差权重：D-pad 右/左 调整
K_STEP = 0.1          # 每按一次方向键，K值变化 0.1
K_MIN = 0.0           # K值最小值，防止误操作调成负数
K_MAX = 5.0           # K值最大值，可根据测试需要调整

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

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

def get_dpad_state(joystick):
    """
    Returns D-pad state as (x, y):
        x =  1 means right, x = -1 means left
        y =  1 means up,    y = -1 means down

    Many PS4 controllers show the D-pad as a HAT in pygame.
    Some controllers may show the D-pad as buttons instead, so this
    function also includes a simple fallback for common PS4 button indexes.
    """
    x, y = 0, 0

    # Preferred method: D-pad as HAT
    if joystick.get_numhats() > 0:
        x, y = joystick.get_hat(0)

    # Fallback method: D-pad as buttons
    # Common pygame/PS4 mapping can vary by OS/driver.
    # If your D-pad does not respond, run a pygame button test and update these indexes.
    if x == 0 and y == 0 and joystick.get_numbuttons() >= 15:
        up = joystick.get_button(11)
        down = joystick.get_button(12)
        left = joystick.get_button(13)
        right = joystick.get_button(14)

        if right:
            x = 1
        elif left:
            x = -1

        if up:
            y = 1
        elif down:
            y = -1

    return x, y

def adjust_k_values(dpad_state, previous_dpad_state, k_dist, k_posture):
    """
    Adjust K values only once per D-pad press.

    Up    -> increase K_DIST by 0.1
    Down  -> decrease K_DIST by 0.1
    Right -> increase K_POSTURE by 0.1
    Left  -> decrease K_POSTURE by 0.1
    """
    x, y = dpad_state
    prev_x, prev_y = previous_dpad_state

    # Only trigger when the direction changes from not pressed to pressed.
    if y == 1 and prev_y != 1:
        k_dist = clamp(round(k_dist + K_STEP, 2), K_MIN, K_MAX)
        print(f"\nK_DIST increased to {k_dist:.1f}")
    elif y == -1 and prev_y != -1:
        k_dist = clamp(round(k_dist - K_STEP, 2), K_MIN, K_MAX)
        print(f"\nK_DIST decreased to {k_dist:.1f}")

    if x == 1 and prev_x != 1:
        k_posture = clamp(round(k_posture + K_STEP, 2), K_MIN, K_MAX)
        print(f"\nK_POSTURE increased to {k_posture:.1f}")
    elif x == -1 and prev_x != -1:
        k_posture = clamp(round(k_posture - K_STEP, 2), K_MIN, K_MAX)
        print(f"\nK_POSTURE decreased to {k_posture:.1f}")

    return k_dist, k_posture

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

    # D-pad previous state, used to avoid changing K repeatedly while holding a button.
    last_dpad_state = (0, 0)

    # Local adjustable K values. These start from the default values above.
    k_dist = K_DIST
    k_posture = K_POSTURE
    
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

            # === 1.5 D-pad读取并调整两个K值 ===
            # Up/Down adjusts K_DIST. Right/Left adjusts K_POSTURE.
            dpad_state = get_dpad_state(joy)
            k_dist, k_posture = adjust_k_values(dpad_state, last_dpad_state, k_dist, k_posture)
            last_dpad_state = dpad_state

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
                            raw_steer = (k_dist * error_dist) + (k_posture * error_posture)
                            
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
            print(f"垂线: {d_perp:>5.1f}cm | R-B: {diff:>+5.1f}cm | Steer: {auto_steer:>+5.2f} | 油门: {q_throttle*100:>+4.0f}% | K_DIST:{k_dist:>3.1f} K_POSTURE:{k_posture:>3.1f} | R:{right_val:>5.1f} B:{back_val:>5.1f}   ", end='\r')
#             print(right_val, back_val)
            time.sleep(0.05) # 50ms循环，约每秒20次检测/控制
            
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