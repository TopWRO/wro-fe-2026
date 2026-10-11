#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import math
import serial
from collections import deque
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

TARGET_RPM = 20  # 自动前进速度设定为 30 RPM
VELOCITY_UNIT_RPM = 0.229
MAX_STEER_DEG = 50

# ================= 自动驾驶算法参数 =================
SENSOR_DIST = 16.0       # 两个传感器之间的距离 (cm)
TARGET_WALL_DIST = 30.0  # 期望距离右墙的距离 (cm)

K_DIST =1            # 距离误差权重 
K_POSTURE = 1         # 姿态误差权重

SUB_error_dist = 0

# ================= 垂直板检测逻辑类 =================
class BoardDetector:
    def __init__(self, threshold=15.0, history_len=5):
        self.threshold = threshold
        self.history = deque(maxlen=history_len)
        self.is_inside_board = False
        self.normal_baseline = TARGET_WALL_DIST

    def update(self, current_val, is_moving):
        if not is_moving:
            self.history.clear()
            self.is_inside_board = False
            return False

        if current_val <= 0 or current_val > 100:
            return False

        detected = False

        if len(self.history) == self.history.maxlen:
            if not self.is_inside_board:
                self.normal_baseline = sum(list(self.history)[:-1]) / (self.history.maxlen - 1)

            if not self.is_inside_board and (self.normal_baseline - current_val) >= self.threshold:
                self.is_inside_board = True

            elif self.is_inside_board and (current_val - (self.normal_baseline - self.threshold)) > 5.0:
                if current_val > (TARGET_WALL_DIST - 10.0):
                    self.is_inside_board = False
                    detected = True

        self.history.append(current_val)
        return detected

# ================= 辅助函数 =================
def to_uint32(value):
    value = int(value)
    if value < 0:
        value = value + 4294967296
    return value

def deg_to_unit(deg):
    return int(deg * 4096 / 360)

# ================= 初始化 =================
try:
    ser_tof = serial.Serial(TOF_PORT, TOF_BAUD, timeout=0)
    print("Opened ToF Sensor at", ser_tof.name)
except Exception as e:
    print(f"Failed to open ToF port: {e}")
    sys.exit()

port = PortHandler(DYNAMIXEL_PORT)
packet = PacketHandler(2.0)

if not port.openPort() or not port.setBaudRate(DYNAMIXEL_BAUD):
    print("Failed to open Dynamixel port.")
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

zero_pos = 2047
detector = BoardDetector(threshold=15.0)

ser_tof.write(b"reset gyro\n")
print("[PY] reset gyro sent")
time.sleep(3)

# ================= 主循环 =================
if __name__ == '__main__':
    last_vel = None
    last_pos = None
    buffer = ""
    
    auto_steer = 0.0  
    d_perp = 0.0      
    diff = 0.0        
    right_val = -1.0
    back_val = -1.0

    print("\nStarting Autonomous Control Loop... (Max 5 seconds)")
    
    # 记录程序发车时的起始时间
    program_start_time = time.time()
    
    try:
        while True:
            # === 时间限制检测 ===
            elapsed_time = time.time() - program_start_time
            if elapsed_time >= 300.0:
                print("\n\n[INFO] 已自动行驶满 10 秒，未检测到板子，自动停车退出。")
                break # 时间到了，跳出循环，进入 finally 刹车断电
            
            # === 1. 自动驾驶速度控制 ===
            target_rpm = TARGET_RPM
            is_moving = True
            vel_val = int(target_rpm / VELOCITY_UNIT_RPM)

            # === 2. 传感器读取与自动转向计算 ===
            n = ser_tof.in_waiting
            if n:
                chunk = ser_tof.read(n).decode('utf-8', errors='ignore')
                if chunk:
                    buffer += chunk
                    *lines, buffer = buffer.split('\n') 
                    
                    latest_valid_parts = None
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
                            right_val = float(latest_valid_parts[2])
                            back_val  = float(latest_valid_parts[3])
                            
                            if right_val == -1: right_val = 55
                            if back_val == -1: back_val = 55
                            
                            # ----------- 垂直板检测与停车逻辑 -----------
                            if detector.update(right_val, is_moving):
                                print("\n\n" + "="*45)
                                print(f"【核心提示】在第 {elapsed_time:.1f} 秒检测到垂直板子！车辆立即停止。")
                                print("="*45)
                                break # 看到板子了，跳出循环，进入 finally 刹车断电
                            # ----------------------------------------

                            diff = right_val - back_val
                            
                            cos_theta = SENSOR_DIST / math.sqrt(SENSOR_DIST**2 + diff**2)
                            d_mid = (right_val + back_val) / 2.0
                            d_perp = d_mid * cos_theta
                            
                            error_dist = (d_perp - TARGET_WALL_DIST)/TARGET_WALL_DIST
                            error_posture = diff
                            
                            raw_steer = (K_DIST * error_dist) #+ (K_POSTURE * error_posture)
                            auto_steer = raw_steer#max(-1.0, min(1.0, raw_steer))
                            SUB_error_dist += auto_steer
                            auto_steer = max(-1.0, min(1.0, SUB_error_dist))/4
                            

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
                
            # 实时打印运行时间与数据
            print(f"时间: {elapsed_time:.1f}s/5.0s | 垂线: {d_perp:>5.1f}cm | Steer: {auto_steer:>+5.2f} | R:{right_val:>5.1f} B:{back_val:>5.1f}   ", end='\r')
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\nShutting down safely (Ctrl+C)...")
    finally:
        # ===== 安全刹车与断电保护 =====
        # 不论是遇到板子(break)、满5秒(break)还是手动打断(Ctrl+C)，都会执行这里，保证安全停车。
        packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_GOAL_VELOCITY, 0)
        packet.write4ByteTxRx(port, ID_STEER, ADDR_GOAL_POSITION, zero_pos)
        time.sleep(0.5)
        packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
        packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
        port.closePort()
        ser_tof.close()