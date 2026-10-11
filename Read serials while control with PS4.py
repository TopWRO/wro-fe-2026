#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import serial
import pygame
from dynamixel_sdk import *

# ==========================================
# 传感器串口配置 (TOF Sensor & Gyro)
# ==========================================
SENSOR_PORT = "/dev/ttyACM0"
SENSOR_BAUD = 115200

# ==========================================
# 电机驱动配置 (Dynamixel)
# ==========================================
MOTOR_PORT = "/dev/ttyUSB0"
MOTOR_BAUD = 57600
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

TARGET_RPM = 30
VELOCITY_UNIT_RPM = 0.229
MAX_STEER_DEG = 50
DEADZONE = 0.18
STEP = 0.10

# --- 辅助函数 ---
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

# ==========================================
# 1. 初始化传感器串口
# ==========================================
try:
    sensor_ser = serial.Serial(SENSOR_PORT, SENSOR_BAUD, timeout=0)
    print(f"[PY] 传感器串口已打开: {sensor_ser.name}")
except Exception as e:
    print(f"[ERROR] 无法打开传感器串口 {SENSOR_PORT}: {e}")
    sys.exit()

# ==========================================
# 2. 初始化 Dynamixel 电机
# ==========================================
port = PortHandler(MOTOR_PORT)
packet = PacketHandler(2.0)

if not port.openPort() or not port.setBaudRate(MOTOR_BAUD):
    print(f"[ERROR] 无法打开电机串口或设置波特率: {MOTOR_PORT}")
    sys.exit()

# 配置油门电机 (Throttle)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_MODE, VELOCITY_MODE)
packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_PROFILE_ACCEL, 100)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_ON)

# 配置转向电机 (Steer)
packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, ID_STEER, ADDR_MODE, POSITION_MODE)
packet.write4ByteTxRx(port, ID_STEER, ADDR_PROFILE_ACCEL, 100)
packet.write4ByteTxRx(port, ID_STEER, ADDR_PROFILE_VELOCITY, 500)
packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_ON)

zero_pos = 2047  # 直接采用原代码指定的零位

# ==========================================
# 3. 初始化手柄 (Pygame)
# ==========================================
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("[ERROR] 未检测到手柄！")
    sys.exit()
joy = pygame.joystick.Joystick(0)
joy.init()
print(f"[PY] 手柄已连接: {joy.get_name()}")

# ==========================================
# 4. 主循环
# ==========================================
last_vel = None
last_pos = None
buffer = ""
timer1 = time.time()

# 用于打印的传感器状态缓存
angle, front, right, back, left = 0.0, 0.0, 0.0, 0.0, 0.0

print("\n--- 车辆控制及传感器监控已启动 (按 Ctrl+C 退出) ---")

if __name__ == '__main__':
    try:
        while True:
            # ----------------------------------------
            # 任务 A: 读取并处理手柄控制
            # ----------------------------------------
            pygame.event.pump()
            
            raw_steer = joy.get_axis(0) 
            raw_throttle = joy.get_axis(4) 
            
            q_steer = quantize(raw_steer)
            q_throttle = quantize(raw_throttle)
            
            target_rpm = q_throttle * TARGET_RPM
            vel_val = int(target_rpm / VELOCITY_UNIT_RPM)
            
            steer_deg = q_steer * MAX_STEER_DEG
            pos_val = int(zero_pos + deg_to_unit(steer_deg))
            
            # 仅在目标值发生变化时发送指令，避免占用串口带宽
            if vel_val != last_vel:
                packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_GOAL_VELOCITY, to_uint32(vel_val))
                last_vel = vel_val
                
            if pos_val != last_pos:
                packet.write4ByteTxRx(port, ID_STEER, ADDR_GOAL_POSITION, pos_val)
                last_pos = pos_val

            # ----------------------------------------
            # 任务 B: 读取并解析传感器串口数据
            # ----------------------------------------
            n = sensor_ser.in_waiting
            if n:
                chunk = sensor_ser.read(n).decode('utf-8', errors='ignore')
                if chunk:
                    buffer += chunk
                    # 按行拆分，buffer 中保留最后一段（可能是不完整行）
                    *lines, buffer = buffer.split('\n')  

                    for line in lines:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            continue

                        parts = line.split(',')
                        if len(parts) == 5:
                            try:
                                angle, front, right, back, left = map(float, parts)
                            except ValueError:
                                pass
            
            # 定时器：每 10 秒可用于发送指令（例如重置陀螺仪）
            if time.time() > (timer1 + 10):
                # sensor_ser.write(b"reset gyro\n")
                timer1 = time.time()

            # ----------------------------------------
            # 任务 C: 合并控制台输出状态栏
            # ----------------------------------------
            # 使用 \r 覆盖打印，让终端看起来非常干净
            print(f"Sensors [Gyro:{angle:6.2f} | F:{front:5.1f} R:{right:5.1f} B:{back:5.1f} L:{left:5.1f}] || Controller [Steer: {q_steer*100:+.0f}% | Throttle: {q_throttle*100:+.0f}%]", end='\r')
            
            # 小延迟，将循环频率控制在约 100Hz
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\n[PY] 检测到中断，正在关闭系统...")
        
        # 安全停止电机
        packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_GOAL_VELOCITY, 0)
        packet.write4ByteTxRx(port, ID_STEER, ADDR_GOAL_POSITION, zero_pos)
        time.sleep(0.5)
        packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
        packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
        
        # 关闭所有端口及 Pygame
        port.closePort()
        sensor_ser.close()
        pygame.quit()
        print("[PY] 退出完成。")