#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PS4 Manual Control + Front Obstacle Auto Left Turn + Gyro 85 Degree Stop

功能：
1. 正常情况下，完全用 PS4 手柄控制车：
   - axis 0 控制转向
   - axis 4 控制油门

2. 当前方 ToF 传感器 front < 40 cm 时，程序自动接管：
   - 车自动往左打满方向
   - 车用固定速度继续转弯
   - 记录触发瞬间的陀螺仪 angle 作为起始角度

3. 当陀螺仪检测到相对转动角度 >= 85 度时：
   - 油门停止
   - 方向回正
   - 机器人保持停止，直到重新运行程序

串口数据格式：
    angle, front, right, back, left

基于你的原始 yanQiangKongZhi.py：保留 Dynamixel、PS4、串口读取方法。
"""

import sys
import time
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

TARGET_RPM = 20
VELOCITY_UNIT_RPM = 0.229
MAX_STEER_DEG = 50
DEADZONE = 0.18
STEP = 0.10
LOOP_DELAY = 0.01

# ================= 自动避障左转参数 =================
FRONT_TRIGGER_CM = 60.0       # 前方小于 40 cm 时接管
TURN_STOP_ANGLE_DEG = 85.0    # 相对转动大于 85 度后停止
AUTO_TURN_RPM = 45.0          # 自动左转时的油门速度
AUTO_LEFT_STEER = -1.0        # 左打满方向。若实际变成右转，改成 +1.0

# 如果你的车手柄向前推出来是负数，就保持 -1.0。
# 如果自动接管时车反而倒车，把它改成 +1.0。
AUTO_FORWARD_SIGN = -1.0

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


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle_deg(angle):
    """把角度限制在 -180 到 +180，避免 359 到 0 跳变出错。"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def parse_tof_line(line):
    """
    解析串口数据：angle, front, right, back, left
    返回 tuple 或 None。
    """
    line = line.strip()
    if not line or line.startswith('#'):
        return None

    parts = line.split(',')
    if len(parts) != 5:
        return None

    try:
        angle = float(parts[0])
        front = float(parts[1])
        right = float(parts[2])
        back = float(parts[3])
        left = float(parts[4])
        return angle, front, right, back, left
    except ValueError:
        return None


# ================= 初始化 ToF 串口 =================
try:
    ser_tof = serial.Serial(TOF_PORT, TOF_BAUD, timeout=0)
    print("Opened ToF Sensor at", ser_tof.name)
except Exception as e:
    print(f"Failed to open ToF port: {e}")
    sys.exit(1)

# ================= 初始化 Dynamixel =================
port = PortHandler(DYNAMIXEL_PORT)
packet = PacketHandler(2.0)

if not port.openPort() or not port.setBaudRate(DYNAMIXEL_BAUD):
    print("Failed to open Dynamixel port.")
    sys.exit(1)

# 油门电机 Velocity Mode
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_MODE, VELOCITY_MODE)
packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_PROFILE_ACCEL, 100)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_ON)

# 转向电机 Position Mode
packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, ID_STEER, ADDR_MODE, POSITION_MODE)
packet.write4ByteTxRx(port, ID_STEER, ADDR_PROFILE_ACCEL, 100)
packet.write4ByteTxRx(port, ID_STEER, ADDR_PROFILE_VELOCITY, 500)
packet.write1ByteTxRx(port, ID_STEER, ADDR_TORQUE, TORQUE_ON)

zero_pos = 2047

# ================= 初始化 PS4 手柄 =================
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick found.")
    sys.exit(1)

joy = pygame.joystick.Joystick(0)
joy.init()
print("Joystick connected:", joy.get_name())
print("Manual mode: axis 0 = steering, axis 4 = throttle")

# ================= 重置 gyro =================
try:
    ser_tof.write(b"reset gyro\n")
    print("[PY] reset gyro sent")
    time.sleep(3)
except Exception as e:
    print("[PY] reset gyro failed:", e)


if __name__ == '__main__':
    last_vel = None
    last_pos = None
    buffer = ""

    # 最新传感器值
    angle_val = 0.0
    front_val = -1.0
    right_val = -1.0
    back_val = -1.0
    left_val = -1.0

    # 状态机：MANUAL -> AUTO_LEFT -> STOPPED
    mode = "MANUAL"
    turn_start_angle = None
    turn_delta = 0.0

    print("\nStarting Control Loop... Press Ctrl+C to stop.")

    try:
        while True:
            # ================= 1. 读取手柄 =================
            pygame.event.pump()
            raw_steer = joy.get_axis(0)
            raw_throttle = joy.get_axis(4)

            q_steer = quantize(raw_steer)
            q_throttle = quantize(raw_throttle)

            # ================= 2. 读取串口传感器 =================
            n = ser_tof.in_waiting
            if n:
                chunk = ser_tof.read(n).decode('utf-8', errors='ignore')
                if chunk:
                    buffer += chunk
                    *lines, buffer = buffer.split('\n')

                    latest_data = None
                    for line in lines:
                        data = parse_tof_line(line)
                        if data is not None:
                            latest_data = data

                    if latest_data is not None:
                        angle_val, front_val, right_val, back_val, left_val = latest_data

            # ================= 3. 状态切换逻辑 =================
            # front = -1 通常代表无效值，不用它触发自动接管
            front_valid = front_val is not None and front_val > 0

            if mode == "MANUAL":
                if front_valid and front_val < FRONT_TRIGGER_CM:
                    mode = "AUTO_LEFT"
                    turn_start_angle = angle_val
                    turn_delta = 0.0
                    print("\n[AUTO] front < 40 cm, auto left turn started.")

            elif mode == "AUTO_LEFT":
                if turn_start_angle is not None:
                    turn_delta = abs(angle_val - turn_start_angle)

                if turn_delta >= TURN_STOP_ANGLE_DEG:
                    mode = "STOPPED"
                    print("\n[AUTO] turned over 85 degrees, robot stopped.")

            # ================= 4. 根据状态决定油门和方向 =================
            if mode == "MANUAL":
                # 正常手柄控制
                steer_cmd = q_steer
                rpm_cmd = q_throttle * TARGET_RPM

            elif mode == "AUTO_LEFT":
                # 程序接管：固定左转 + 固定前进速度
                steer_cmd = AUTO_LEFT_STEER
                rpm_cmd = AUTO_FORWARD_SIGN * AUTO_TURN_RPM

            else:  # STOPPED
                # 停止并回正
                steer_cmd = 0.0
                rpm_cmd = 0.0

            steer_cmd = clamp(steer_cmd, -1.0, 1.0)

            vel_val = int(rpm_cmd / VELOCITY_UNIT_RPM)
            steer_deg = steer_cmd * MAX_STEER_DEG
            pos_val = int(zero_pos + deg_to_unit(steer_deg))

            # ================= 5. 发送 Dynamixel 指令 =================
            if vel_val != last_vel:
                packet.write4ByteTxRx(
                    port,
                    ID_THROTTLE,
                    ADDR_GOAL_VELOCITY,
                    to_uint32(vel_val)
                )
                last_vel = vel_val

            if pos_val != last_pos:
                packet.write4ByteTxRx(
                    port,
                    ID_STEER,
                    ADDR_GOAL_POSITION,
                    pos_val
                )
                last_pos = pos_val

            # ================= 6. 实时打印 =================
            print(
                f"Mode:{mode:<9} | "
                f"A:{angle_val:>7.2f} Δ:{turn_delta:>6.1f} | "
                f"F:{front_val:>6.1f} R:{right_val:>6.1f} B:{back_val:>6.1f} L:{left_val:>6.1f} | "
                f"Steer:{steer_cmd:>+5.2f} RPM:{rpm_cmd:>+6.1f} | "
                f"JoyS:{q_steer*100:>+4.0f}% JoyT:{q_throttle*100:>+4.0f}%   ",
                end='\r'
            )

            time.sleep(LOOP_DELAY)

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
