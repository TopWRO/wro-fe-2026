#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
yanQiangKongZhi_forward_reverse.py

基于原来的沿墙控制程序优化：
1. 保留原来的串口 ToF 数据读取方式。
2. 保留 PS4 手柄 axis 4 控制前进/后退油门。
3. 保留 Dynamixel 油门电机 velocity mode 和转向电机 position mode。
4. 增加倒车沿墙逻辑：
   - 前进时使用正常自动转向。
   - 倒车时自动把转向修正方向反过来。
5. 实时打印前进/倒车方向、垂直距离、R-B、原始转向、最终转向、油门等数据。

传感器串口格式仍然是：
    angle, front, right, back, left

这里沿用你原程序的用法：
    right = 右前/右侧传感器
    back  = 右后传感器
"""

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
LOOP_DELAY = 0.01

# ================= 自动驾驶算法参数 =================
SENSOR_DIST = 18.0        # 两个右侧传感器之间的前后距离 cm
TARGET_WALL_DIST = 30.0   # 期望离右墙距离 cm

# 权重参数：如果车修正太慢，加大；如果左右抖动，减小
K_DIST = 0.5              # 距离误差权重
K_POSTURE = 1.5           # 平行/姿态误差权重

# 倒车优化核心：倒车时车的运动响应与前进相反，所以转向修正需要反向
REVERSE_STEER_MULTIPLIER = -1.0
FORWARD_STEER_MULTIPLIER = 1.0

# 如果你的舵机方向和程序相反，把这个改成 -1.0
STEER_DIRECTION = 1.0

# 无效测距时使用的保底值
INVALID_DISTANCE_CM = 55.0

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


def parse_tof_line(line):
    """
    串口格式：angle, front, right, back, left
    返回 tuple 或 None
    """
    line = line.strip()
    if not line or line.startswith('#'):
        return None

    parts = line.split(',')
    if len(parts) != 5:
        return None

    try:
        angle, front, right, back, left = map(float, parts)
        return angle, front, right, back, left
    except ValueError:
        return None


def valid_distance(value):
    """把 -1 或异常值替换成保底距离，避免算法崩掉。"""
    if value is None or value < 0:
        return INVALID_DISTANCE_CM
    return value


def calculate_wall_steer(right_val, back_val, q_throttle):
    """
    根据右前 right_val 和右后 back_val 计算沿墙转向。

    前进：final_steer = raw_steer
    倒车：final_steer = -raw_steer

    返回：
        final_steer, raw_steer, d_perp, diff, error_dist, error_posture, drive_mode
    """
    right_val = valid_distance(right_val)
    back_val = valid_distance(back_val)

    # 右前 - 右后：用于判断车身是不是与墙平行
    diff = right_val - back_val

    # 计算车身中点到墙面的近似垂直距离
    cos_theta = SENSOR_DIST / math.sqrt(SENSOR_DIST ** 2 + diff ** 2)
    d_mid = (right_val + back_val) / 2.0
    d_perp = d_mid * cos_theta

    # 归一化误差
    error_dist = (d_perp - TARGET_WALL_DIST) / TARGET_WALL_DIST
    error_posture = diff / TARGET_WALL_DIST

    # 原始沿墙转向：正值默认代表向右打方向
    raw_steer = (K_DIST * error_dist) + (K_POSTURE * error_posture)
    raw_steer = clamp(raw_steer, -1.0, 1.0)

    # 根据当前油门方向决定转向是否反向
    if q_throttle > 0:
        drive_mode = "FWD"
        final_steer = raw_steer * FORWARD_STEER_MULTIPLIER
    elif q_throttle < 0:
        drive_mode = "REV"
        final_steer = raw_steer * REVERSE_STEER_MULTIPLIER
    else:
        drive_mode = "STOP"
        # 停车时回正，避免原地一直打舵
        final_steer = 0.0

    final_steer = clamp(final_steer * STEER_DIRECTION, -1.0, 1.0)

    return final_steer, raw_steer, d_perp, diff, error_dist, error_posture, drive_mode


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

# 配置油门电机 Velocity Mode
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_MODE, VELOCITY_MODE)
packet.write4ByteTxRx(port, ID_THROTTLE, ADDR_PROFILE_ACCEL, 100)
packet.write1ByteTxRx(port, ID_THROTTLE, ADDR_TORQUE, TORQUE_ON)

# 配置转向电机 Position Mode
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
print("Axis 4 = throttle. Push forward/backward to test FWD/REV wall following.")

# ================= 主循环 =================
ser_tof.write(b"reset gyro\n")
print("[PY] reset gyro sent")
time.sleep(3)

if __name__ == '__main__':
    last_vel = None
    last_pos = None
    buffer = ""

    # 初始状态变量
    final_steer = 0.0
    raw_steer = 0.0
    d_perp = 0.0
    diff = 0.0
    error_dist = 0.0
    error_posture = 0.0
    right_val = INVALID_DISTANCE_CM
    back_val = INVALID_DISTANCE_CM
    drive_mode = "STOP"

    print("\nStarting Forward/Reverse Wall Control Loop... Press Ctrl+C to stop.")

    try:
        while True:
            # === 1. 遥控器读取：只控制前后油门 ===
            pygame.event.pump()
            raw_throttle = joy.get_axis(4)
            q_throttle = quantize(raw_throttle)

            target_rpm = q_throttle * TARGET_RPM
            vel_val = int(target_rpm / VELOCITY_UNIT_RPM)

            # === 2. 串口读取传感器数据 ===
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

            # === 3. 根据最新传感器和油门方向计算自动转向 ===
            final_steer, raw_steer, d_perp, diff, error_dist, error_posture, drive_mode = calculate_wall_steer(
                right_val,
                back_val,
                q_throttle
            )

            # === 4. 发送电机指令 ===
            steer_deg = final_steer * MAX_STEER_DEG
            pos_val = int(zero_pos + deg_to_unit(steer_deg))

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

            # === 5. 实时状态打印 ===
            print(
                f"Mode:{drive_mode:>4} | "
                f"垂线:{d_perp:>5.1f}cm | "
                f"R-B:{diff:>+6.1f}cm | "
                f"E_dist:{error_dist:>+5.2f} | "
                f"E_post:{error_posture:>+5.2f} | "
                f"Raw:{raw_steer:>+5.2f} | "
                f"Final:{final_steer:>+5.2f} | "
                f"油门:{q_throttle*100:>+4.0f}% | "
                f"R:{valid_distance(right_val):>5.1f} B:{valid_distance(back_val):>5.1f}   ",
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
        print("Done.")
