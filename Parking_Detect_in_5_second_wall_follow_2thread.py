#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math
import serial
import threading
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

# 注意：这里是 Dynamixel 的目标 RPM，不是严格意义上的“百分比”。
# TARGET_RPM = 20 表示目标转速 20 RPM；设为 0 时车辆不前进。
TARGET_RPM = 20
VELOCITY_UNIT_RPM = 0.229

# auto_steer = ±1.0 时对应的最大转向角
MAX_STEER_DEG = 50.0

# ================= 运行时间设置 =================
MAX_RUN_SECONDS = 300.0

# ================= 两颗 ToF 沿墙控制参数 =================
SENSOR_DIST = 16.0       # 两颗右侧 ToF 测量中心的前后间距，单位 cm
TARGET_WALL_DIST = 30.0  # 车辆中心到右墙的目标垂直距离，单位 cm

# 姿态控制：负责让车体迅速恢复到与墙平行
K_POSTURE = 1

# 距离控制：负责让整车逐渐回到目标墙距
# 这里的量纲由程序吸收，可理解为“每 cm 距离误差对应多少修正强度”
K_DIST = 0.8

# 距离积分：仅消除长期固定偏差，初次测试建议保持 0
KI_DIST = 0.00

# 低速保护。TARGET_RPM 越小，距离修正不能无限增强
RPM_SOFTENING = 8.0

# ToF 一阶低通滤波。越大反应越快，越小越平滑
TOF_FILTER_ALPHA = 0.35

# 转向输出低通滤波
STEER_FILTER_ALPHA = 0.45

# 死区：减少传感器轻微波动导致的左右抖动
DIST_DEADBAND_CM = 0.6
POSTURE_DEADBAND_DEG = 0.7

# 积分只在小误差范围内工作，并限制累计值
INTEGRAL_ACTIVE_RANGE_CM = 6.0
INTEGRAL_LIMIT = 20.0

# auto_steer 每秒最大允许变化量，防止瞬间猛打方向
MAX_STEER_RATE_PER_SEC = 2.5

# 如果实车转向方向与程序相反，只把此项改成 -1.0
STEER_SIGN = 1.0

# ToF 有效范围
MIN_VALID_TOF_CM = 2.0
MAX_VALID_TOF_CM = 200.0


# ================= 两颗 ToF 沿墙控制器 =================
class TwoToFWallFollower:
    """
    right_val：右前侧朝右的 ToF 距离
    back_val ：右后侧朝右的 ToF 距离

    默认符号：
        auto_steer > 0：向右转
        auto_steer < 0：向左转
    """

    def __init__(self):
        self.filtered_right = None
        self.filtered_back = None
        self.distance_integral = 0.0
        self.last_auto_steer = 0.0
        self.last_update_time = None

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    @staticmethod
    def apply_deadband(value, deadband):
        """连续死区，避免跨过死区边缘时控制量突然跳变。"""
        if abs(value) <= deadband:
            return 0.0
        return value - math.copysign(deadband, value)

    @staticmethod
    def low_pass(new_value, old_value, alpha):
        if old_value is None:
            return new_value
        return alpha * new_value + (1.0 - alpha) * old_value

    @staticmethod
    def valid_tof(value):
        return (
            value is not None
            and math.isfinite(value)
            and MIN_VALID_TOF_CM <= value <= MAX_VALID_TOF_CM
        )

    def reset(self):
        self.filtered_right = None
        self.filtered_back = None
        self.distance_integral = 0.0
        self.last_auto_steer = 0.0
        self.last_update_time = None

    def update(self, right_val, back_val, target_rpm, now=None):
        """
        仅在收到一组新的有效 ToF 数据时调用。

        返回：
            {
                "valid": bool,
                "auto_steer": -1.0 ~ +1.0,
                ...调试数据...
            }
        """
        if now is None:
            now = time.monotonic()

        if self.last_update_time is None:
            dt = 0.05
        else:
            dt = now - self.last_update_time
            dt = self.clamp(dt, 0.005, 0.30)

        self.last_update_time = now

        if not self.valid_tof(right_val) or not self.valid_tof(back_val):
            # 丢失 ToF 时不继续计算新方向，缓慢回中
            max_change = MAX_STEER_RATE_PER_SEC * dt
            change = self.clamp(
                -self.last_auto_steer,
                -max_change,
                max_change
            )
            self.last_auto_steer += change

            return {
                "valid": False,
                "auto_steer": self.last_auto_steer,
                "reason": "invalid ToF"
            }

        # 1. 对两颗 ToF 分别滤波
        self.filtered_right = self.low_pass(
            float(right_val),
            self.filtered_right,
            TOF_FILTER_ALPHA
        )
        self.filtered_back = self.low_pass(
            float(back_val),
            self.filtered_back,
            TOF_FILTER_ALPHA
        )

        right = self.filtered_right
        back = self.filtered_back

        # 2. 姿态角
        # diff > 0：车头比车尾离右墙更远，需要向右修正
        diff = right - back
        wall_angle_rad = math.atan2(diff, SENSOR_DIST)

        # 3. 车辆中心到墙面的垂直距离
        d_mid = (right + back) / 2.0
        d_perp = d_mid * math.cos(wall_angle_rad)

        # 4. 距离误差
        # > 0：离右墙太远，需要向右修正
        # < 0：离右墙太近，需要向左修正
        distance_error_cm = d_perp - TARGET_WALL_DIST
        distance_error_ctrl = self.apply_deadband(
            distance_error_cm,
            DIST_DEADBAND_CM
        )

        # 5. 姿态误差
        posture_error_rad = self.apply_deadband(
            wall_angle_rad,
            math.radians(POSTURE_DEADBAND_DEG)
        )

        # 6. 距离积分，只处理长期小偏差
        if abs(distance_error_ctrl) <= INTEGRAL_ACTIVE_RANGE_CM:
            self.distance_integral += distance_error_ctrl * dt
        else:
            self.distance_integral *= 0.95

        self.distance_integral = self.clamp(
            self.distance_integral,
            -INTEGRAL_LIMIT,
            INTEGRAL_LIMIT
        )

        # 7. 姿态修正项：优先让车身恢复与墙平行
        posture_term_rad = K_POSTURE * posture_error_rad

        # 8. Stanley 型距离修正项
        # TARGET_RPM 越高，同样距离误差对应的修正会更温和
        distance_term_rad = math.atan2(
            K_DIST * distance_error_ctrl,
            abs(float(target_rpm)) + RPM_SOFTENING
        )

        # 9. 很小的积分修正项
        integral_term_rad = KI_DIST * self.distance_integral

        # 10. 合成目标转向角
        desired_steer_rad = (
            posture_term_rad
            + distance_term_rad
            + integral_term_rad
        ) * STEER_SIGN

        # 11. 先限制到车辆机械最大转角
        max_steer_rad = math.radians(MAX_STEER_DEG)
        limited_steer_rad = self.clamp(
            desired_steer_rad,
            -max_steer_rad,
            max_steer_rad
        )

        # 12. 等比例映射为 -1.0 ~ +1.0
        target_auto_steer = limited_steer_rad / max_steer_rad

        # 13. 输出低通滤波
        filtered_auto_steer = self.low_pass(
            target_auto_steer,
            self.last_auto_steer,
            STEER_FILTER_ALPHA
        )

        # 14. 限制方向变化速度
        max_change = MAX_STEER_RATE_PER_SEC * dt
        steer_change = self.clamp(
            filtered_auto_steer - self.last_auto_steer,
            -max_change,
            max_change
        )

        auto_steer = self.last_auto_steer + steer_change
        auto_steer = self.clamp(auto_steer, -1.0, 1.0)
        self.last_auto_steer = auto_steer

        return {
            "valid": True,
            "auto_steer": auto_steer,
            "right_filtered": right,
            "back_filtered": back,
            "diff": diff,
            "wall_angle_deg": math.degrees(wall_angle_rad),
            "d_mid": d_mid,
            "d_perp": d_perp,
            "distance_error_cm": distance_error_cm,
            "posture_term_deg": math.degrees(posture_term_rad),
            "distance_term_deg": math.degrees(distance_term_rad),
            "integral_term_deg": math.degrees(integral_term_rad),
            "desired_steer_deg": math.degrees(desired_steer_rad),
            "target_auto_steer": target_auto_steer
        }


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
                self.normal_baseline = (
                    sum(list(self.history)[:-1])
                    / (self.history.maxlen - 1)
                )

            if (
                not self.is_inside_board
                and (self.normal_baseline - current_val) >= self.threshold
            ):
                self.is_inside_board = True

            elif (
                self.is_inside_board
                and (current_val - (self.normal_baseline - self.threshold)) > 5.0
            ):
                if current_val > (TARGET_WALL_DIST - 10.0):
                    self.is_inside_board = False
                    detected = True

        self.history.append(current_val)
        return detected


# ================= 辅助函数 =================
def to_uint32(value):
    value = int(value)
    if value < 0:
        value += 4294967296
    return value


def deg_to_unit(deg):
    return int(deg * 4096 / 360)



# ================= 串口共享数据 =================
class SharedToFData:
    """
    串口线程负责写入；主控线程只读取快照。

    使用 threading.Lock 保护整组数据，避免主控线程读取到
    “right_val 已更新、back_val 尚未更新”的半组数据。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._front_val = -1.0
        self._right_val = -1.0
        self._back_val = -1.0
        self._extra_val = -1.0
        self._timestamp = 0.0
        self._sequence = 0

    def update(self, front_val, right_val, back_val, extra_val):
        now = time.monotonic()

        # 锁只在复制几个变量时持有，时间非常短
        with self._lock:
            self._front_val = front_val
            self._right_val = right_val
            self._back_val = back_val
            self._extra_val = extra_val
            self._timestamp = now
            self._sequence += 1

    def snapshot(self):
        # 一次性复制完整快照，离开 with 后锁立即释放
        with self._lock:
            return {
                "front_val": self._front_val,
                "right_val": self._right_val,
                "back_val": self._back_val,
                "extra_val": self._extra_val,
                "timestamp": self._timestamp,
                "sequence": self._sequence,
            }


class ToFSerialReceiver(threading.Thread):
    """
    线程 1：只负责从串口接收数据并更新 SharedToFData。
    不进行转向计算，也不控制电机。
    """

    def __init__(self, serial_port, shared_data, stop_event):
        super().__init__(name="ToFSerialReceiver", daemon=True)
        self.serial_port = serial_port
        self.shared_data = shared_data
        self.stop_event = stop_event
        self.buffer = ""
        self.error = None

    def run(self):
        try:
            while not self.stop_event.is_set():
                n = self.serial_port.in_waiting

                if n <= 0:
                    # 避免无数据时占满一个 CPU 核心
                    time.sleep(0.001)
                    continue

                chunk = self.serial_port.read(n).decode(
                    "utf-8",
                    errors="ignore"
                )

                if not chunk:
                    continue

                self.buffer += chunk
                *lines, self.buffer = self.buffer.split("\n")

                # 只保留本批数据中最后一组完整数据
                latest_parts = None

                for line in lines:
                    line = line.strip()

                    if not line or line.startswith("#"):
                        continue

                    parts = line.split(",")

                    if len(parts) == 5:
                        latest_parts = parts

                if latest_parts is None:
                    continue

                try:
                    front_val = float(latest_parts[1])
                    right_val = float(latest_parts[2])
                    back_val = float(latest_parts[3])
                    extra_val = float(latest_parts[4])
                except ValueError:
                    continue

                # 四个值作为同一个原子快照写入
                self.shared_data.update(
                    front_val,
                    right_val,
                    back_val,
                    extra_val
                )

        except Exception as exc:
            self.error = exc
            self.stop_event.set()


# ================= 主程序 =================
def main():
    try:
        # timeout 不能太大，否则结束程序时串口线程退出会变慢
        ser_tof = serial.Serial(
            TOF_PORT,
            TOF_BAUD,
            timeout=0.02
        )
        print("Opened ToF Sensor at", ser_tof.name)
    except Exception as e:
        print(f"Failed to open ToF port: {e}")
        sys.exit()

    port = PortHandler(DYNAMIXEL_PORT)
    packet = PacketHandler(2.0)

    if not port.openPort() or not port.setBaudRate(DYNAMIXEL_BAUD):
        print("Failed to open Dynamixel port.")
        ser_tof.close()
        sys.exit()

    packet.write1ByteTxRx(
        port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF
    )
    packet.write1ByteTxRx(
        port, ID_THROTTLE, ADDR_MODE, VELOCITY_MODE
    )
    packet.write4ByteTxRx(
        port, ID_THROTTLE, ADDR_PROFILE_ACCEL, 100
    )
    packet.write1ByteTxRx(
        port, ID_THROTTLE, ADDR_TORQUE, TORQUE_ON
    )

    packet.write1ByteTxRx(
        port, ID_STEER, ADDR_TORQUE, TORQUE_OFF
    )
    packet.write1ByteTxRx(
        port, ID_STEER, ADDR_MODE, POSITION_MODE
    )
    packet.write4ByteTxRx(
        port, ID_STEER, ADDR_PROFILE_ACCEL, 100
    )
    packet.write4ByteTxRx(
        port, ID_STEER, ADDR_PROFILE_VELOCITY, 500
    )
    packet.write1ByteTxRx(
        port, ID_STEER, ADDR_TORQUE, TORQUE_ON
    )

    zero_pos = 2020
    detector = BoardDetector(threshold=15.0)
    wall_follower = TwoToFWallFollower()

    # 先发送复位命令，再启动串口接收线程
    ser_tof.write(b"reset gyro\n")
    print("[PY] reset gyro sent")
    time.sleep(3)

    shared_tof = SharedToFData()
    stop_event = threading.Event()

    receiver = ToFSerialReceiver(
        serial_port=ser_tof,
        shared_data=shared_tof,
        stop_event=stop_event
    )
    receiver.start()

    last_vel = None
    last_pos = None
    last_sensor_sequence = -1

    auto_steer = 0.0
    d_perp = 0.0
    wall_angle_deg = 0.0
    right_val = -1.0
    back_val = -1.0

    # 主控循环固定为约 100 Hz。
    # 串口线程独立接收，所以这里不会等待串口。
    CONTROL_PERIOD = 0.010

    # 超过这个时间没有新 ToF 数据，认为传感器数据已过期
    SENSOR_STALE_TIMEOUT = 0.30

    print(
        f"\nStarting two-thread autonomous control... "
        f"(Max {MAX_RUN_SECONDS:.1f} seconds)"
    )

    program_start_time = time.time()
    next_control_time = time.monotonic()

    try:
        while True:
            now_monotonic = time.monotonic()
            elapsed_time = time.time() - program_start_time

            if elapsed_time >= MAX_RUN_SECONDS:
                print(
                    f"\n\n[INFO] 已自动行驶满 "
                    f"{MAX_RUN_SECONDS:.1f} 秒，自动停车退出。"
                )
                break

            if receiver.error is not None:
                print(
                    f"\n[ERROR] ToF 串口接收线程异常："
                    f"{receiver.error}"
                )
                break

            target_rpm = TARGET_RPM
            is_moving = abs(target_rpm) > 0.01
            vel_val = int(target_rpm / VELOCITY_UNIT_RPM)

            # 主控线程只读取一份受锁保护的完整快照
            tof = shared_tof.snapshot()

            sensor_age = (
                now_monotonic - tof["timestamp"]
                if tof["timestamp"] > 0.0
                else float("inf")
            )

            has_new_sample = (
                tof["sequence"] != last_sensor_sequence
            )

            if has_new_sample:
                last_sensor_sequence = tof["sequence"]
                right_val = tof["right_val"]
                back_val = tof["back_val"]

                right_valid = (
                    right_val != -1
                    and MIN_VALID_TOF_CM
                    <= right_val
                    <= MAX_VALID_TOF_CM
                )
                back_valid = (
                    back_val != -1
                    and MIN_VALID_TOF_CM
                    <= back_val
                    <= MAX_VALID_TOF_CM
                )

                # 板检测只对每一组新数据执行一次
                if (
                    right_valid
                    and detector.update(right_val, is_moving)
                ):
                    print("\n\n" + "=" * 45)
                    print(
                        f"【核心提示】在第 "
                        f"{elapsed_time:.1f} 秒检测到垂直板子！"
                        f"车辆立即停止。"
                    )
                    print("=" * 45)
                    break

                # 沿墙控制也只对新传感器数据计算一次。
                # 两组数据之间，主控线程继续使用上一次的 auto_steer。
                result = wall_follower.update(
                    right_val=right_val,
                    back_val=back_val,
                    target_rpm=target_rpm,
                    now=now_monotonic
                )

                auto_steer = result["auto_steer"]

                if result["valid"]:
                    d_perp = result["d_perp"]
                    wall_angle_deg = result["wall_angle_deg"]

            elif sensor_age > SENSOR_STALE_TIMEOUT:
                # 数据长时间未更新时，逐步让转向回中，避免一直保留旧方向
                dt = CONTROL_PERIOD
                max_change = MAX_STEER_RATE_PER_SEC * dt

                if auto_steer > max_change:
                    auto_steer -= max_change
                elif auto_steer < -max_change:
                    auto_steer += max_change
                else:
                    auto_steer = 0.0

            # ================= 发送电机指令 =================
            steer_deg = auto_steer * MAX_STEER_DEG
            pos_val = int(
                zero_pos + deg_to_unit(steer_deg)
            )

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

            age_text = (
                f"{sensor_age * 1000:4.0f}ms"
                if math.isfinite(sensor_age)
                else "----"
            )

            print(
                f"时间:{elapsed_time:6.1f}s/"
                f"{MAX_RUN_SECONDS:.1f}s | "
                f"垂距:{d_perp:5.1f}cm | "
                f"姿态:{wall_angle_deg:+5.1f}° | "
                f"Steer:{auto_steer:+5.2f} | "
                f"R:{right_val:5.1f} "
                f"B:{back_val:5.1f} | "
                f"Age:{age_text}   ",
                end="\r"
            )

            # 固定主控周期，不等待串口
            next_control_time += CONTROL_PERIOD
            sleep_time = next_control_time - time.monotonic()

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # 如果某一轮超时，重新对齐，避免持续追赶
                next_control_time = time.monotonic()

    except KeyboardInterrupt:
        print("\nShutting down safely (Ctrl+C)...")

    finally:
        # 先通知串口线程退出
        stop_event.set()
        receiver.join(timeout=1.0)

        packet.write4ByteTxRx(
            port,
            ID_THROTTLE,
            ADDR_GOAL_VELOCITY,
            0
        )
        packet.write4ByteTxRx(
            port,
            ID_STEER,
            ADDR_GOAL_POSITION,
            zero_pos
        )

        time.sleep(0.5)

        packet.write1ByteTxRx(
            port,
            ID_THROTTLE,
            ADDR_TORQUE,
            TORQUE_OFF
        )
        packet.write1ByteTxRx(
            port,
            ID_STEER,
            ADDR_TORQUE,
            TORQUE_OFF
        )

        port.closePort()
        ser_tof.close()


if __name__ == "__main__":
    main()
