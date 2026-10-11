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
ADDR_HARDWARE_ERROR = 70
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112

VELOCITY_MODE = 1
POSITION_MODE = 3
TORQUE_ON = 1
TORQUE_OFF = 0

# Dynamixel Protocol 2.0 Alert Bit。电机进入硬件保护时，状态包仍可读取，
# 但dxl_error会带0x80；恢复流程必须允许它，才能读取地址70并执行Reboot。
DXL_ALERT = 0x80

# ================= DYNAMIXEL 启动检测与恢复 =================
# Reboot 后给电机重新启动和恢复通信的等待时间
DYNAMIXEL_REBOOT_WAIT_SEC = 1.0

# 启动恢复最多尝试次数
DYNAMIXEL_RECOVERY_RETRIES = 2

# 所有功能共用的100%速度基准。
# 各功能传入speed=-100～+100，并分别换算自己的目标RPM。
# 因为这里设为100，所以数值上speed=10约等于10RPM，speed=40约等于40RPM。
TARGET_RPM = 100.0
VELOCITY_UNIT_RPM = 0.229

# auto_steer = ±1.0 时对应的最大转向角
MAX_STEER_DEG = 50.0

# ================= Run for Time 参数 =================
# run_for_time() 中 speed=±100 时对应的实际驱动电机转速。
# 当前100%对应100 RPM。
RUN_FOR_TIME_MAX_RPM = TARGET_RPM

# ================= Run for Distance 参数 =================
# 驱动电机转动一整圈对应的 Dynamixel Position 单位数。
DXL_POSITION_UNITS_PER_REV = 4096

# LEGO 轮胎 62.4×20：直径62.4 mm=6.24 cm；20 mm宽度不参与计算。
WHEEL_DIAMETER_CM = 6.24

# 三级加速：驱动电机转1圈，车轮转3圈。
WHEEL_REVS_PER_MOTOR_REV = 3.0

# 实车距离校准：命令20.0cm时实际行驶18.5cm。
# 因此实际距离是理论距离的18.5/20.0=0.925倍。
DISTANCE_CALIBRATION_FACTOR = 18.5 / 20.0

# 电机每变化1个Position单位，对应的校准后行驶距离（cm）。
DISTANCE_CM_PER_MOTOR_UNIT = (
    math.pi * WHEEL_DIAMETER_CM * WHEEL_REVS_PER_MOTOR_REV
    / DXL_POSITION_UNITS_PER_REV
    * DISTANCE_CALIBRATION_FACTOR
)

# 读取电机位置并累计距离的周期，理论频率约100 Hz。
RUN_FOR_DISTANCE_CONTROL_PERIOD = 0.010

# 电机有速度指令但位置长时间完全不变化时，安全停车并报错。
RUN_FOR_DISTANCE_STALL_TIMEOUT_SEC = 2.0

# runForDistanceByToF中允许使用的ToF数据最大更新时间。
RUN_FOR_DISTANCE_TOF_STALE_TIMEOUT_SEC = 0.30

# ================= 运行时间设置 =================
MAX_RUN_SECONDS = 300.0

# ================= 两颗 ToF 纯 P 沿墙控制参数 =================
SENSOR_DIST = 15.0       # 两颗右侧 ToF 测量中心的前后间距 side，单位 cm
TARGET_WALL_DIST = 30.0  # 目标垂直墙距 TD，单位 cm

# 两个误差先分别归一化，再进行加权。
# 当姿态角达到 ±15° 时，姿态归一化误差达到 ±1。
MAX_ANGLE_ERROR_DEG = 15.0

# 当墙距误差达到 ±15 cm 时，距离归一化误差达到 ±1。
MAX_DISTANCE_ERROR_CM = 15.0

# 纯 P 控制权重：姿态优先，距离其次。
K_ANGLE = 0.4
K_DISTANCE = 1

# ToF 一阶低通滤波。越大反应越快，越小越平滑。
TOF_FILTER_ALPHA = 0.8

# 最终转向输出低通滤波；这只是信号滤波，不是 PID 的 D 项。
STEER_FILTER_ALPHA = 0.85

# 小误差死区，用于减少 ToF 噪声导致的方向抖动。
DIST_DEADBAND_CM = 0.6
ANGLE_DEADBAND_DEG = 0.7

# auto_steer 每秒最大变化量；这是执行器保护，不是 PID 的 D 项。
MAX_STEER_RATE_PER_SEC = 8

# 如果实车的正负转向方向相反，只把这里改成 -1.0。
STEER_SIGN = 1.0

# ToF 有效范围
MIN_VALID_TOF_CM = 2.0

# 传感器超过有效量程后返回 -1。
# 沿墙控制中把 -1 解释为“距离至少达到 50 cm”，
# 因此用 50 cm 作为饱和值，而不是把该帧直接判为无效。
TOF_FAR_VALUE_CM = 50.0
MAX_VALID_TOF_CM = 50.0

# ================= Parking lot 垂直板检测参数 =================
# 正常沿墙距离突然减少至少这么多，才可能是停车区垂直板。
BOARD_DROP_THRESHOLD_CM = 11.0

# 板子读数本身必须小于等于这个距离，避免把普通墙距波动误判为板子。
BOARD_MAX_DISTANCE_CM = 22.0

# 右前、右后两颗 ToF 不必在同一帧看到板子；任意一颗先看到后，
# 另一颗必须在这个时间窗口内也看到，才确认停车。
BOARD_CONFIRM_TIMEOUT_SEC = 4.0

# 每颗 ToF 独立保存最近几帧正常墙距，用作动态基准。
BOARD_BASELINE_HISTORY_LEN = 5

# 传感器连续恢复多少帧后，才认为已经完全离开1.5 cm厚的停车板。
BOARD_RECOVERY_FRAMES = 2

# ================= 启动时车身与右墙平行校正 =================
# True：串口接收线程启动后，先执行一次姿态校正，再进入原主程序。
AUTO_ALIGN_AT_START = True

# main()中的默认功能速度；各函数调用时可以分别覆盖。
DEFAULT_ALIGN_SPEED = 3.0
DEFAULT_PARKING_LOT_SPEED = 10.0

# 右前、右后距离之差不超过该值，认为车身已经与墙平行。
ALIGN_TOLERANCE_CM = 0.8

# 连续多少组新 ToF 数据都在允许误差内，才正式完成校正。
ALIGN_STABLE_SAMPLES = 3

# 校正最大持续时间；超时后一定停车并将转向回中。
ALIGN_TIMEOUT_SEC = 15.0
ALIGN_SENSOR_STALE_TIMEOUT_SEC = 0.30

# 校正转向范围。auto_steer=0.20/0.45 分别约为10°/22.5°。
ALIGN_MIN_STEER_AUTO = 1.00
ALIGN_MAX_STEER_AUTO = 1.00

# 当前后距离差达到该值时，使用最大校正转向。
ALIGN_FULL_STEER_ERROR_CM = 10.0

# R、B同时为-1时，先最大向右转并向前推动这么长时间。
ALIGN_BOTH_FAR_RIGHT_TURN_SEC = 0.50
ALIGN_BOTH_FAR_RIGHT_STEER_AUTO = 1.00

# 搜索另一颗ToF数据的最长时间和防撞距离。
ALIGN_SEARCH_TIMEOUT_SEC = 8.0
ALIGN_SEARCH_MIN_SAFE_CM = 8.0


# ================= 两颗 ToF 纯 P 沿墙控制器 =================
class TwoToFWallFollower:
    """
    几何定义：
        a = right_val：右前 ToF
        b = back_val ：右后 ToF
        side = SENSOR_DIST

        angle1 = atan2(a - b, side)
        d = (a + b) / 2
        h = cos(angle1) * d

        error1 = angle1，目标为 0°
        error2 = h - TARGET_WALL_DIST

    控制公式：
        steering = K_ANGLE * normalized(error1)
                 + K_DISTANCE * normalized(error2)

    本控制器只有 P 控制，没有 I 项，也没有 D 项。
    """

    def __init__(self):
        self.filtered_right = None
        self.filtered_back = None
        self.last_auto_steer = 0.0
        self.last_update_time = None

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    @staticmethod
    def apply_deadband(value, deadband):
        """
        连续死区：死区内输出 0，越过死区后连续增长。
        """
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
        self.last_auto_steer = 0.0
        self.last_update_time = None

    def update(self, right_val, back_val, target_rpm, now=None):
        """
        target_rpm 参数为保留接口，纯 P 公式本身不使用车速。
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
            # 数据无效时逐步回中，不继续使用错误数据计算。
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

        # 1. 两颗 ToF 分别滤波
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

        a = self.filtered_right
        b = self.filtered_back

        # 2. angle1：车辆与墙的偏角
        # a > b 时 angle1 > 0，表示车头比车尾离右墙更远。
        diff = a - b
        angle1_rad = math.atan2(diff, SENSOR_DIST)
        angle1_deg = math.degrees(angle1_rad)

        # 3. d：两颗传感器距离的平均值
        d = (a + b) / 2.0

        # 4. h：车辆中线位置到墙面的垂直距离
        h = math.cos(angle1_rad) * d

        # 5. 两个原始误差
        error1_deg = angle1_deg
        error2_cm = h - TARGET_WALL_DIST

        # 6. 对小误差设置死区
        error1_ctrl_deg = self.apply_deadband(
            error1_deg,
            ANGLE_DEADBAND_DEG
        )
        error2_ctrl_cm = self.apply_deadband(
            error2_cm,
            DIST_DEADBAND_CM
        )

        # 7. 两个误差分别归一化到 -1 ~ +1
        error1_norm = self.clamp(
            error1_ctrl_deg / MAX_ANGLE_ERROR_DEG,
            -1.0,
            1.0
        )
        error2_norm = self.clamp(
            error2_ctrl_cm / MAX_DISTANCE_ERROR_CM,
            -1.0,
            1.0
        )

        # 8. 纯 P 控制
        angle_term = K_ANGLE * error1_norm
        distance_term = K_DISTANCE * error2_norm

        steering_raw = (
            angle_term
            + distance_term
        ) * STEER_SIGN

        # 因为默认 K_ANGLE + K_DISTANCE = 1，
        # 正常情况下 steering_raw 已在 -1 ~ +1 内。
        # 此处再限幅仅作为安全保护。
        target_auto_steer = self.clamp(
            steering_raw,
            -1.0,
            1.0
        )

        # 9. 转向输出低通滤波
        filtered_auto_steer = self.low_pass(
            target_auto_steer,
            self.last_auto_steer,
            STEER_FILTER_ALPHA
        )

        # 10. 限制每秒最大转向变化量
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

            "right_filtered": a,
            "back_filtered": b,
            "diff": diff,

            "angle1_rad": angle1_rad,
            "angle1_deg": angle1_deg,
            "wall_angle_deg": angle1_deg,

            "d_mid": d,
            "d_perp": h,
            "h_cm": h,

            "error1_deg": error1_deg,
            "error2_cm": error2_cm,
            "error1_norm": error1_norm,
            "error2_norm": error2_norm,

            "angle_term": angle_term,
            "distance_term": distance_term,
            "steering_raw": steering_raw,
            "target_auto_steer": target_auto_steer
        }


# ================= Parking lot 垂直板检测逻辑 =================
class ParkingBoardDetector:
    """
    右前和右后 ToF 分别保存正常墙距基准。

    状态顺序：
        1. 等待右前看到板子。
        2. 右前看到板子期间强制直行；右前恢复后重新启用沿墙控制。
        3. 等待右后看到同一块板子。
        4. 右后看到板子期间强制直行；右后恢复后确认并停车。

    8～12 cm 的特殊近距离只用于停车板检测，绝不进入沿墙控制。
    """

    def __init__(self):
        self.right_history = deque(maxlen=BOARD_BASELINE_HISTORY_LEN)
        self.back_history = deque(maxlen=BOARD_BASELINE_HISTORY_LEN)
        self.reset_candidate()

    def reset_candidate(self):
        self.state = "WAIT_FRONT"
        self.wait_back_start_time = None
        self.right_board_value = None
        self.back_board_value = None
        self.right_board_baseline = None
        self.back_board_baseline = None
        self.recovery_count = 0

    @staticmethod
    def valid_raw(value):
        return (
            value != -1
            and MIN_VALID_TOF_CM <= value <= TOF_FAR_VALUE_CM
        )

    @staticmethod
    def baseline(history):
        if len(history) < history.maxlen:
            return None
        return sum(history) / len(history)

    @staticmethod
    def is_special(value, baseline):
        return (
            baseline is not None
            and value <= BOARD_MAX_DISTANCE_CM
            and (baseline - value) >= BOARD_DROP_THRESHOLD_CM
        )

    def update(self, right_raw, back_raw, is_moving, now):
        if not is_moving:
            self.right_history.clear()
            self.back_history.clear()
            self.reset_candidate()
            return {
                "detected": False,
                "control_right": right_raw,
                "control_back": back_raw,
                "right_new": False,
                "back_new": False,
                "right_recovered": False,
                "back_recovered": False,
                "force_straight": False,
            }

        right_baseline = self.baseline(self.right_history)
        back_baseline = self.baseline(self.back_history)

        right_valid = self.valid_raw(right_raw)
        back_valid = self.valid_raw(back_raw)
        right_special = (
            right_valid and self.is_special(right_raw, right_baseline)
        )
        back_special = (
            back_valid and self.is_special(back_raw, back_baseline)
        )

        right_new = False
        back_new = False
        right_recovered = False
        back_recovered = False
        detected = False
        force_straight = False

        if self.state == "WAIT_FRONT":
            if right_special:
                right_new = True
                force_straight = True
                self.state = "FRONT_ACTIVE"
                self.recovery_count = 0
                self.right_board_value = right_raw
                self.right_board_baseline = right_baseline

        elif self.state == "FRONT_ACTIVE":
            force_straight = True
            if right_valid and not right_special:
                self.recovery_count += 1
                if self.recovery_count >= BOARD_RECOVERY_FRAMES:
                    right_recovered = True
                    force_straight = False
                    self.state = "WAIT_BACK"
                    self.wait_back_start_time = now
                    self.recovery_count = 0
            else:
                self.recovery_count = 0

        elif self.state == "WAIT_BACK":
            if (
                now - self.wait_back_start_time
                > BOARD_CONFIRM_TIMEOUT_SEC
            ):
                self.reset_candidate()
            elif back_special:
                back_new = True
                force_straight = True
                self.state = "BACK_ACTIVE"
                self.recovery_count = 0
                self.back_board_value = back_raw
                self.back_board_baseline = back_baseline

        elif self.state == "BACK_ACTIVE":
            force_straight = True
            if back_valid and not back_special:
                self.recovery_count += 1
                if self.recovery_count >= BOARD_RECOVERY_FRAMES:
                    back_recovered = True
                    detected = True
                    force_straight = True
                    self.state = "DONE"
            else:
                self.recovery_count = 0

        elif self.state == "DONE":
            detected = True
            force_straight = True

        if right_new:
            self.right_board_value = right_raw
            self.right_board_baseline = right_baseline

        if back_new:
            self.back_board_value = back_raw
            self.back_board_baseline = back_baseline

        # 只有正常读数才能更新墙距基准；特殊板子读数绝不污染基准。
        if right_valid and not right_special:
            self.right_history.append(float(right_raw))
        if back_valid and not back_special:
            self.back_history.append(float(back_raw))

        # 板子特殊读数只用于检测。沿墙控制继续使用该传感器最近的正常基准。
        control_right = (
            right_baseline if right_special and right_baseline is not None
            else right_raw
        )
        control_back = (
            back_baseline if back_special and back_baseline is not None
            else back_raw
        )

        return {
            "detected": detected,
            "control_right": control_right,
            "control_back": control_back,
            "right_new": right_new,
            "back_new": back_new,
            "right_recovered": right_recovered,
            "back_recovered": back_recovered,
            "force_straight": force_straight,
            "state": self.state,
            "right_board_value": self.right_board_value,
            "back_board_value": self.back_board_value,
            "right_board_baseline": self.right_board_baseline,
            "back_board_baseline": self.back_board_baseline,
        }


# ================= 辅助函数 =================
def to_uint32(value):
    value = int(value)
    if value < 0:
        value += 4294967296
    return value


def deg_to_unit(deg):
    return int(deg * 4096 / 360)




# ================= DYNAMIXEL 检测、恢复与初始化 =================
def decode_dxl_hardware_error(hardware_error):
    errors = []
    if hardware_error & 0x01:
        errors.append("输入电压错误")
    if hardware_error & 0x04:
        errors.append("电机过热")
    if hardware_error & 0x08:
        errors.append("编码器错误")
    if hardware_error & 0x10:
        errors.append("电击/供电不足")
    if hardware_error & 0x20:
        errors.append("过载/堵转")
    return errors if errors else ["无硬件错误"]


def dxl_comm_ok(
    packet,
    comm_result,
    dxl_error,
    action,
    dxl_id,
    allow_alert=False,
):
    """
    检查一次 DYNAMIXEL SDK 通信结果。
    """
    if comm_result != COMM_SUCCESS:
        print(
            f"[DXL ID {dxl_id}] {action} 通信失败："
            f"{packet.getTxRxResult(comm_result)}"
        )
        return False

    if dxl_error != 0:
        # 0x80表示电机返回了有效状态包，同时提示检查Hardware Error。
        # 恢复阶段不能把它当成通信失败，否则无法读取错误并Reboot。
        if allow_alert and (dxl_error & DXL_ALERT):
            return True

        print(
            f"[DXL ID {dxl_id}] {action} 状态包错误："
            f"{packet.getRxPacketError(dxl_error)} "
            f"(0x{dxl_error:02X})"
        )
        return False

    return True


def read_dxl_byte(
    port,
    packet,
    dxl_id,
    address,
    name,
    allow_alert=False,
):
    value, comm_result, dxl_error = packet.read1ByteTxRx(
        port,
        dxl_id,
        address
    )

    if not dxl_comm_ok(
        packet,
        comm_result,
        dxl_error,
        f"读取 {name}",
        dxl_id,
        allow_alert=allow_alert,
    ):
        return None

    return value


def write_dxl_byte(
    port,
    packet,
    dxl_id,
    address,
    value,
    name,
    allow_alert=False,
):
    comm_result, dxl_error = packet.write1ByteTxRx(
        port,
        dxl_id,
        address,
        value
    )

    return dxl_comm_ok(
        packet,
        comm_result,
        dxl_error,
        f"写入 {name}={value}",
        dxl_id,
        allow_alert=allow_alert,
    )


def write_dxl_dword(
    port,
    packet,
    dxl_id,
    address,
    value,
    name,
    allow_alert=False,
):
    comm_result, dxl_error = packet.write4ByteTxRx(
        port,
        dxl_id,
        address,
        value
    )

    return dxl_comm_ok(
        packet,
        comm_result,
        dxl_error,
        f"写入 {name}={value}",
        dxl_id,
        allow_alert=allow_alert,
    )


def reboot_dynamixel(port, packet, dxl_id, motor_name):
    """
    对发生 Shutdown/Hardware Error 的电机发送 Protocol 2.0 Reboot。
    """
    print(
        f"[RECOVERY] {motor_name}（DXL ID {dxl_id}）"
        "检测到硬件保护，正在发送Reboot……"
    )

    comm_result, dxl_error = packet.reboot(
        port,
        dxl_id
    )

    if not dxl_comm_ok(
        packet,
        comm_result,
        dxl_error,
        "Reboot",
        dxl_id,
        allow_alert=True,
    ):
        return False

    time.sleep(DYNAMIXEL_REBOOT_WAIT_SEC)
    print(f"[RECOVERY] {motor_name} Reboot完成。")
    return True


def recover_and_configure_dynamixel(
    port,
    packet,
    dxl_id,
    operating_mode,
    profile_accel,
    profile_velocity=None,
    motor_name="Dynamixel电机",
):
    """
    启动前执行：
      1. 读取 Hardware Error Status(70) 和 Torque Enable(64)
      2. 若存在硬件错误，发送 Reboot
      3. 若只是 Torque OFF，则直接重新配置并 Torque ON
      4. 设置工作模式和 Profile
      5. 再次读取，确认 Hardware Error=0 且 Torque Enable=1

    返回 True 表示电机可以进入主控程序；
    返回 False 表示恢复失败，主程序应停止，防止车辆失控。
    """

    for attempt in range(1, DYNAMIXEL_RECOVERY_RETRIES + 1):
        print(
            f"\n[STARTUP CHECK] {motor_name}（DXL ID {dxl_id}）"
            f"({attempt}/{DYNAMIXEL_RECOVERY_RETRIES})"
        )

        hardware_error = read_dxl_byte(
            port,
            packet,
            dxl_id,
            ADDR_HARDWARE_ERROR,
            "Hardware Error Status",
            allow_alert=True,
        )
        torque_state = read_dxl_byte(
            port,
            packet,
            dxl_id,
            ADDR_TORQUE,
            "Torque Enable",
            allow_alert=True,
        )

        if hardware_error is None or torque_state is None:
            print(f"[ERROR] 无法读取{motor_name}的状态。")
            time.sleep(0.2)
            continue

        print(
            f"[STATUS] {motor_name}（ID {dxl_id}）"
            f"Hardware Error=0x{hardware_error:02X}, "
            f"Torque Enable={torque_state}, "
            f"错误说明={decode_dxl_hardware_error(hardware_error)}"
        )

        # Shutdown 后仅写 Torque ON 通常不能解除锁定，必须 Reboot。
        if hardware_error != 0:
            print(
                f"[WARNING] 出问题的是：{motor_name}（ID {dxl_id}）。"
            )
            if not reboot_dynamixel(port, packet, dxl_id, motor_name):
                continue

            # Reboot 后重新读取，确认错误条件已经消失
            hardware_error = read_dxl_byte(
                port,
                packet,
                dxl_id,
                ADDR_HARDWARE_ERROR,
                "Reboot 后 Hardware Error Status",
                allow_alert=True,
            )

            if hardware_error is None:
                continue

            if hardware_error != 0:
                print(
                    f"[FATAL] {motor_name} Reboot后硬件错误仍存在："
                    f"0x{hardware_error:02X}，"
                    f"{decode_dxl_hardware_error(hardware_error)}。"
                )
                print(
                    "[SAFETY] 可能仍有过流、过热、电压或机械堵转问题，"
                    "不能强行启动。"
                )
                time.sleep(0.2)
                continue

            print(f"[RECOVERY] {motor_name}硬件错误已经清除。")
        else:
            print(f"[OK] {motor_name}没有硬件错误，无需Reboot。")

        # 改工作模式前必须 Torque OFF
        if not write_dxl_byte(
            port,
            packet,
            dxl_id,
            ADDR_TORQUE,
            TORQUE_OFF,
            "Torque Enable",
            allow_alert=True,
        ):
            continue

        if not write_dxl_byte(
            port,
            packet,
            dxl_id,
            ADDR_MODE,
            operating_mode,
            "Operating Mode",
            allow_alert=True,
        ):
            continue

        if not write_dxl_dword(
            port,
            packet,
            dxl_id,
            ADDR_PROFILE_ACCEL,
            profile_accel,
            "Profile Acceleration",
            allow_alert=True,
        ):
            continue

        if profile_velocity is not None:
            if not write_dxl_dword(
                port,
                packet,
                dxl_id,
                ADDR_PROFILE_VELOCITY,
                profile_velocity,
                "Profile Velocity",
                allow_alert=True,
            ):
                continue

        # 驱动电机重新激活前，先明确写入速度0，防止Torque On后突然运动。
        if operating_mode == VELOCITY_MODE:
            if not write_dxl_dword(
                port,
                packet,
                dxl_id,
                ADDR_GOAL_VELOCITY,
                0,
                "Goal Velocity安全归零",
                allow_alert=True,
            ):
                continue

        if not write_dxl_byte(
            port,
            packet,
            dxl_id,
            ADDR_TORQUE,
            TORQUE_ON,
            "Torque Enable",
            allow_alert=True,
        ):
            continue

        time.sleep(0.1)

        final_error = read_dxl_byte(
            port,
            packet,
            dxl_id,
            ADDR_HARDWARE_ERROR,
            "最终 Hardware Error Status",
            allow_alert=True,
        )
        final_torque = read_dxl_byte(
            port,
            packet,
            dxl_id,
            ADDR_TORQUE,
            "最终 Torque Enable",
            allow_alert=True,
        )

        if final_error == 0 and final_torque == TORQUE_ON:
            print(
                f"[RECOVERY OK] {motor_name}（ID {dxl_id}）检测通过，"
                "已重新配置并Torque On。"
            )
            return True

        print(
            f"[RECOVERY FAILED] {motor_name}（ID {dxl_id}）检测未通过："
            f"Hardware Error={final_error}, "
            f"Torque Enable={final_torque}"
        )

    print(
        f"\n[SAFETY] {motor_name}（DYNAMIXEL ID {dxl_id}）自动恢复失败，"
        "主程序不会启动。"
    )
    return False


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


# ================= 车身与右墙平行校正子程序 =================
def align_parallel_to_right_wall(
    port,
    packet,
    shared_tof,
    receiver,
    zero_pos,
    speed,
):
    """
    先处理ToF无数据情况，再根据右前、右后距离差调整车身姿态。

    R=-1、B=-1：最大右转并低速前推0.5秒，再回中低速直行找墙。
    R有效、B=-1：回中低速前进，直到两颗都有效。
    R=-1、B有效：回中低速倒退，直到两颗都有效。
    R、B都有效：根据前后差使用最大左/右转向进行平行校正。
    两者接近相等：立即停止驱动；连续确认后转向回到 zero_pos。

    返回 True 表示校正成功，False 表示超时、串口异常或数据失联。
    无论如何退出，本函数都会停止驱动并将转向回中。
    """
    speed = float(speed)
    if not math.isfinite(speed) or not 0.0 < speed <= 100.0:
        raise ValueError("对墙校正speed必须是大于0、且不超过100的数值")

    align_rpm = speed / 100.0 * TARGET_RPM

    print(
        "\n[ALIGN] 开始校正车身姿态，使车辆与右墙平行。"
        f" speed={speed:.1f}%（约{align_rpm:.1f}RPM）"
    )

    start_time = time.monotonic()
    last_sequence = -1
    stable_count = 0
    last_steer_pos = None
    last_velocity = None
    success = False
    search_start_time = None
    both_far_turn_end = None

    align_velocity = int(align_rpm / VELOCITY_UNIT_RPM)
    search_velocity = align_velocity

    def set_velocity(signed_velocity):
        nonlocal last_velocity
        if signed_velocity != last_velocity:
            packet.write4ByteTxRx(
                port,
                ID_THROTTLE,
                ADDR_GOAL_VELOCITY,
                to_uint32(signed_velocity)
            )
            last_velocity = signed_velocity

    def set_steer_auto(auto_steer):
        nonlocal last_steer_pos
        steer_deg = auto_steer * MAX_STEER_DEG
        steer_pos = int(zero_pos + deg_to_unit(steer_deg))
        if steer_pos != last_steer_pos:
            packet.write4ByteTxRx(
                port,
                ID_STEER,
                ADDR_GOAL_POSITION,
                steer_pos
            )
            last_steer_pos = steer_pos
        return steer_deg

    # 每次进入本函数，都先取消主程序残留的速度和转向。
    set_velocity(0)
    set_steer_auto(0.0)
    time.sleep(0.20)

    try:
        while time.monotonic() - start_time < ALIGN_TIMEOUT_SEC:
            if receiver.error is not None:
                print(f"\n[ALIGN] ToF串口异常：{receiver.error}")
                break

            tof = shared_tof.snapshot()
            now = time.monotonic()
            sensor_age = (
                now - tof["timestamp"]
                if tof["timestamp"] > 0.0
                else float("inf")
            )

            if sensor_age > ALIGN_SENSOR_STALE_TIMEOUT_SEC:
                if last_velocity != 0:
                    packet.write4ByteTxRx(
                        port, ID_THROTTLE,
                        ADDR_GOAL_VELOCITY, 0
                    )
                    last_velocity = 0
                time.sleep(0.01)
                continue

            if tof["sequence"] == last_sequence:
                time.sleep(0.005)
                continue

            last_sequence = tof["sequence"]
            right = tof["right_val"]
            back = tof["back_val"]

            right_valid = (
                right != -1
                and MIN_VALID_TOF_CM <= right <= TOF_FAR_VALUE_CM
            )
            back_valid = (
                back != -1
                and MIN_VALID_TOF_CM <= back <= TOF_FAR_VALUE_CM
            )

            if not right_valid or not back_valid:
                stable_count = 0

                if search_start_time is None:
                    search_start_time = now

                if now - search_start_time > ALIGN_SEARCH_TIMEOUT_SEC:
                    print(
                        "\n[ALIGN][SAFETY] 搜索墙面数据超时，"
                        "放弃本次姿态校正。"
                    )
                    break

                if not right_valid and not back_valid:
                    # 两颗都为-1：先让车头朝右墙方向偏转0.5秒。
                    if both_far_turn_end is None:
                        both_far_turn_end = (
                            now + ALIGN_BOTH_FAR_RIGHT_TURN_SEC
                        )
                        steer_deg = set_steer_auto(
                            STEER_SIGN
                            * ALIGN_BOTH_FAR_RIGHT_STEER_AUTO
                        )
                        set_velocity(search_velocity)
                        print(
                            f"\n[ALIGN] R=-1、B=-1：向右转"
                            f"{steer_deg:+.1f}°并以"
                            f"{align_rpm:.1f}RPM前进"
                            f"{ALIGN_BOTH_FAR_RIGHT_TURN_SEC:.2f}秒。"
                        )
                    elif now >= both_far_turn_end:
                        set_steer_auto(0.0)
                        set_velocity(search_velocity)
                        print(
                            "\r[ALIGN] R=-1、B=-1：已回中，"
                            "低速直行等待右前ToF先恢复。   ",
                            end=""
                        )

                elif right_valid and not back_valid:
                    # 右前已经看到墙，保持直行让右后逐渐进入量程。
                    both_far_turn_end = None
                    set_steer_auto(0.0)
                    if right <= ALIGN_SEARCH_MIN_SAFE_CM:
                        print(
                            f"\n[ALIGN][SAFETY] 右前距离仅"
                            f"{right:.1f}cm，停止搜索以防撞墙。"
                        )
                        break
                    set_velocity(search_velocity)
                    print(
                        f"\r[ALIGN] R:{right:5.1f} B:-1 | "
                        f"回中并以{align_rpm:.1f}RPM前进，"
                        "等待B恢复。   ",
                        end=""
                    )

                else:
                    # 右后看到墙而右前没有：直线倒退让右前进入量程。
                    both_far_turn_end = None
                    set_steer_auto(0.0)
                    if back <= ALIGN_SEARCH_MIN_SAFE_CM:
                        print(
                            f"\n[ALIGN][SAFETY] 右后距离仅"
                            f"{back:.1f}cm，停止搜索以防撞墙。"
                        )
                        break
                    set_velocity(-search_velocity)
                    print(
                        f"\r[ALIGN] R:-1 B:{back:5.1f} | "
                        f"回中并以{align_rpm:.1f}RPM倒退，"
                        "等待R恢复。   ",
                        end=""
                    )
                continue

            # 两颗ToF都恢复后，结束搜索阶段并进入正式姿态校正。
            search_start_time = None
            both_far_turn_end = None

            error_cm = float(right) - float(back)

            if abs(error_cm) <= ALIGN_TOLERANCE_CM:
                # 一进入允许范围就先停车，避免为了确认三帧而继续越过。
                set_velocity(0)

                stable_count += 1
                print(
                    f"\r[ALIGN] R:{right:5.1f} B:{back:5.1f} "
                    f"差值:{error_cm:+5.2f}cm "
                    f"稳定:{stable_count}/{ALIGN_STABLE_SAMPLES}   ",
                    end=""
                )

                if stable_count >= ALIGN_STABLE_SAMPLES:
                    success = True
                    break

                continue

            stable_count = 0

            steer_magnitude = (
                abs(error_cm)
                / ALIGN_FULL_STEER_ERROR_CM
                * ALIGN_MAX_STEER_AUTO
            )
            steer_magnitude = max(
                ALIGN_MIN_STEER_AUTO,
                min(ALIGN_MAX_STEER_AUTO, steer_magnitude)
            )

            # error<0：右前更近，负转向（向左）；error>0：向右。
            align_auto_steer = (
                STEER_SIGN
                * math.copysign(steer_magnitude, error_cm)
            )
            previous_steer_pos = last_steer_pos
            steer_deg = set_steer_auto(align_auto_steer)

            if last_steer_pos != previous_steer_pos:
                # 先让方向轮到位，再轻微前推。
                time.sleep(0.12)

            set_velocity(align_velocity)

            direction_text = "左" if error_cm < 0 else "右"
            print(
                f"\r[ALIGN] R:{right:5.1f} B:{back:5.1f} "
                f"差值:{error_cm:+5.2f}cm | "
                f"转向:{direction_text} {steer_deg:+5.1f}° | "
                f"速度:{align_rpm:.1f}RPM   ",
                end=""
            )

    finally:
        set_velocity(0)
        set_steer_auto(0.0)
        time.sleep(0.3)

    if success:
        print("\n[ALIGN] 姿态校正完成：车辆已停止，转向已回中。")
    else:
        print("\n[ALIGN] 姿态校正未完成：已安全停车并将转向回中。")

    return success


# ================= 统一硬件初始化 =================
def initialize_robot():
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

    # =========================================================
    # 启动前检测并自动恢复两个 DYNAMIXEL
    # =========================================================
    throttle_ready = recover_and_configure_dynamixel(
        port=port,
        packet=packet,
        dxl_id=ID_THROTTLE,
        operating_mode=VELOCITY_MODE,
        profile_accel=100,
        profile_velocity=None,
        motor_name="驱动/行驶电机（stepping/throttle）",
    )

    steer_ready = recover_and_configure_dynamixel(
        port=port,
        packet=packet,
        dxl_id=ID_STEER,
        operating_mode=POSITION_MODE,
        profile_accel=100,
        profile_velocity=500,
        motor_name="转向舵机（servo/steering）",
    )

    if not throttle_ready or not steer_ready:
        if not throttle_ready:
            print(
                "\n[MOTOR FAILED] 驱动/行驶电机 "
                "（stepping/throttle，ID 1）恢复失败。"
            )
        if not steer_ready:
            print(
                "\n[MOTOR FAILED] 转向舵机 "
                "（servo/steering，ID 2）恢复失败。"
            )
        print(
            "\n[SAFETY] 上述电机未通过启动检测，"
            "关闭端口并退出。"
        )

        # 尽力关闭两个电机 Torque
        packet.write1ByteTxRx(
            port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF
        )
        packet.write1ByteTxRx(
            port, ID_STEER, ADDR_TORQUE, TORQUE_OFF
        )

        port.closePort()
        ser_tof.close()
        sys.exit(1)

    zero_pos =2010#2047

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

    return {
        "ser_tof": ser_tof,
        "port": port,
        "packet": packet,
        "zero_pos": zero_pos,
        "shared_tof": shared_tof,
        "stop_event": stop_event,
        "receiver": receiver,
        "shutdown_complete": False,
    }


# ================= 全局安全停车与资源关闭 =================
def safe_shutdown_robot(robot, reason=""):
    """
    所有运动功能共用的最终安全关闭程序。
    可被重复调用；第一次执行后，后续调用不会再次操作已关闭的端口。
    """
    if robot is None or robot.get("shutdown_complete", False):
        return

    # 先锁定状态，避免多个finally重复关闭同一串口。
    robot["shutdown_complete"] = True

    port = robot["port"]
    packet = robot["packet"]
    zero_pos = robot["zero_pos"]
    stop_event = robot["stop_event"]
    receiver = robot["receiver"]
    ser_tof = robot["ser_tof"]

    if reason:
        print(f"\n[GLOBAL SAFETY] {reason}")
    print("[GLOBAL SAFETY] 正在停止驱动电机并关闭全部硬件……")

    # 第一优先级：立即停止驱动电机。
    try:
        packet.write4ByteTxRx(
            port,
            ID_THROTTLE,
            ADDR_GOAL_VELOCITY,
            0
        )
    except Exception as exc:
        print(f"[GLOBAL SAFETY] 驱动速度归零失败：{exc}")

    # 第二优先级：转向回中。
    try:
        packet.write4ByteTxRx(
            port,
            ID_STEER,
            ADDR_GOAL_POSITION,
            zero_pos
        )
    except Exception as exc:
        print(f"[GLOBAL SAFETY] 转向回中失败：{exc}")

    # 停止ToF接收线程。
    try:
        stop_event.set()
        receiver.join(timeout=1.0)
    except Exception as exc:
        print(f"[GLOBAL SAFETY] ToF线程停止失败：{exc}")

    # 关闭两个电机Torque；即使前面的速度命令通信失败，也会再次强制停机。
    try:
        packet.write1ByteTxRx(
            port,
            ID_THROTTLE,
            ADDR_TORQUE,
            TORQUE_OFF
        )
    except Exception as exc:
        print(f"[GLOBAL SAFETY] 驱动电机Torque Off失败：{exc}")

    try:
        packet.write1ByteTxRx(
            port,
            ID_STEER,
            ADDR_TORQUE,
            TORQUE_OFF
        )
    except Exception as exc:
        print(f"[GLOBAL SAFETY] 转向电机Torque Off失败：{exc}")

    try:
        port.closePort()
    except Exception as exc:
        print(f"[GLOBAL SAFETY] Dynamixel串口关闭失败：{exc}")

    try:
        ser_tof.close()
    except Exception as exc:
        print(f"[GLOBAL SAFETY] ToF串口关闭失败：{exc}")

    print("[GLOBAL SAFETY] 驱动已停止，Torque已关闭，程序安全退出。")


# ================= 功能一：启动时与右墙对齐 =================
def align_to_wall(robot, speed):
    """
    执行一次车身与右墙平行校正并返回成功状态。
    校正失败不会关闭程序，main() 仍会继续调用 detect_parking_lot()。
    """
    if not AUTO_ALIGN_AT_START:
        print("\n[ALIGN] AUTO_ALIGN_AT_START=False，跳过启动姿态校正。")
        return True

    align_success = align_parallel_to_right_wall(
        port=robot["port"],
        packet=robot["packet"],
        shared_tof=robot["shared_tof"],
        receiver=robot["receiver"],
        zero_pos=robot["zero_pos"],
        speed=speed,
    )

    if not align_success:
        print(
            "\n[WARNING] 启动姿态校正未成功，"
            "但继续执行沿墙行驶和停车板检测。"
        )
    else:
        print(
            "\n[INFO] 启动姿态校正成功，"
            "继续执行沿墙行驶和停车板检测。"
        )

    return align_success


# ================= 功能三：按指定方向、速度和时间运行 =================
def run_for_time(robot, steering, speed, run_time):
    """
    steering: -1.0～+1.0，对应 -50°～+50°转向。
    speed:    -100～+100，表示最大设定RPM的百分比；负数为倒车。
    run_time: 运行秒数，必须大于或等于0。

    例如 run_for_time(robot, -1, 100, 3)：
    最大向左转，以 RUN_FOR_TIME_MAX_RPM 前进3秒，然后驱动速度归零。
    本函数结束时保留指定转向角，不自动回中。
    """
    steering = float(steering)
    speed = float(speed)
    run_time = float(run_time)

    if not math.isfinite(steering) or not -1.0 <= steering <= 1.0:
        raise ValueError("steering 必须是 -1.0 到 +1.0 之间的数值")
    if not math.isfinite(speed) or not -100.0 <= speed <= 100.0:
        raise ValueError("speed 必须是 -100 到 +100 之间的数值")
    if not math.isfinite(run_time) or run_time < 0.0:
        raise ValueError("run_time 必须是大于或等于0的秒数")

    port = robot["port"]
    packet = robot["packet"]
    zero_pos = robot["zero_pos"]

    steer_deg = steering * MAX_STEER_DEG
    steer_pos = int(zero_pos + deg_to_unit(steer_deg))
    target_rpm = speed / 100.0 * RUN_FOR_TIME_MAX_RPM
    velocity_value = int(target_rpm / VELOCITY_UNIT_RPM)

    print(
        f"\n[RUN FOR TIME] 转向:{steering:+.2f} "
        f"({steer_deg:+.1f}°) | 速度:{speed:+.1f}% "
        f"({target_rpm:+.2f}RPM) | 时间:{run_time:.2f}秒"
    )

    try:
        # 先让方向轮到达目标位置，再启动驱动电机。
        packet.write4ByteTxRx(
            port,
            ID_STEER,
            ADDR_GOAL_POSITION,
            steer_pos
        )
        time.sleep(0.12)

        packet.write4ByteTxRx(
            port,
            ID_THROTTLE,
            ADDR_GOAL_VELOCITY,
            to_uint32(velocity_value)
        )

        end_time = time.monotonic() + run_time
        while True:
            remaining = end_time - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.01, remaining))

    finally:
        # 正常结束、Ctrl+C或发生异常时都立即把驱动速度设为0。
        packet.write4ByteTxRx(
            port,
            ID_THROTTLE,
            ADDR_GOAL_VELOCITY,
            0
        )

    print("[RUN FOR TIME] 运行结束，驱动速度已归零。")


# ================= 功能四：按轮子实际行驶距离运行 =================
def runForDistance(robot, steering, speed, distance_cm):
    """
    steering:   -1.0～+1.0，对应 -50°～+50°转向。
    speed:      -100～+100，表示最大设定RPM的百分比；负数为倒车。
    distance_cm:轮子的目标行驶距离，单位cm，最小为0.1cm。

    Dynamixel驱动电机每转一整圈对应4096个Position单位。
    电机转1圈时，经过3倍加速后车轮转3圈。
    轮胎直径为6.24cm，因此电机转1圈的理论行驶距离为：
        pi × 6.24 × 3 ≈ 58.81cm

    实测命令20.0cm时只行驶18.5cm，因此换算中乘以0.925校准系数；
    校准后会让电机比原公式多转约8.108%，使实际距离更接近输入值。

    本函数读取ID_THROTTLE的Present Position，累计电机真实变化量，
    再换算为校准后的行驶厘米数；可处理0/4095回绕和超过一圈的距离。

    例如 runForDistance(robot, 1, -100, 30)：
    最大向右转，以100%速度倒退，理论行驶30cm后停车。
    本函数结束时保留指定转向角，不自动回中。
    """
    steering = float(steering)
    speed = float(speed)
    distance_cm = float(distance_cm)

    if not math.isfinite(steering) or not -1.0 <= steering <= 1.0:
        raise ValueError("steering 必须是 -1.0 到 +1.0 之间的数值")
    if not math.isfinite(speed) or not -100.0 <= speed <= 100.0:
        raise ValueError("speed 必须是 -100 到 +100 之间的数值")
    if not math.isfinite(distance_cm) or distance_cm < 0.1:
        raise ValueError("distance_cm 必须是大于或等于0.1的厘米数")
    if abs(speed) < 0.01:
        raise ValueError("运行距离大于0时，speed不能为0")

    port = robot["port"]
    packet = robot["packet"]
    zero_pos = robot["zero_pos"]

    steer_deg = steering * MAX_STEER_DEG
    steer_pos = int(zero_pos + deg_to_unit(steer_deg))
    target_rpm = speed / 100.0 * RUN_FOR_TIME_MAX_RPM
    velocity_value = int(target_rpm / VELOCITY_UNIT_RPM)
    target_motor_units = distance_cm / DISTANCE_CM_PER_MOTOR_UNIT

    def read_throttle_position():
        raw_position, comm_result, dxl_error = packet.read4ByteTxRx(
            port,
            ID_THROTTLE,
            ADDR_PRESENT_POSITION
        )
        if not dxl_comm_ok(
            packet,
            comm_result,
            dxl_error,
            "读取 Present Position",
            ID_THROTTLE
        ):
            raise RuntimeError("无法读取驱动电机的Present Position")
        return int(raw_position) % DXL_POSITION_UNITS_PER_REV

    # 在启动电机以前读取起始位置，避免把启动前的变化计入目标距离。
    previous_position = read_throttle_position()
    accumulated_motor_units = 0.0
    accumulated_distance_cm = 0.0

    print(
        f"\n[RUN FOR DISTANCE] 转向:{steering:+.2f} "
        f"({steer_deg:+.1f}°) | 速度:{speed:+.1f}% "
        f"({target_rpm:+.2f}RPM) | "
        f"目标距离:{distance_cm:.2f}cm | "
        f"目标电机量:{target_motor_units:.1f}单位"
    )

    try:
        # 先让方向轮到达目标位置，再启动驱动电机。
        packet.write4ByteTxRx(
            port,
            ID_STEER,
            ADDR_GOAL_POSITION,
            steer_pos
        )
        time.sleep(0.12)

        packet.write4ByteTxRx(
            port,
            ID_THROTTLE,
            ADDR_GOAL_VELOCITY,
            to_uint32(velocity_value)
        )

        last_motion_time = time.monotonic()

        while accumulated_distance_cm < distance_cm:
            loop_start = time.monotonic()
            current_position = read_throttle_position()

            # 把跨越0/4095的差值转换为本周期内最短的有符号变化量。
            delta = current_position - previous_position
            half_circle = DXL_POSITION_UNITS_PER_REV // 2
            if delta > half_circle:
                delta -= DXL_POSITION_UNITS_PER_REV
            elif delta < -half_circle:
                delta += DXL_POSITION_UNITS_PER_REV

            previous_position = current_position

            if delta != 0:
                accumulated_motor_units += abs(delta)
                accumulated_distance_cm = (
                    accumulated_motor_units
                    * DISTANCE_CM_PER_MOTOR_UNIT
                )
                last_motion_time = loop_start
            elif (
                loop_start - last_motion_time
                > RUN_FOR_DISTANCE_STALL_TIMEOUT_SEC
            ):
                raise RuntimeError(
                    "驱动电机位置长时间没有变化，已安全停车"
                )

            print(
                f"\r[RUN FOR DISTANCE] 已运行:"
                f"{accumulated_distance_cm:7.2f}/"
                f"{distance_cm:.2f}cm | "
                f"电机累计:{accumulated_motor_units:.1f}单位 | "
                f"当前位置:{current_position:4d}   ",
                end=""
            )

            remaining_time = (
                RUN_FOR_DISTANCE_CONTROL_PERIOD
                - (time.monotonic() - loop_start)
            )
            if remaining_time > 0.0:
                time.sleep(remaining_time)

    finally:
        # 达到目标、Ctrl+C或发生异常时都立即把驱动速度设为0。
        packet.write4ByteTxRx(
            port,
            ID_THROTTLE,
            ADDR_GOAL_VELOCITY,
            0
        )

    print(
        f"\n[RUN FOR DISTANCE] 运行结束，"
        f"理论行驶:{accumulated_distance_cm:.2f}cm，"
        f"电机累计:{accumulated_motor_units:.1f}单位，"
        "驱动速度已归零。"
    )


# ================= 功能五：按距离运行，并用前/后ToF提前停车 =================
def runForDistanceByToF(
    robot,
    steering,
    speed,
    distance_cm,
    keep_distance_cm,
):
    """
    与runForDistance相同，但增加ToF提前停车条件：
      speed > 0：监测front_val（车头前方ToF）。
      speed < 0：监测extra_val（原左侧、现安装于车尾的后向ToF）。

    达到distance_cm，或对应ToF距离<=keep_distance_cm，任一条件先满足
    都会立即把驱动速度设为0。ToF返回-1表示超量程，不触发近距离停车。
    """
    steering = float(steering)
    speed = float(speed)
    distance_cm = float(distance_cm)
    keep_distance_cm = float(keep_distance_cm)

    if not math.isfinite(steering) or not -1.0 <= steering <= 1.0:
        raise ValueError("steering 必须是 -1.0 到 +1.0 之间的数值")
    if not math.isfinite(speed) or not -100.0 <= speed <= 100.0:
        raise ValueError("speed 必须是 -100 到 +100 之间的数值")
    if abs(speed) < 0.01:
        raise ValueError("speed不能为0")
    if not math.isfinite(distance_cm) or distance_cm < 0.1:
        raise ValueError("distance_cm 必须是大于或等于0.1的厘米数")
    if not math.isfinite(keep_distance_cm) or keep_distance_cm < 0.1:
        raise ValueError("keep_distance_cm 必须是大于或等于0.1的厘米数")

    port = robot["port"]
    packet = robot["packet"]
    zero_pos = robot["zero_pos"]
    shared_tof = robot["shared_tof"]
    receiver = robot["receiver"]

    tof_key = "front_val" if speed > 0.0 else "extra_val"
    tof_name = "前方ToF" if speed > 0.0 else "后方ToF"

    steer_deg = steering * MAX_STEER_DEG
    steer_pos = int(zero_pos + deg_to_unit(steer_deg))
    target_rpm = speed / 100.0 * RUN_FOR_TIME_MAX_RPM
    velocity_value = int(target_rpm / VELOCITY_UNIT_RPM)
    target_motor_units = distance_cm / DISTANCE_CM_PER_MOTOR_UNIT

    def read_throttle_position():
        raw_position, comm_result, dxl_error = packet.read4ByteTxRx(
            port,
            ID_THROTTLE,
            ADDR_PRESENT_POSITION
        )
        if not dxl_comm_ok(
            packet,
            comm_result,
            dxl_error,
            "读取 Present Position",
            ID_THROTTLE
        ):
            raise RuntimeError("无法读取驱动电机的Present Position")
        return int(raw_position) % DXL_POSITION_UNITS_PER_REV

    previous_position = read_throttle_position()
    accumulated_motor_units = 0.0
    accumulated_distance_cm = 0.0
    stop_reason = "达到目标行驶距离"
    last_tof_value = -1.0

    print(
        f"\n[RUN FOR DISTANCE BY TOF] 转向:{steering:+.2f} "
        f"({steer_deg:+.1f}°) | 速度:{speed:+.1f}% "
        f"({target_rpm:+.2f}RPM) | 目标距离:{distance_cm:.2f}cm | "
        f"{tof_name}停车距离:{keep_distance_cm:.2f}cm | "
        f"目标电机量:{target_motor_units:.1f}单位"
    )

    try:
        packet.write4ByteTxRx(
            port,
            ID_STEER,
            ADDR_GOAL_POSITION,
            steer_pos
        )
        time.sleep(0.12)

        # 启动以前先确认ToF数据是新的；如果已经进入安全距离则不启动。
        tof = shared_tof.snapshot()
        now = time.monotonic()
        tof_age = (
            now - tof["timestamp"]
            if tof["timestamp"] > 0.0
            else float("inf")
        )
        if tof_age > RUN_FOR_DISTANCE_TOF_STALE_TIMEOUT_SEC:
            raise RuntimeError(f"{tof_name}数据过期，已禁止启动")

        last_tof_value = float(tof[tof_key])
        if (
            last_tof_value != -1
            and last_tof_value > 0.0
            and last_tof_value <= keep_distance_cm
        ):
            stop_reason = (
                f"启动前{tof_name}已达到安全距离"
                f"({last_tof_value:.1f}cm)"
            )
        else:
            packet.write4ByteTxRx(
                port,
                ID_THROTTLE,
                ADDR_GOAL_VELOCITY,
                to_uint32(velocity_value)
            )

            last_motion_time = time.monotonic()

            while accumulated_distance_cm < distance_cm:
                loop_start = time.monotonic()

                if receiver.error is not None:
                    raise RuntimeError(f"ToF串口异常：{receiver.error}")

                tof = shared_tof.snapshot()
                tof_age = (
                    loop_start - tof["timestamp"]
                    if tof["timestamp"] > 0.0
                    else float("inf")
                )
                if tof_age > RUN_FOR_DISTANCE_TOF_STALE_TIMEOUT_SEC:
                    raise RuntimeError(f"{tof_name}数据过期，已安全停车")

                last_tof_value = float(tof[tof_key])
                if (
                    last_tof_value != -1
                    and last_tof_value > 0.0
                    and last_tof_value <= keep_distance_cm
                ):
                    stop_reason = (
                        f"{tof_name}达到安全距离"
                        f"({last_tof_value:.1f}cm)"
                    )
                    break

                current_position = read_throttle_position()
                delta = current_position - previous_position
                half_circle = DXL_POSITION_UNITS_PER_REV // 2
                if delta > half_circle:
                    delta -= DXL_POSITION_UNITS_PER_REV
                elif delta < -half_circle:
                    delta += DXL_POSITION_UNITS_PER_REV

                previous_position = current_position

                if delta != 0:
                    accumulated_motor_units += abs(delta)
                    accumulated_distance_cm = (
                        accumulated_motor_units
                        * DISTANCE_CM_PER_MOTOR_UNIT
                    )
                    last_motion_time = loop_start
                elif (
                    loop_start - last_motion_time
                    > RUN_FOR_DISTANCE_STALL_TIMEOUT_SEC
                ):
                    raise RuntimeError(
                        "驱动电机位置长时间没有变化，已安全停车"
                    )

                tof_text = (
                    "-1(超量程)"
                    if last_tof_value == -1
                    else f"{last_tof_value:.1f}cm"
                )
                print(
                    f"\r[RUN FOR DISTANCE BY TOF] 已运行:"
                    f"{accumulated_distance_cm:7.2f}/"
                    f"{distance_cm:.2f}cm | "
                    f"{tof_name}:{tof_text} | "
                    f"停车值:{keep_distance_cm:.1f}cm   ",
                    end=""
                )

                remaining_time = (
                    RUN_FOR_DISTANCE_CONTROL_PERIOD
                    - (time.monotonic() - loop_start)
                )
                if remaining_time > 0.0:
                    time.sleep(remaining_time)

    finally:
        packet.write4ByteTxRx(
            port,
            ID_THROTTLE,
            ADDR_GOAL_VELOCITY,
            0
        )

    print(
        f"\n[RUN FOR DISTANCE BY TOF] 运行结束：{stop_reason}；"
        f"已运行:{accumulated_distance_cm:.2f}cm，驱动速度已归零。"
    )


# ================= 功能六：按时间运行，并用前/后ToF提前停车 =================
def runForTimeByToF(
    robot,
    steering,
    speed,
    run_time,
    keep_distance_cm,
):
    """
    与run_for_time相同，但增加ToF提前停车条件：
      speed > 0：监测front_val（车头前方ToF）。
      speed < 0：监测extra_val（原左侧、现安装于车尾的后向ToF）。

    达到run_time，或对应ToF距离<=keep_distance_cm，任一条件先满足
    都会立即把驱动速度设为0。ToF返回-1表示超量程，不触发近距离停车。
    """
    steering = float(steering)
    speed = float(speed)
    run_time = float(run_time)
    keep_distance_cm = float(keep_distance_cm)

    if not math.isfinite(steering) or not -1.0 <= steering <= 1.0:
        raise ValueError("steering 必须是 -1.0 到 +1.0 之间的数值")
    if not math.isfinite(speed) or not -100.0 <= speed <= 100.0:
        raise ValueError("speed 必须是 -100 到 +100 之间的数值")
    if abs(speed) < 0.01:
        raise ValueError("speed不能为0")
    if not math.isfinite(run_time) or run_time < 0.0:
        raise ValueError("run_time 必须是大于或等于0的秒数")
    if not math.isfinite(keep_distance_cm) or keep_distance_cm < 0.1:
        raise ValueError("keep_distance_cm 必须是大于或等于0.1的厘米数")

    port = robot["port"]
    packet = robot["packet"]
    zero_pos = robot["zero_pos"]
    shared_tof = robot["shared_tof"]
    receiver = robot["receiver"]

    tof_key = "front_val" if speed > 0.0 else "extra_val"
    tof_name = "前方ToF" if speed > 0.0 else "后方ToF"

    steer_deg = steering * MAX_STEER_DEG
    steer_pos = int(zero_pos + deg_to_unit(steer_deg))
    target_rpm = speed / 100.0 * RUN_FOR_TIME_MAX_RPM
    velocity_value = int(target_rpm / VELOCITY_UNIT_RPM)

    elapsed_time = 0.0
    last_tof_value = -1.0
    stop_reason = "达到目标运行时间"

    print(
        f"\n[RUN FOR TIME BY TOF] 转向:{steering:+.2f} "
        f"({steer_deg:+.1f}°) | 速度:{speed:+.1f}% "
        f"({target_rpm:+.2f}RPM) | 目标时间:{run_time:.2f}秒 | "
        f"{tof_name}停车距离:{keep_distance_cm:.2f}cm"
    )

    try:
        packet.write4ByteTxRx(
            port,
            ID_STEER,
            ADDR_GOAL_POSITION,
            steer_pos
        )
        time.sleep(0.12)

        # 启动以前先确认ToF数据是新的；如果已经进入安全距离则不启动。
        tof = shared_tof.snapshot()
        now = time.monotonic()
        tof_age = (
            now - tof["timestamp"]
            if tof["timestamp"] > 0.0
            else float("inf")
        )
        if tof_age > RUN_FOR_DISTANCE_TOF_STALE_TIMEOUT_SEC:
            raise RuntimeError(f"{tof_name}数据过期，已禁止启动")

        last_tof_value = float(tof[tof_key])
        if (
            last_tof_value != -1
            and last_tof_value > 0.0
            and last_tof_value <= keep_distance_cm
        ):
            stop_reason = (
                f"启动前{tof_name}已达到安全距离"
                f"({last_tof_value:.1f}cm)"
            )
        elif run_time <= 0.0:
            stop_reason = "目标运行时间为0秒"
        else:
            packet.write4ByteTxRx(
                port,
                ID_THROTTLE,
                ADDR_GOAL_VELOCITY,
                to_uint32(velocity_value)
            )

            start_time = time.monotonic()
            end_time = start_time + run_time

            while True:
                loop_start = time.monotonic()
                elapsed_time = loop_start - start_time

                if loop_start >= end_time:
                    elapsed_time = run_time
                    break

                if receiver.error is not None:
                    raise RuntimeError(f"ToF串口异常：{receiver.error}")

                tof = shared_tof.snapshot()
                tof_age = (
                    loop_start - tof["timestamp"]
                    if tof["timestamp"] > 0.0
                    else float("inf")
                )
                if tof_age > RUN_FOR_DISTANCE_TOF_STALE_TIMEOUT_SEC:
                    raise RuntimeError(f"{tof_name}数据过期，已安全停车")

                last_tof_value = float(tof[tof_key])
                if (
                    last_tof_value != -1
                    and last_tof_value > 0.0
                    and last_tof_value <= keep_distance_cm
                ):
                    stop_reason = (
                        f"{tof_name}达到安全距离"
                        f"({last_tof_value:.1f}cm)"
                    )
                    break

                tof_text = (
                    "-1(超量程)"
                    if last_tof_value == -1
                    else f"{last_tof_value:.1f}cm"
                )
                print(
                    f"\r[RUN FOR TIME BY TOF] 已运行:"
                    f"{elapsed_time:6.2f}/{run_time:.2f}秒 | "
                    f"{tof_name}:{tof_text} | "
                    f"停车值:{keep_distance_cm:.1f}cm   ",
                    end=""
                )

                remaining_period = (
                    RUN_FOR_DISTANCE_CONTROL_PERIOD
                    - (time.monotonic() - loop_start)
                )
                remaining_total = end_time - time.monotonic()
                sleep_time = min(remaining_period, remaining_total)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

    finally:
        packet.write4ByteTxRx(
            port,
            ID_THROTTLE,
            ADDR_GOAL_VELOCITY,
            0
        )

    print(
        f"\n[RUN FOR TIME BY TOF] 运行结束：{stop_reason}；"
        f"已运行:{elapsed_time:.2f}秒，驱动速度已归零。"
    )


# ================= 功能二：沿墙行驶并检测停车区 =================
def detect_parking_lot(robot, speed):
    """
    沿右墙行驶，依次用右前、右后 ToF 检测停车板；
    确认两颗传感器都通过停车板后立即停车并安全关闭硬件。
    """
    ser_tof = robot["ser_tof"]
    port = robot["port"]
    packet = robot["packet"]
    zero_pos = robot["zero_pos"]
    shared_tof = robot["shared_tof"]
    stop_event = robot["stop_event"]
    receiver = robot["receiver"]

    detector = ParkingBoardDetector()
    wall_follower = TwoToFWallFollower()

    speed = float(speed)
    if not math.isfinite(speed) or not 0.0 < speed <= 100.0:
        raise ValueError("detect_parking_lot的speed必须大于0且不超过100")

    target_rpm = speed / 100.0 * TARGET_RPM


    last_vel = None
    last_pos = None
    last_sensor_sequence = -1

    auto_steer = 0.0
    d_perp = 0.0
    wall_angle_deg = 0.0
    right_val = -1.0
    back_val = -1.0
    right_raw = -1.0
    back_raw = -1.0

    # 主控循环固定为约 100 Hz。
    # 串口线程独立接收，所以这里不会等待串口。
    CONTROL_PERIOD = 0.010

    # 超过这个时间没有新 ToF 数据，认为传感器数据已过期
    SENSOR_STALE_TIMEOUT = 0.30

    print(
        f"\nStarting two-thread PURE-P wall control... "
        f"speed={speed:.1f}%（约{target_rpm:.1f}RPM），"
        f"Max {MAX_RUN_SECONDS:.1f} seconds"
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

                # 保留串口原始值，便于板检测和调试。
                right_raw = tof["right_val"]
                back_raw = tof["back_val"]

                # =================================================
                # 超量程处理
                #
                # 传感器返回 -1 时，表示目标超出约 50 cm 的有效量程。
                # 对沿墙控制而言，将它解释成 50 cm 饱和值。
                #
                # 注意：只对控制用数据做替换；垂直板检测仍使用原始值，
                # 避免把一次 -1 错误当成真实 50 cm 数据参与板检测。
                # =================================================
                right_val = (
                    TOF_FAR_VALUE_CM
                    if right_raw == -1
                    else min(float(right_raw), TOF_FAR_VALUE_CM)
                )
                back_val = (
                    TOF_FAR_VALUE_CM
                    if back_raw == -1
                    else min(float(back_raw), TOF_FAR_VALUE_CM)
                )

                # =================================================
                # Parking lot 垂直板检测
                #
                # 检测器使用右前、右后两颗 ToF 的原始值分别锁存。
                # 特殊近距离只用于板检测；返回的 control_right/control_back
                # 已用最近正常墙距替换特殊值，不会干扰沿墙控制。
                # =================================================
                board = detector.update(
                    right_raw=right_raw,
                    back_raw=back_raw,
                    is_moving=is_moving,
                    now=now_monotonic
                )

                if board["right_new"]:
                    print(
                        f"\n[BOARD] 右前 ToF 记录到停车板："
                        f"原始值={board['right_board_value']:.1f} cm，"
                        f"正常基准={board['right_board_baseline']:.1f} cm"
                    )

                if board["back_new"]:
                    print(
                        f"\n[BOARD] 右后 ToF 记录到停车板："
                        f"原始值={board['back_board_value']:.1f} cm，"
                        f"正常基准={board['back_board_baseline']:.1f} cm"
                    )

                if board["right_recovered"]:
                    print(
                        "\n[BOARD] 右前 ToF 已离开停车板，"
                        "恢复沿墙转向，等待右后 ToF。"
                    )

                if board["back_recovered"]:
                    print(
                        "\n[BOARD] 右后 ToF 已离开停车板，"
                        "前后检测流程完成。"
                    )

                if board["detected"]:
                    print("\n\n" + "=" * 45)
                    print(
                        f"【核心提示】在第 "
                        f"{elapsed_time:.1f} 秒，右前和右后 ToF "
                        f"均已通过停车板，车辆立即停止。"
                    )
                    print(
                        f"右前特殊值：{board['right_board_value']:.1f} cm | "
                        f"右后特殊值：{board['back_board_value']:.1f} cm"
                    )
                    print("=" * 45)

                    print("\n[INFO] 停车板检测完成，车辆停车退出。")

                    break

                # 将不受停车板特殊读数影响的数据交给沿墙控制。
                right_for_control = board["control_right"]
                back_for_control = board["control_back"]

                right_val = (
                    TOF_FAR_VALUE_CM
                    if right_for_control == -1
                    else min(float(right_for_control), TOF_FAR_VALUE_CM)
                )
                back_val = (
                    TOF_FAR_VALUE_CM
                    if back_for_control == -1
                    else min(float(back_for_control), TOF_FAR_VALUE_CM)
                )

                if board["force_straight"]:
                    # 任一传感器正在扫过停车板时，无条件直行。
                    # 清除沿墙控制器内部保留的旧转向，避免恢复控制时跳变。
                    auto_steer = 0.0
                    wall_follower.last_auto_steer = 0.0
                    wall_follower.last_update_time = now_monotonic
                else:
                    # 沿墙控制使用经过 50 cm 饱和处理后的数据。
                    # 两组数据之间，继续使用上一次的 auto_steer。
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
                f"R:{right_val:5.1f}({right_raw:5.1f}) "
                f"B:{back_val:5.1f}({back_raw:5.1f}) | "
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
        print("\n[CTRL+C] detect_parking_lot收到退出指令。")
        raise

    finally:
        # 功能一结束时只停车，不关闭串口、Torque或ToF线程，
        # 这样功能二可以继续使用同一套硬件连接。
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
        time.sleep(0.3)


# ================= 用户选择的运动功能顺序 =================
def run_selected_motion_sequence(robot):

    runForDistanceByToF(
          robot, steering=0, speed=10,
          distance_cm=38, keep_distance_cm=8#kaishi qianjin original 40
    )
    
    time.sleep(2)
    # 倒车时自动监测原左侧、现在安装在车尾的extra_val后向ToF。
    runForDistanceByToF(
          robot, steering=1, speed=-10,
          distance_cm=50, keep_distance_cm=8#ruku daodi og 48
      )
    time.sleep(1.5)
    runForDistanceByToF(
          robot, steering=0, speed=10,
          distance_cm=15, keep_distance_cm=8#qianjin
      )

    time.sleep(1.5)

    runForDistanceByToF(
          robot, steering=-1, speed=-10,
          distance_cm=37, keep_distance_cm=10#cemian daoche 30
      )

    time.sleep(2)
    
    runForDistanceByToF(
          robot, steering=1, speed=10,
          distance_cm=11, keep_distance_cm=8#tiao zheng 15
      )

    time.sleep(2)

    runForDistanceByToF(
          robot, steering=-1, speed=-10,
          distance_cm=13, keep_distance_cm=8#tiao zheng 10
      )

   

    # 功能六：最多前进3秒，或前方ToF先达到10cm时提前停车。
#     runForTimeByToF(
#          robot, steering=0, speed=-30,
#          run_time=10, keep_distance_cm=12
#      )

    # 倒车时自动监测车尾的extra_val；最多倒车2秒或在8cm处停车。
#     runForTimeByToF(
#           robot, steering=0, speed=-30,
#           run_time=10, keep_distance_cm=12
#       )

    # 功能二：沿墙行驶并检测停车区。
    #detect_parking_lot(robot, speed=DEFAULT_PARKING_LOT_SPEED)


# ================= 全局主程序与Ctrl+C保护 =================
def main():
    robot = None

    try:
        robot = initialize_robot()

        detect_parking_lot(robot, 15)
        run_selected_motion_sequence(robot)

    except KeyboardInterrupt:
        print("\n[CTRL+C] 收到全局退出指令，立即安全停车。")

    finally:
        safe_shutdown_robot(
            robot,
            reason="所有运动功能已经结束，执行全局安全关闭。"
        )


if __name__ == "__main__":
    main()
