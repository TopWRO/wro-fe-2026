#!/usr/bin/env python3
"""
WRO 2025 – 统一主控脚本（Unified I/O 20Hz 版本）

功能分三层：
1) 启动时：
   - 打开串口 (Pico)
   - 加载 AI 模型 (KerasLinear)
   - 初始化 PiCamera
   - 初始化 UnifiedRobotIO / DynamixelSteering / DynamixelThrottle

2) 主功能：
   - gyro_run(): 由陀螺 + ToF + PID 控制的小段行驶（目标角度跟随）
   - ai_run():   由 AI 自动驾驶 + 陀螺累计角度停止

3) main():
   - 多次调用 gyro_run() + 一次 ai_run() 组成完整流程
"""

import sys
import time
from pathlib import Path
from threading import Thread, Lock, Event
from sense_hat import SenseHat


import cv2
import serial
from dynamixel_sdk import *

# DonkeyCar 部分
try:
    from donkeycar.parts.keras import KerasInterpreter, KerasLinear
    from donkeycar.parts.camera import PiCamera
except ImportError:
    sys.exit("请先安装带 Keras/TensorFlow 支持的 DonkeyCar。")

# =====================  全局配置  =====================

# 相机尺寸和裁剪尺寸（与你训练时保持一致）
CAPTURE_W, CAPTURE_H = (320, 240)
CROP_W, CROP_H       = (160, 120)


MODEL_PATH_CCW       = str(Path("~/Top_Singapore/ccwmodel/mypilot.h5").expanduser())
MODEL_PATH_CW        = str(Path("~/Top_Singapore/cwmodel/teshucwmypilot.h5").expanduser())
Run_Clockwise        = "CCW"
MODEL_PATH_DEFAULT   = MODEL_PATH_CCW
AI_LOOP_HZ           = 15.0
AI_MAX_SPEED_DEFAULT = 0.40   # AI throttle execution limiter; model output itself is unchanged
CONTROL_LOOP_HZ      = 20.0
CONTROL_LOOP_DT      = 1.0 / CONTROL_LOOP_HZ

MAX_SPEED_PERCENT    = 55
STEERING_MAX_SPEED   = 100
ANGLE_OFFSET         = 1.1
AI_RUN_ANGLE_OFFSET  = 0.85
gyro_offset_cw       = 1.0
gyro_offset_ccw      = 1.01


SERIAL_PORT          = "/dev/ttyACM0"
SERIAL_BAUD          = 115200

# 统一 I/O 线程频率：Pico 串口 + Steering + Throttle + 轮子位置
IO_LOOP_HZ           = 20.0
IO_LOOP_DT           = 1.0 / IO_LOOP_HZ


# =====================  Dynamixel 电机配置  =====================
# 与当前 Robot / Run_BY_Gyro 软件平台保持一致：
# ID1 = 驱动/Throttle，Velocity Mode
# ID2 = 转向/Steering，Position Mode
DYNAMIXEL_PORT       = "/dev/ttyUSB0"
DYNAMIXEL_BAUD       = 57600

ID_THROTTLE          = 1
ID_STEER             = 2

ADDR_MODE            = 11
ADDR_TORQUE          = 64
ADDR_HARDWARE_ERROR  = 70
ADDR_GOAL_VELOCITY   = 104
ADDR_PROFILE_ACCEL   = 108
ADDR_PROFILE_VELOCITY= 112
ADDR_GOAL_POSITION   = 116
ADDR_PRESENT_POSITION= 132

VELOCITY_MODE        = 1
POSITION_MODE        = 3
TORQUE_OFF           = 0
TORQUE_ON            = 1
DXL_ALERT             = 0x80

# 当前机械参数。若实车中心位以后重新标定，只修改这一项。
DXL_STEER_CENTER_POS = 2016
DXL_MAX_STEER_DEG    = 40.0
DXL_TARGET_RPM       = 100.0
DXL_VELOCITY_UNIT_RPM= 0.229

# 当前驱动结构：电机 1 圈 -> 车轮约 3 圈。
# get_wheel_degrees() 返回“车轮等效角度”，尽量保持旧 obstacle 程序
# run_distance 参数的物理意义不变。
DXL_WHEEL_REVS_PER_MOTOR_REV = 3.0

DYNAMIXEL_REBOOT_WAIT_SEC = 1.0
DYNAMIXEL_RECOVERY_RETRIES = 2


def dxl_to_uint32(value):
    value = int(value)
    if value < 0:
        value += 4294967296
    return value


def dxl_to_int32(value):
    value = int(value) & 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


def dxl_deg_to_unit(deg):
    return int(float(deg) * 4096.0 / 360.0)


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


def dxl_comm_ok(packet, comm_result, dxl_error, action, dxl_id, allow_alert=False):
    if comm_result != COMM_SUCCESS:
        print(f"[DXL ID {dxl_id}] {action} 通信失败: {packet.getTxRxResult(comm_result)}")
        return False

    if dxl_error != 0:
        if allow_alert and (dxl_error & DXL_ALERT):
            return True
        print(
            f"[DXL ID {dxl_id}] {action} 状态包错误: "
            f"{packet.getRxPacketError(dxl_error)} (0x{dxl_error:02X})"
        )
        return False
    return True


def read_dxl_byte(port, packet, dxl_id, address, name, allow_alert=False):
    value, comm_result, dxl_error = packet.read1ByteTxRx(port, dxl_id, address)
    if not dxl_comm_ok(packet, comm_result, dxl_error, f"读取 {name}",
                       dxl_id, allow_alert=allow_alert):
        return None
    return value


def write_dxl_byte(port, packet, dxl_id, address, value, name, allow_alert=False):
    comm_result, dxl_error = packet.write1ByteTxRx(port, dxl_id, address, int(value))
    return dxl_comm_ok(packet, comm_result, dxl_error, f"写入 {name}={value}",
                       dxl_id, allow_alert=allow_alert)


def write_dxl_dword(port, packet, dxl_id, address, value, name, allow_alert=False):
    comm_result, dxl_error = packet.write4ByteTxRx(port, dxl_id, address, int(value))
    return dxl_comm_ok(packet, comm_result, dxl_error, f"写入 {name}={value}",
                       dxl_id, allow_alert=allow_alert)


def recover_and_configure_dynamixel(
    port, packet, dxl_id, operating_mode,
    profile_accel=100, profile_velocity=None,
    motor_name="Dynamixel"
):
    """启动前检查硬件错误；必要时 Reboot，然后重新配置并 Torque ON。"""
    for attempt in range(1, DYNAMIXEL_RECOVERY_RETRIES + 1):
        print(
            f"\n[STARTUP CHECK] {motor_name} (DXL ID {dxl_id}) "
            f"{attempt}/{DYNAMIXEL_RECOVERY_RETRIES}"
        )

        hw = read_dxl_byte(
            port, packet, dxl_id, ADDR_HARDWARE_ERROR,
            "Hardware Error Status", allow_alert=True
        )
        torque = read_dxl_byte(
            port, packet, dxl_id, ADDR_TORQUE,
            "Torque Enable", allow_alert=True
        )

        if hw is None or torque is None:
            time.sleep(0.2)
            continue

        print(
            f"[STATUS] {motor_name}: HW=0x{hw:02X}, Torque={torque}, "
            f"{decode_dxl_hardware_error(hw)}"
        )

        if hw != 0:
            print(f"[WARNING] 出问题的是 {motor_name} (ID {dxl_id})")
            comm_result, dxl_error = packet.reboot(port, dxl_id)
            if not dxl_comm_ok(
                packet, comm_result, dxl_error, "Reboot",
                dxl_id, allow_alert=True
            ):
                continue

            time.sleep(DYNAMIXEL_REBOOT_WAIT_SEC)

            hw = read_dxl_byte(
                port, packet, dxl_id, ADDR_HARDWARE_ERROR,
                "Reboot 后 Hardware Error Status", allow_alert=True
            )
            if hw != 0:
                print(
                    f"[FATAL] {motor_name} Reboot 后仍有错误: "
                    f"{decode_dxl_hardware_error(hw or 0)}"
                )
                continue

        # 修改 Operating Mode 前必须 Torque OFF
        if not write_dxl_byte(
            port, packet, dxl_id, ADDR_TORQUE, TORQUE_OFF,
            "Torque Enable", allow_alert=True
        ):
            continue

        if not write_dxl_byte(
            port, packet, dxl_id, ADDR_MODE, operating_mode,
            "Operating Mode", allow_alert=True
        ):
            continue

        if not write_dxl_dword(
            port, packet, dxl_id, ADDR_PROFILE_ACCEL, profile_accel,
            "Profile Acceleration", allow_alert=True
        ):
            continue

        if profile_velocity is not None:
            if not write_dxl_dword(
                port, packet, dxl_id, ADDR_PROFILE_VELOCITY, profile_velocity,
                "Profile Velocity", allow_alert=True
            ):
                continue

        # 驱动电机 Torque ON 前先清零速度
        if operating_mode == VELOCITY_MODE:
            if not write_dxl_dword(
                port, packet, dxl_id, ADDR_GOAL_VELOCITY, 0,
                "Goal Velocity 安全归零", allow_alert=True
            ):
                continue

        if not write_dxl_byte(
            port, packet, dxl_id, ADDR_TORQUE, TORQUE_ON,
            "Torque Enable", allow_alert=True
        ):
            continue

        time.sleep(0.1)

        final_hw = read_dxl_byte(
            port, packet, dxl_id, ADDR_HARDWARE_ERROR,
            "最终 Hardware Error Status", allow_alert=True
        )
        final_torque = read_dxl_byte(
            port, packet, dxl_id, ADDR_TORQUE,
            "最终 Torque Enable", allow_alert=True
        )

        if final_hw == 0 and final_torque == TORQUE_ON:
            print(f"[RECOVERY OK] {motor_name} (ID {dxl_id}) ready")
            return True

    print(f"[SAFETY] {motor_name} (ID {dxl_id}) 自动恢复失败")
    return False


class DynamixelBus:
    """两个 DXL 共用一个 U2D2 串口。"""

    def __init__(self):
        self.port = PortHandler(DYNAMIXEL_PORT)
        self.packet = PacketHandler(2.0)
        self.closed = False

        if not self.port.openPort():
            raise RuntimeError(f"无法打开 Dynamixel 端口 {DYNAMIXEL_PORT}")
        if not self.port.setBaudRate(DYNAMIXEL_BAUD):
            self.port.closePort()
            raise RuntimeError(f"无法设置 Dynamixel 波特率 {DYNAMIXEL_BAUD}")

        throttle_ok = recover_and_configure_dynamixel(
            self.port, self.packet,
            ID_THROTTLE, VELOCITY_MODE,
            profile_accel=100,
            profile_velocity=None,
            motor_name="驱动/Throttle 电机"
        )
        steer_ok = recover_and_configure_dynamixel(
            self.port, self.packet,
            ID_STEER, POSITION_MODE,
            profile_accel=100,
            profile_velocity=500,
            motor_name="转向/Steering 电机"
        )

        if not throttle_ok or not steer_ok:
            self.safe_stop()
            self.close()
            raise RuntimeError("Dynamixel 启动检测/恢复失败")

        # 初始化转向回中
        self.packet.write4ByteTxRx(
            self.port, ID_STEER, ADDR_GOAL_POSITION, DXL_STEER_CENTER_POS
        )
        time.sleep(0.2)
        print("[DXL] bus initialized")

    def safe_stop(self):
        try:
            self.packet.write4ByteTxRx(
                self.port, ID_THROTTLE, ADDR_GOAL_VELOCITY, 0
            )
        except Exception:
            pass
        try:
            self.packet.write4ByteTxRx(
                self.port, ID_STEER, ADDR_GOAL_POSITION, DXL_STEER_CENTER_POS
            )
        except Exception:
            pass

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.safe_stop()
        time.sleep(0.1)
        try:
            self.packet.write1ByteTxRx(
                self.port, ID_THROTTLE, ADDR_TORQUE, TORQUE_OFF
            )
        except Exception:
            pass
        try:
            self.packet.write1ByteTxRx(
                self.port, ID_STEER, ADDR_TORQUE, TORQUE_OFF
            )
        except Exception:
            pass
        try:
            self.port.closePort()
        except Exception:
            pass
        print("[DXL] bus closed")


# PiCamera 手动参数
CAMERA_FRAMERATE             = 30
PICAMERA_AWB_MODE            = 'off'
PICAMERA_EXPOSURE_MODE       = 'off'
PICAMERA_ISO                 = 100
PICAMERA_SHUTTER_SPEED       = 15000
PICAMERA_AWB_GAINS           = (1.2, 1.5)
PICAMERA_EXPOSURE_COMPENSATION = 0


# =====================  工具函数  =====================

def center_crop(img, tw=CROP_W, th=CROP_H):
    """简单处理：直接缩放到训练尺寸。"""
    resized = cv2.resize(img, (tw, th))
    return resized


# =====================  Dynamixel 电机模块  =====================

class UnifiedRobotIO:
    """
    单一 I/O 线程，统一管理：
      1) Pico USB Serial：只保留最新完整 7 字段帧
      2) DXL ID2 Steering：Position Mode
      3) DXL ID1 Throttle：Velocity Mode
      4) DXL ID1 Present Position：累计 wheel_degrees

    设计目标：所有 U2D2 Tx/Rx 只由这个线程发起，避免两个线程同时访问同一 PortHandler。
    主线程只读取传感器快照、写入 steering/throttle 目标值，不直接访问 U2D2。
    """

    def __init__(self, ser, bus, loop_hz=IO_LOOP_HZ, max_rpm=DXL_TARGET_RPM):
        self.ser = ser
        self.bus = bus
        self.port = bus.port
        self.packet = bus.packet
        self.loop_hz = float(loop_hz)
        self.loop_dt = 1.0 / self.loop_hz
        self.max_rpm = float(max_rpm)

        self.running = True
        self.sensor_lock = Lock()
        self.command_lock = Lock()
        self.state_lock = Lock()
        self.gyro_reset_event = Event()

        self.latest_data = {
            "angle": 0.0,
            "angle_valid": False,
            "front": 999.0,
            "R1": 999.0,
            "R2": 999.0,
            "back": 999.0,
            "L2": 999.0,
            "L1": 999.0,
        }

        # 只保存一个最新目标值，不使用命令队列，不存在旧命令积压。
        self.target_steering = 0.0
        self.target_throttle = 0.0
        self.last_steer_pos = None
        self.last_velocity_value = None

        # Pico 串口半帧缓存：完整帧立即解析，末尾残缺帧留到下一轮拼接。
        self._serial_rx_buffer = b""

        # 轮子累计位置
        self._position_initialized = False
        self._last_raw_position = 0
        self._accum_motor_units = 0.0
        self.wheel_degrees = 0.0

        # 启动前安全目标
        self._write_steering_immediate(0.0)
        self._write_throttle_immediate(0.0)

        self.thread = Thread(target=self._loop, daemon=True, name="UnifiedRobotIO")
        self.thread.start()
        print(f"[UnifiedIO] thread started at target {self.loop_hz:.1f} Hz")

    # ---------- 主线程 <-> I/O 线程：命令 ----------
    def set_steering(self, angle):
        angle = max(min(float(angle), 1.0), -1.0)
        with self.command_lock:
            self.target_steering = angle

    def set_throttle(self, throttle):
        throttle = max(min(float(throttle), 1.0), -1.0)
        with self.command_lock:
            self.target_throttle = throttle

    def get_sensor_data(self):
        with self.sensor_lock:
            return self.latest_data.copy()

    def get_wheel_degrees(self):
        with self.state_lock:
            return float(self.wheel_degrees)

    def request_gyro_reset(self):
        """由 I/O 线程自己向 Pico 发送 reset，避免主线程和 I/O 线程同时操作同一串口。"""
        self.gyro_reset_event.set()

    # ---------- Pico Serial ----------
    def _handle_gyro_reset_if_requested(self):
        if not self.gyro_reset_event.is_set():
            return
        try:
            self.ser.write(b"reset gyro\n")
            self.ser.flush()
            self.ser.reset_input_buffer()
            self._serial_rx_buffer = b""
            print("[UnifiedIO] reset gyro")
        except Exception as e:
            print("[UnifiedIO] reset gyro failed:", e)
        finally:
            self.gyro_reset_event.clear()

    def _read_latest_serial_frame(self):
        """
        非阻塞读出当前所有可用字节。
        仅解析以换行结束的完整帧；如果一次读到多帧，只应用最后一个合法 7 字段帧。
        正式格式固定：gyro, front, R1, R2, back, L2, L1
        """
        try:
            chunk = self.ser.read_all()
        except Exception as e:
            print("[UnifiedIO] Pico serial read error:", e)
            return

        if not chunk:
            return

        self._serial_rx_buffer += chunk
        parts = self._serial_rx_buffer.split(b"\n")
        self._serial_rx_buffer = parts[-1]  # 最后一段可能是半帧，留到下一轮

        latest_valid = None
        for raw_line in parts[:-1]:
            line = raw_line.strip()
            if not line:
                continue
            try:
                nums = [float(x.strip()) for x in line.decode("utf-8", errors="ignore").split(",")]
            except Exception:
                continue

            # 比赛正式版只接受完整 7 字段帧，避免损坏的 6 字段被错位解析。
            if len(nums) != 7:
                continue

            angle = -nums[0]
            if angle >= 0:
                angle *= gyro_offset_cw
            else:
                angle *= gyro_offset_ccw

            latest_valid = {
                "angle": angle,
                "angle_valid": True,
                "front": nums[1],
                "R1": nums[2],
                "R2": nums[3],
                "back": nums[4],
                "L2": nums[5],
                "L1": nums[6],
            }

        if latest_valid is not None:
            with self.sensor_lock:
                self.latest_data = latest_valid

    # ---------- DXL Steering / Throttle：全部在同一线程 ----------
    def _write_steering_immediate(self, angle):
        angle = max(min(float(angle) * ANGLE_OFFSET, 1.0), -1.0)
        steer_deg = angle * DXL_MAX_STEER_DEG
        pos = int(DXL_STEER_CENTER_POS + dxl_deg_to_unit(steer_deg))

        if pos == self.last_steer_pos:
            return

        comm_result, dxl_error = self.packet.write4ByteTxRx(
            self.port, ID_STEER, ADDR_GOAL_POSITION, pos
        )
        if dxl_comm_ok(
            self.packet, comm_result, dxl_error,
            "Steering Goal Position", ID_STEER
        ):
            self.last_steer_pos = pos
        else:
            print("[UnifiedIO] steering command failed")

    def _write_throttle_immediate(self, throttle):
        throttle = max(min(float(throttle), 1.0), -1.0)
        target_rpm = throttle * self.max_rpm
        velocity_value = int(target_rpm / DXL_VELOCITY_UNIT_RPM)

        if velocity_value == self.last_velocity_value:
            return

        comm_result, dxl_error = self.packet.write4ByteTxRx(
            self.port,
            ID_THROTTLE,
            ADDR_GOAL_VELOCITY,
            dxl_to_uint32(velocity_value)
        )
        if dxl_comm_ok(
            self.packet, comm_result, dxl_error,
            "Throttle Goal Velocity", ID_THROTTLE
        ):
            self.last_velocity_value = velocity_value
        else:
            print("[UnifiedIO] throttle command failed")

    def _update_position(self):
        raw, comm_result, dxl_error = self.packet.read4ByteTxRx(
            self.port, ID_THROTTLE, ADDR_PRESENT_POSITION
        )
        if not dxl_comm_ok(
            self.packet, comm_result, dxl_error,
            "Throttle Present Position", ID_THROTTLE
        ):
            return

        current = dxl_to_int32(raw)
        if not self._position_initialized:
            self._last_raw_position = current
            self._position_initialized = True
            return

        delta = current - self._last_raw_position
        self._last_raw_position = current
        if abs(delta) > 100000:
            return

        self._accum_motor_units += delta
        motor_degrees = self._accum_motor_units * 360.0 / 4096.0
        wheel_degrees = motor_degrees * DXL_WHEEL_REVS_PER_MOTOR_REV
        with self.state_lock:
            self.wheel_degrees = wheel_degrees

    def _loop(self):
        next_tick = time.monotonic()
        while self.running:
            # 1) Pico：先取得本周期最新完整传感器帧
            self._handle_gyro_reset_if_requested()
            self._read_latest_serial_frame()

            # 2) 从主线程只抓一次最新目标，不处理历史命令
            with self.command_lock:
                steering_cmd = self.target_steering
                throttle_cmd = self.target_throttle

            # 3) U2D2 所有操作严格串行：Steering -> Throttle -> Present Position
            try:
                self._write_steering_immediate(steering_cmd)
                self._write_throttle_immediate(throttle_cmd)
                self._update_position()
            except Exception as e:
                print("[UnifiedIO] DXL cycle error:", e)

            # 4) 绝对时间节拍，避免简单 sleep(dt) 累积漂移
            next_tick += self.loop_dt
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # 本轮超时，不补历史周期，直接从当前时间重新起算。
                next_tick = time.monotonic()

    def shutdown(self):
        if not self.running:
            return
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

        # I/O 线程停止后，再由当前线程执行一次最终安全停车。
        try:
            self._write_throttle_immediate(0.0)
        except Exception:
            pass
        try:
            self._write_steering_immediate(0.0)
        except Exception:
            pass
        print("[UnifiedIO] stopped")


class DynamixelSteering:
    """保持原 steering.run(angle) API，但不再创建独立线程。"""

    def __init__(self, io):
        self.io = io
        self.bus = io.bus
        print("[Steering/DXL] using UnifiedIO")

    def run(self, angle: float):
        self.io.set_steering(angle)

    def shutdown(self):
        self.io.set_steering(0.0)
        print("[Steering/DXL] target centered")


class DynamixelThrottle:
    """保持原 throttle.run()/get_wheel_degrees() API，但不再创建独立线程。"""

    def __init__(self, io, max_rpm=DXL_TARGET_RPM):
        self.io = io
        self.bus = io.bus
        print("[Throttle/DXL] using UnifiedIO")

    def run(self, throttle: float):
        self.io.set_throttle(throttle)

    def get_wheel_degrees(self):
        return self.io.get_wheel_degrees()

    def shutdown(self):
        self.io.set_throttle(0.0)
        print("[Throttle/DXL] target stopped")


# =====================  Camera + AI 模块  =====================

class AIPilot:
    """封装 DonkeyCar 的 KerasLinear 模型，提供 run(img) → (angle, throttle)"""

    def __init__(self, model_path: str):
        interpreter = KerasInterpreter()
        self.pilot = KerasLinear(
            interpreter=interpreter,
            input_shape=(CROP_H, CROP_W, 3)
        )
        self.pilot.load(model_path)
        print("[AI] model loaded:", model_path)

    def run(self, img):
        angle, throttle = self.pilot.run(img)
        return float(angle), float(throttle)


def init_camera():
    cam = PiCamera(image_w=CAPTURE_W, image_h=CAPTURE_H, image_d=3)
    time.sleep(2)

    cam.camera.framerate             = CAMERA_FRAMERATE
    cam.camera.exposure_mode         = PICAMERA_EXPOSURE_MODE
    cam.camera.awb_mode              = PICAMERA_AWB_MODE
    cam.camera.iso                   = PICAMERA_ISO
    cam.camera.shutter_speed         = PICAMERA_SHUTTER_SPEED
    cam.camera.exposure_compensation = PICAMERA_EXPOSURE_COMPENSATION
    cam.camera.awb_gains             = PICAMERA_AWB_GAINS

    print("[Camera] initialized")
    return cam


# =====================  CarController：封装串口 + 控制函数  =====================

class CarController:
    def __init__(self, ser, steering, throttle, camera, ai_pilot):
        self.ser = ser
        self.steering = steering
        self.throttle = throttle
        self.io = steering.io
        self.camera = camera
        self.ai_pilot = ai_pilot

        # 保存最近一次串口数据
        self.last_data = {
            # 新串口六颗ToF顺序：front, R1, R2, back, L2, L1
            # 若固件仍额外在最前方输出gyro，则也兼容7字段格式。
            "angle": 0.0,
            "angle_valid": False,
            "front": 999.0,
            "R1": 999.0,
            "R2": 999.0,
            "back": 999.0,
            "L2": 999.0,
            "L1": 999.0,
        }

    # ===================== 传感器快照：由 UnifiedIO 线程维护 =====================
    def read_sensors(self):
        self.last_data = self.io.get_sensor_data()
        return self.last_data

    def reset_gyro(self):
        self.io.request_gyro_reset()
        print("[UnifiedIO] gyro reset requested")

    def get_tof(self, data, direction):
        """
        根据统一的六颗ToF名称返回距离。

        正式名称：front, R1, R2, back, L2, L1
        为兼容旧调用，right临时映射到R1，left临时映射到L1。
        新代码建议只使用正式名称。
        """
        aliases = {
            "front": "front",
            "R1": "R1", "r1": "R1", "right": "R1",
            "R2": "R2", "r2": "R2",
            "back": "back", "rear": "back",
            "L2": "L2", "l2": "L2",
            "L1": "L1", "l1": "L1", "left": "L1",
        }
        key = aliases.get(str(direction))
        if key is None:
            raise ValueError(
                f"未知ToF方向 {direction!r}；请使用 "
                "front/R1/R2/back/L2/L1"
            )
        return data[key], key

    # =====================  功能 1：gyro_run  =====================
    def gyro_run(self,
                 target_angle_deg: float,
                 speed: float,
                 run_time: float,
                 stop_dist_cm: float,
                 direction: str = "front",
                 run_distance: float = None,   # 新增：由电机转角度控制停止（单位：度）
                 kp: float = 0.01,
                 ki: float = 0.0,
                 kd: float = 0.00):


        print(f"[gyro_run] target_angle={target_angle_deg}°, speed={speed}, time={run_time}s")

        # 初始 PID 状态
        integral = 0.0
        prev_error = 0.0
        prev_time  = time.time()
        start_time = prev_time
        next_tick = time.monotonic()
        


        # 如果传入了 run_distance，就从当前轮子角度记录一个起点
        start_deg = None
        if run_distance is not None:
            start_deg = self.throttle.get_wheel_degrees()
            if start_deg is not None:
                print(f"[gyro_run] distance mode ON, start wheel_deg={start_deg:.1f}, target_delta={run_distance:.1f}")
            else:
                print("[gyro_run] distance mode requested but wheel_deg unavailable, will ignore run_distance")
                run_distance = None  # 读不到就退化成纯时间 / ToF 停止

        
        
        

        while True:
            now = time.time()

            # 超时停止
            if now - start_time > run_time:
                print("[gyro_run] timeout stop")
                break

            # 读取串口
            data = self.read_sensors()
            angle = data["angle"]
            front = data["front"]
            R1 = data["R1"]      # 右前ToF
            R2 = data["R2"]      # 右后ToF
            back = data["back"]  # 正后ToF
            L2 = data["L2"]      # 左后ToF
            L1 = data["L1"]      # 左前ToF
            det_direction, det_name = self.get_tof(data, direction)
                
            # ToF 停车
            if det_direction < stop_dist_cm and det_direction != -1:
                print(f"[gyro_run] {det_name} ToF {det_direction:.1f}cm < {stop_dist_cm}cm, STOP")
                break
            p_target_angle_deg = target_angle_deg
            
            if speed >= 0:
                if R1 < 15 and R1 != -1:
                    p_target_angle_deg = p_target_angle_deg - (2+(18-R1)*3)
                if L1 < 10 and L1 != -1:
                    p_target_angle_deg = p_target_angle_deg + (2+(10-L1)*2)
            else:
                if R1 < 20 and R1 != -1:
                    p_target_angle_deg = p_target_angle_deg +  (2+abs((25-R1))*1.2)
                if L1 < 20 and L1 != -1:
                    p_target_angle_deg = p_target_angle_deg - (2+abs((25-L1))*1.2)
            
            
            # 4) 由轮子累计转角度决定停止（run_distance）
            if run_distance is not None and start_deg is not None:
                cur_deg = self.throttle.get_wheel_degrees()
#                 print(cur_deg)
                if cur_deg is not None:
                    delta_deg = abs(cur_deg - start_deg)
                    if delta_deg >= run_distance:
                        print(f"[gyro_run] distance stop: wheel_delta={delta_deg:.1f} >= {run_distance:.1f}")
                        break
            
            
            
            # 连续积分角度直接误差
            error = p_target_angle_deg - angle

            # PID
            dt = max(now - prev_time, 1e-3)
            integral += error * dt
            derivative = (error - prev_error) / dt

            steering_cmd = kp * error + ki * integral + kd * derivative
            steering_cmd = max(min(steering_cmd, 1.0), -1.0)
            steering_cmd = round(steering_cmd,2)

            # 倒车时反向转向
            if speed < 0:
                steering_cmd = -steering_cmd
            
            # 下发指令

            self.steering.run(steering_cmd)
            self.throttle.run(speed)

            prev_error = error
            prev_time  = now

            next_tick += CONTROL_LOOP_DT
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

        self.throttle.run(0)
        time.sleep(0.3)
        print("[gyro_run] finished.\n")



#=========gyro run tof
    # =====================  功能 1：gyro_run  =====================
    def gyro_run_tof(self,
                 target_angle_deg: float,
                 speed: float,
                 run_time: float,
                 stop_dist_cm: float,
                 direction: str = "front",
                 run_distance: float = None,   # 新增：由电机转角度控制停止（单位：度）
                 kp: float = 0.015,
                 ki: float = 0.0,
                 kd: float = 0.005):


        print(f"[gyro_run] target_angle={target_angle_deg}°, speed={speed}, time={run_time}s")

        # 初始 PID 状态
        integral = 0.0
        prev_error = 0.0
        prev_time  = time.time()
        start_time = prev_time
        next_tick = time.monotonic()
        


        # 如果传入了 run_distance，就从当前轮子角度记录一个起点
        start_deg = None
        if run_distance is not None:
            start_deg = self.throttle.get_wheel_degrees()
            if start_deg is not None:
                print(f"[gyro_run] distance mode ON, start wheel_deg={start_deg:.1f}, target_delta={run_distance:.1f}")
            else:
                print("[gyro_run] distance mode requested but wheel_deg unavailable, will ignore run_distance")
                run_distance = None  # 读不到就退化成纯时间 / ToF 停止

        
        
        

        while True:
            now = time.time()

            # 超时停止
            if now - start_time > run_time:
                print("[gyro_run] timeout stop")
                break

            # 读取串口
            data = self.read_sensors()
            angle = data["angle"]
            front = data["front"]
            R1 = data["R1"]      # 右前ToF
            R2 = data["R2"]      # 右后ToF
            back = data["back"]  # 正后ToF
            L2 = data["L2"]      # 左后ToF
            L1 = data["L1"]      # 左前ToF
            det_direction, det_name = self.get_tof(data, direction)
                
            # ToF 停车
            if det_direction < stop_dist_cm and det_direction != -1:
                print(f"[gyro_run_tof] {det_name} ToF {det_direction:.1f}cm < {stop_dist_cm}cm, STOP")
                break
            p_target_angle_deg = target_angle_deg
            
            if speed >= 0:
                if R1 < 15 and R1 != -1:
                    p_target_angle_deg = p_target_angle_deg - (2+(18-R1)*3)
                if L1 < 10 and L1 != -1:
                    p_target_angle_deg = p_target_angle_deg + (2+(10-L1)*2)
            else:
                if direction in ("L1", "l1", "left"):
                
                    if det_direction < 25 and det_direction!=-1:
                        p_target_angle_deg = p_target_angle_deg - (2+abs((25-L1))*1.2)
                    if det_direction > 28 and det_direction!=-1:
                        p_target_angle_deg = p_target_angle_deg + (2+abs((28-L1))*1.2)
                        
                if direction in ("R1", "r1", "right"):
                
                    if det_direction < 24 and det_direction!=-1:
                        p_target_angle_deg = p_target_angle_deg + (2+abs((24-R1))*1.2)
                    if det_direction > 28 and det_direction!=-1:
                        p_target_angle_deg = p_target_angle_deg - (2+abs((28-R1))*1.2)

                
                
            
            
            # 4) 由轮子累计转角度决定停止（run_distance）
            if run_distance is not None and start_deg is not None:
                cur_deg = self.throttle.get_wheel_degrees()
                print(cur_deg)
                if cur_deg is not None:
                    delta_deg = abs(cur_deg - start_deg)
                    if delta_deg >= run_distance:
                        print(f"[gyro_run] distance stop: wheel_delta={delta_deg:.1f} >= {run_distance:.1f}")
                        break
            
            
            
            # 连续积分角度直接误差
            error = p_target_angle_deg - angle

            # PID
            dt = max(now - prev_time, 1e-3)
            integral += error * dt
            derivative = (error - prev_error) / dt

            steering_cmd = kp * error + ki * integral + kd * derivative
            steering_cmd = max(min(steering_cmd, 1.0), -1.0)
            steering_cmd = round(steering_cmd,2)

            # 倒车时反向转向
            if speed < 0:
                steering_cmd = -steering_cmd
            
            # 下发指令

            self.steering.run(steering_cmd)
            self.throttle.run(speed)
            print("det_direction",det_direction,"p_target_angle_deg",p_target_angle_deg,"steering_cmd",steering_cmd)
            prev_error = error
            prev_time  = now

            next_tick += CONTROL_LOOP_DT
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

        self.throttle.run(0)
        time.sleep(0.3)
        print("[gyro_run] finished.\n")








    # =====================  功能 2：gyro_run  =====================
    def motor_run(self,
                 target_steering: float,
                 speed: float,
                 run_time: float,
                 stop_dist_cm: float,
                 direction: str = "front",
                 run_distance: float = None)  :# 新增：由电机转角度控制停止（单位：度）



        print(f"[motor_run] target_sterint={target_steering}, speed={speed}, time={run_time}s")
        prev_time  = time.time()
        start_time = prev_time
        next_tick = time.monotonic()
        


        # 如果传入了 run_distance，就从当前轮子角度记录一个起点
        start_deg = None
        if run_distance is not None:
            start_deg = self.throttle.get_wheel_degrees()
            if start_deg is not None:
                print(f"[gyro_run] distance mode ON, start wheel_deg={start_deg:.1f}, target_delta={run_distance:.1f}")
            else:
                print("[gyro_run] distance mode requested but wheel_deg unavailable, will ignore run_distance")
                run_distance = None  # 读不到就退化成纯时间 / ToF 停止

        
        
        

        while True:
            now = time.time()

            # 超时停止
            if now - start_time > run_time:
                print("[motor_run] timeout stop")
                break

            # 读取串口
            data = self.read_sensors()
            angle = data["angle"]
            front = data["front"]
            R1 = data["R1"]      # 右前ToF
            R2 = data["R2"]      # 右后ToF
            back = data["back"]  # 正后ToF
            L2 = data["L2"]      # 左后ToF
            L1 = data["L1"]      # 左前ToF
            det_direction, det_name = self.get_tof(data, direction)
                
            # ToF 停车
            if det_direction < stop_dist_cm and det_direction != -1:
                print(f"[motor_run] {det_name} ToF {det_direction:.1f}cm < {stop_dist_cm}cm, STOP")
                break

            

            
            
            # 4) 由轮子累计转角度决定停止（run_distance）
            if run_distance is not None and start_deg is not None:
                cur_deg = self.throttle.get_wheel_degrees()
                print(cur_deg)
                if cur_deg is not None:
                    delta_deg = abs(cur_deg - start_deg)
                    if delta_deg >= run_distance:
                        print(f"[gyro_run] distance stop: wheel_delta={delta_deg:.1f} >= {run_distance:.1f}")
                        break
            
            

            steering_cmd = target_steering
            
            # 下发指令

            self.steering.run(steering_cmd)
            self.throttle.run(speed)


            prev_time  = now

            next_tick += CONTROL_LOOP_DT
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

        self.throttle.run(0)
        time.sleep(0.3)
        print("[motor_run] finished.\n")



        
    # =====================  功能 3：gyro_turn  =====================
    def gyro_turn(self,
                  steer_cmd: float,
                  speed: float,
                  target_angle_deg: float,
                  max_time: float = 5.0):
        """
        gyro_turn(转向命令, 速度命令, 目标角度, 最大时间)

        - steer_cmd: 转向控制（-1 ~ 1），负值=左打，正值=右打
        - speed:     速度控制（-1 ~ 1），正值=前进，负值=倒车
        - target_angle_deg: 期望的绝对陀螺角度（连续积分值，如 -45, 120, 450 等）
        - max_time:  安全超时，防止死循环

        方向约定（按照你给的要求）：
        1) 左转 / 逆时针：
           - steer_cmd < 0 且 speed > 0   （前进左转）
           - steer_cmd > 0 且 speed < 0   （倒车右转 → 车头实际向左偏）
           判定条件：当前角度 > 目标角度 时允许继续转动；
                   一旦 当前角度 <= 目标角度，就停止并回正方向盘。

        2) 右转 / 顺时针：
           - steer_cmd > 0 且 speed > 0   （前进右转）
           - steer_cmd < 0 且 speed < 0   （倒车左转 → 车头实际向右偏）
           判定条件：当前角度 < 目标角度 时允许继续转动；
                   一旦 当前角度 >= 目标角度，就停止并回正方向盘。
        """

        # 剪裁输入范围，避免异常
        steer_cmd = max(min(steer_cmd, 1.0), -1.0)
        speed     = max(min(speed, 1.0), -1.0)

        if steer_cmd == 0 or speed == 0:
            print("[gyro_turn] steer_cmd 或 speed 为 0，无法转弯，直接返回")
            return

        # 判断转向方向：左（逆时针）还是右（顺时针）
        # 左：steer<0,speed>0 或 steer>0,speed<0  → 符号相反
        # 右：steer>0,speed>0 或 steer<0,speed<0  → 符号相同
        if (steer_cmd < 0 and speed > 0) or (steer_cmd > 0 and speed < 0):
            turn_dir = "left"   # 逆时针
        else:
            turn_dir = "right"  # 顺时针

        print(f"[gyro_turn] start: steer={steer_cmd:.2f}, speed={speed:.2f}, "
              f"target_angle={target_angle_deg:.1f}°, dir={turn_dir}")

        # 初始角度
        data = self.read_sensors()
        start_angle = data["angle"]
        print(f"[gyro_turn] current angle={start_angle:.1f}°")

        start_time = time.time()
        next_tick = time.monotonic()

        # 下发初始转向 / 速度
        self.steering.run(steer_cmd)
        self.throttle.run(speed)

        while True:
            now = time.time()
            if now - start_time > max_time:
                print(f"[gyro_turn] timeout {max_time:.1f}s, STOP")
                break

            data = self.read_sensors()
            angle = data["angle"]

            # 根据转向方向决定停止条件
            if turn_dir == "left":
                # 希望从当前角度往“更小”的目标角度旋转
                # 当前角度 <= 目标角度 → 说明已经转到或越过目标
                if angle <= target_angle_deg:
                    print(f"[gyro_turn] LEFT reached: angle={angle:.1f}° <= target={target_angle_deg:.1f}°")
                    break
            else:  # right
                # 希望从当前角度往“更大”的目标角度旋转
                # 当前角度 >= 目标角度 → 说明已经转到或越过目标
                if angle >= target_angle_deg:
                    print(f"[gyro_turn] RIGHT reached: angle={angle:.1f}° >= target={target_angle_deg:.1f}°")
                    break

            # 保持转弯命令（防止中途被其它地方改写）
            self.steering.run(steer_cmd)
            self.throttle.run(speed)

            next_tick += CONTROL_LOOP_DT
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

        # 到这里说明：达到目标角度 或 超时
        # → 停车 + 回正方向盘
        self.throttle.run(0.0)
        time.sleep(0.2)
        self.steering.run(0.0)   # 回正
        time.sleep(0.2)

        print("[gyro_turn] finished.\n")
        

    

    # =====================  功能 2：ai_run  =====================
    def ai_run(self,
               stop_delta_deg: float = 1080.0,
               max_speed: float = AI_MAX_SPEED_DEFAULT,
               loop_hz: float = AI_LOOP_HZ):

        loop_hz = float(loop_hz)
        target_dt = 1.0 / loop_hz
        max_speed = max(0.0, min(1.0, float(max_speed)))
        print(
            f"[ai_run] start, stop_delta={stop_delta_deg}°, "
            f"max_speed={max_speed:.2f}, target={loop_hz:.1f} Hz"
        )

        next_tick = time.monotonic()
        loop_count = 0
        total_work_time = 0.0
        max_work_time = 0.0
        report_start = time.monotonic()

        while True:
            cycle_start = time.monotonic()

            # 1. 相机取图
            raw = self.camera.run()
            img = center_crop(raw)

            # 2. AI 推理
            angle_pred, throttle_pred = self.ai_pilot.run(img)
            angle_pred = angle_pred * AI_RUN_ANGLE_OFFSET

            # 3. AI 原始 throttle_pred 保持不变；max_speed 只限制实际电机速度。
            #    例如 throttle_pred=+1.0, max_speed=0.40 -> 实际执行 +0.40
            #    在 DXL_TARGET_RPM=100 时约等于 +40 RPM。
            drive_throttle = max(-1.0, min(1.0, float(throttle_pred))) * max_speed

            # 只更新最新目标；UnifiedIO 线程负责实际 U2D2 下发
            self.steering.run(angle_pred)
            self.throttle.run(drive_throttle)

            # 4. 从 UnifiedIO 获取最新 gyro 快照并判断停止
            data = self.read_sensors()
            angle = data["angle"]

            if stop_delta_deg >= 0:
                if angle >= stop_delta_deg:
                    print("[ai_run] stop delta reached")
                    break
            else:
                if angle <= stop_delta_deg:
                    print("[ai_run] stop delta reached")
                    break

            work_time = time.monotonic() - cycle_start
            loop_count += 1
            total_work_time += work_time
            max_work_time = max(max_work_time, work_time)

            # 每约 5 秒报告一次真实计算耗时和实际循环频率，便于 Pi 5 实测。
            report_now = time.monotonic()
            if report_now - report_start >= 5.0 and loop_count > 0:
                avg_work_ms = total_work_time / loop_count * 1000.0
                max_work_ms = max_work_time * 1000.0
                actual_hz = loop_count / (report_now - report_start)
                print(
                    f"[ai_run PERF] target={loop_hz:.1f}Hz, actual={actual_hz:.1f}Hz, "
                    f"avg_work={avg_work_ms:.1f}ms, max_work={max_work_ms:.1f}ms"
                )
                report_start = report_now
                loop_count = 0
                total_work_time = 0.0
                max_work_time = 0.0

            # 固定周期节拍：计算用了多少时间，只睡剩余时间。
            next_tick += target_dt
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # AI 本轮超过目标周期，不补历史帧，立即从当前时间重新起算。
                next_tick = time.monotonic()

        self.throttle.run(0)
        time.sleep(0.3)
        print("[ai_run] finished.\n")
        

    # 定义 --------- shutdown-----------
    def shutdown(self):
        print("[CarController] Shutdown called. Stopping motors...")

        try:
            self.throttle.run(0)
        except:
            pass

        try:
            time.sleep(0.2)
            self.steering.run(0)
        except:
            pass

        try:
            self.throttle.shutdown()
            self.steering.shutdown()
        except:
            pass

        # 统一 I/O 线程先退出，再关闭 U2D2 与 Pico 串口。
        try:
            self.io.shutdown()
        except:
            pass

        try:
            self.throttle.bus.close()
        except:
            pass

        try:
            self.ser.close()
        except:
            pass
        
        try:
            sense.clear()
        except:
            pass

        print("[CarController] All motors stopped and serial closed.")


        
        
        


def read_sensors_once(io, last_data=None):
    """兼容旧调用名称；实际直接读取 UnifiedIO 的最新传感器快照。"""
    return io.get_sensor_data()


def detect_run_clockwise(io, sense, sample_count=5):
    print("start detect. see the sense hat led")
    """
    使用 ToF 左右距离判断赛道是 CW 还是 CCW。
    Pico 数据由 UnifiedIO 后台线程持续维护；这里不再直接碰 USB Serial。
    joystick 中键按住时，每 1 秒切换一次方向，松开后确认。
    """
    L1_vals = []
    R1_vals = []
    auto_dir = "CW"
    holding = False
    press_time = None
    last_toggle_time = None
    confirmed_dir = None

    def show_dir_led(direction: str):
        sense.clear()
        if direction == "CCW":
            sense.set_pixel(0, 7, 255, 255, 255)
        elif direction == "CW":
            sense.set_pixel(7, 7, 255, 255, 255)

    io.request_gyro_reset()
    print("[UnifiedIO] gyro reset requested before direction detect")
    time.sleep(4)

    while True:
        now = time.time()

        if not holding:
            last_data = io.get_sensor_data()
            l = last_data["L1"]
            r = last_data["R1"]
            g = last_data["angle"]
            gyro_available = last_data.get("angle_valid", False)

            if gyro_available and (-0.5 < g < 0.5) and l == 999 and r == 999:
                sense.set_pixel(3, 3, 255, 0, 0)
                io.request_gyro_reset()
                print("[UnifiedIO] reset gyro requested: no ToF data")
                time.sleep(4)

            if gyro_available and (not (-0.5 < g < 0.5)):
                sense.set_pixel(3, 3, 0, 0, 255)
                io.request_gyro_reset()
                print("[UnifiedIO] reset gyro requested: angle not near zero")
                time.sleep(4)
            elif gyro_available:
                sense.set_pixel(3, 3, 0, 255, 0)
                time.sleep(0.01)
            else:
                sense.set_pixel(3, 3, 255, 255, 0)

            if l == -1:
                l = 100.0
            if r == -1:
                r = 100.0

            L1_vals.append(l)
            R1_vals.append(r)
            if len(L1_vals) > sample_count:
                L1_vals.pop(0)
                R1_vals.pop(0)

            avg_left = sum(L1_vals) / len(L1_vals)
            avg_right = sum(R1_vals) / len(R1_vals)
            auto_dir = "CCW" if avg_right < avg_left else "CW"
            show_dir_led(auto_dir)

        for event in sense.stick.get_events():
            if event.direction == "middle":
                if event.action == "pressed":
                    holding = True
                    press_time = now
                    last_toggle_time = now
                    confirmed_dir = auto_dir
                    show_dir_led(confirmed_dir)
                elif event.action == "released" and holding:
                    holding = False
                    duration = now - press_time if press_time is not None else 0.0
                    print(f"[Detect] joystick released, duration={duration:.2f}s, final={confirmed_dir}")
                    return confirmed_dir

        if holding:
            now = time.time()
            if now - last_toggle_time >= 1.0:
                confirmed_dir = "CCW" if confirmed_dir == "CW" else "CW"
                last_toggle_time = now
                show_dir_led(confirmed_dir)

        time.sleep(0.5)

# =====================  main：调用顺序示例  =====================

def main():
    global car
    # 串口
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.05)
    print("[Serial] opened:", ser.name)
    ser.reset_input_buffer()

    # 硬件初始化
    # Dynamixel 共用 U2D2 总线；统一 I/O 线程同时维护 Pico + ID2 + ID1。
    dxl_bus = DynamixelBus()
    robot_io = UnifiedRobotIO(ser, dxl_bus, loop_hz=IO_LOOP_HZ)
    steering = DynamixelSteering(robot_io)
    throttle = DynamixelThrottle(robot_io)
    camera = init_camera()
    
    # Sense HAT（用于显示 & Joystick）
    sense = SenseHat()

    # ========= 第一步：方向检测（CW / CCW） =========
    Run_Clockwise = detect_run_clockwise(robot_io, sense, sample_count=5)

    # ========= 第二步：根据结果选择模型路径 =========
    if Run_Clockwise == "CCW":
        MODEL_PATH_DEFAULT = MODEL_PATH_CCW
    else:
        MODEL_PATH_DEFAULT = MODEL_PATH_CW

    print(f"[Main] Run_Clockwise={Run_Clockwise}, model={MODEL_PATH_DEFAULT}")    
    
    
    ai_pilot = AIPilot(MODEL_PATH_DEFAULT)   # 或者替换成你的模型路径

    car = CarController(ser, steering, throttle, camera, ai_pilot)

    # 这里就是 WRO 流程示例：
    # 1) 先用 gyro_run 做两段小动作
    car.reset_gyro()
    
    time.sleep(4)
    
#=====part1: start from parking lot========
    if Run_Clockwise == "CCW":        
        car.gyro_turn(steer_cmd=-1,speed=0.15, target_angle_deg=-45,max_time=20.0)
        car.gyro_turn(steer_cmd= 1,speed=0.15, target_angle_deg=-30,max_time=20.0)
        car.gyro_run(target_angle_deg=0, speed=-0.2,  run_time=3, stop_dist_cm = 20, run_distance = 10000.0,direction = "front")
        
    else: 
        car.gyro_turn(steer_cmd= 1,speed=0.2, target_angle_deg=70,max_time=20.0)
        car.gyro_turn(steer_cmd=-1,speed=0.2, target_angle_deg=15,max_time=20.0)
        
        #car.gyro_run(target_angle_deg=0, speed=-1,  run_time=5.5, stop_dist_cm = 5, run_distance = 10000.0,direction = "front")
#            
#     

#=====part2:AI run 3 laps===========    
    if Run_Clockwise == "CCW":
        car.ai_run(stop_delta_deg= -1125)
        
    else:
        car.ai_run(stop_delta_deg= 1110)
#         
#     data=car.read_sensors()
#     print(data)
# 
#         
# #====part3: parking at parking lot======
#     if Run_Clockwise == "CCW":
#         stand_degree = -1170
#         car.gyro_run(target_angle_deg=stand_degree+5, speed=-1,  run_time=8, stop_dist_cm = 5, run_distance = 10000.0,direction = "front")
#         car.gyro_run(target_angle_deg=stand_degree, speed=0.5,  run_time=10, stop_dist_cm = 5, run_distance = 500,direction = "front")
# 
#         car.gyro_turn(steer_cmd=1,speed=0.7, target_angle_deg=stand_degree+25,max_time=3.0)
#         car.gyro_turn(steer_cmd=-1,speed=-0.7, target_angle_deg=stand_degree+70,max_time=3.0)
#         
# 
#         car.gyro_run_tof(target_angle_deg=stand_degree+90, speed=-0.5,  run_time=10000, stop_dist_cm = 15, run_distance = 100000.0,direction = "R1")
# 
#         car.gyro_run(target_angle_deg=stand_degree+90, speed= 0.5,  run_time=20, stop_dist_cm = 5, run_distance = 410.0,direction = "front") 
#         car.gyro_turn(steer_cmd=1,speed=-0.3, target_angle_deg=stand_degree+25,max_time=10.0)
#         car.gyro_run(target_angle_deg=stand_degree+30, speed=-0.3,  run_time=20, stop_dist_cm = 20, run_distance = 20.0,direction = "L1")
#         car.gyro_turn(steer_cmd=-1,speed=-0.3, target_angle_deg=stand_degree+55,max_time=5.0)
#         car.motor_run(target_steering = 1, speed= 0.4,run_time=5,stop_dist_cm = 1,direction = "R1",run_distance = 160)
# 
# # #     
# # # 
#     else:
#         stand_degree = 1170
# 
#         car.gyro_run(target_angle_deg=stand_degree, speed=-1,  run_time=8, stop_dist_cm = 5, run_distance = 10000.0,direction = "front")
#         car.gyro_run(target_angle_deg=stand_degree, speed=0.5,  run_time=10, stop_dist_cm = 5, run_distance = 480,direction = "front")
# 
#         car.gyro_turn(steer_cmd=-1,speed=0.7, target_angle_deg=stand_degree-25,max_time=3.0)
#         car.gyro_turn(steer_cmd=1,speed=-0.7, target_angle_deg=stand_degree-70,max_time=3.0)
#         
#         car.gyro_run_tof(target_angle_deg=stand_degree-90, speed=-0.5,  run_time=10000, stop_dist_cm = 15, run_distance = 100000.0,direction = "L1")
# 
#         car.gyro_run(target_angle_deg=stand_degree-90, speed= 0.5,  run_time=20, stop_dist_cm = 5, run_distance = 410.0,direction = "front") 
#         car.gyro_turn(steer_cmd=-1,speed=-0.3, target_angle_deg=stand_degree-25,max_time=10.0)
#         car.gyro_run(target_angle_deg=stand_degree-30, speed=-0.3,  run_time=20, stop_dist_cm = 20, run_distance = 20.0,direction = "L1")
#         car.gyro_turn(steer_cmd=1,speed=-0.3, target_angle_deg=stand_degree-55,max_time=5.0)
#         
#         car.motor_run(target_steering = -1, speed= 0.4,run_time=5,stop_dist_cm = 1,direction = "R1",run_distance = 160)






#     while True:    
#         data = car.read_sensors()
#         print(data["angle"])
#         time.sleep(0.1)     


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[System] Ctrl + C detected! Attempting graceful shutdown...")

        try:
            car.shutdown()  # 停车 + 回正 + 结束线程
        except:
            print("[System] car object not initialized or already closed.")

        print("[System] Program terminated safely.")
        