#!/usr/bin/env python3
"""
AFI CUP - CW / CCW Parking 外部运行流程 V01

使用方法：
1. AFI_CUP_V05f.py 和本文件放在同一个文件夹。
2. 平时主要修改本文件的 RUN PROGRAM 区域。
3. AFI_CUP_V05f.py 作为底层功能库，尽量不要频繁修改。

当前底层功能例如：
    car.gyro_run(...)                 # 纯 Gyro，不受 ToF 干扰
    car.gyro_run_touch_wall(...)      # 原 Gyro Run，带左右近墙保护
    car.gyro_single_tof_wall_run(...) # 单后侧 ToF 墙距 + Gyro + 前侧 Parking
    car.gyro_run_tof(...)
    car.motor_run(...)
    car.gyro_turn(...)
    car.ai_run(...)
"""

import time
import serial
from sense_hat import SenseHat

import AFI_CUP_V05f as R


def build_robot():
    """
    只负责完成硬件与模型初始化。
    返回：
        car
        sense
        run_clockwise
    """

    # ---------- Pico Serial ----------
    ser = serial.Serial(R.SERIAL_PORT, R.SERIAL_BAUD, timeout=0.05)
    print("[Serial] opened:", ser.name)
    ser.reset_input_buffer()

    # ---------- Dynamixel / Unified I/O ----------
    dxl_bus = R.DynamixelBus()
    robot_io = R.UnifiedRobotIO(ser, dxl_bus, loop_hz=R.IO_LOOP_HZ)
    steering = R.DynamixelSteering(robot_io)
    throttle = R.DynamixelThrottle(robot_io)

    # ---------- Camera ----------
    camera = R.init_camera()

    # ---------- Sense HAT ----------
    sense = SenseHat()

    # ---------- Detect CW / CCW ----------
    run_clockwise = R.detect_run_clockwise(
        robot_io,
        sense,
        sample_count=5
    )

    # ---------- Select AI model ----------
    if run_clockwise == "CCW":
        model_path = R.MODEL_PATH_CCW
    else:
        model_path = R.MODEL_PATH_CW

    print(f"[Main] Run_Clockwise={run_clockwise}, model={model_path}")

    ai_pilot = R.AIPilot(model_path)

    car = R.CarController(
        ser,
        steering,
        throttle,
        camera,
        ai_pilot
    )

    return car, sense, run_clockwise


def run_program(car, sense, run_clockwise):
    """
    ============================================================
    AFI CUP Parking Run

    CCW:
      - 右侧贴墙
      - R2 作为对应右侧 ToF 流程
      - 原有动作逻辑保持不变

    CW:
      - 完整镜像 CCW
      - 右墙 -> 左墙
      - R2 -> L2
      - 左右 Steering 符号全部反转
      - Gyro 目标角度符号全部反转
      - 前进/后退速度符号保持原动作含义不变
      - front/back 停止方向保持不变
    ============================================================
    """

    car.reset_gyro()
    time.sleep(4)

    if run_clockwise == "CCW":

        car.ai_run(stop_delta_deg = -1060)
        car.gyro_run(
            target_angle_deg=-1080,
            speed=0.4,
            run_time=2.5,
            run_distance=2000
        )
    else:
        car.ai_run(stop_delta_deg = 1060)
        car.gyro_run(
            target_angle_deg=1080,
            speed=0.4,
            run_time=2.5,
            run_distance=2000
        )
            
        
        



def main():
    car = None
    sense = None

    try:
        car, sense, run_clockwise = build_robot()

        # 真正比赛/测试流程
        run_program(car, sense, run_clockwise)

    except KeyboardInterrupt:
        print("\n[System] Ctrl+C detected.")

    finally:
        if car is not None:
            try:
                car.shutdown()
            except Exception as e:
                print("[System] car shutdown error:", e)

        if sense is not None:
            try:
                sense.clear()
            except Exception:
                pass

        print("[System] Program finished safely.")


if __name__ == "__main__":
    main()
