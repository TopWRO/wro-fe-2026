#!/usr/bin/env python3
"""
AFI CUP - 外部运行流程文件 V01

使用方法：
1. AFI_CUP_V02.py 和本文件放在同一个文件夹。
2. 平时主要修改本文件的 RUN PROGRAM 区域。
3. AFI_CUP_V02.py 作为底层功能库，尽量不要频繁修改。

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

import AFI_CUP_V05 as R


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
    以后你主要修改这里。
    ============================================================
    """

    # ---------- Gyro reset ----------
    car.reset_gyro()
    time.sleep(4)

    # ==========================================================
    # DEMO 1：纯 Gyro Run
    # 完全不使用 ToF。
    # time / distance 哪个先到，哪个停止。
    # ==========================================================
    car.gyro_single_tof_wall_run(target_angle_deg=0,speed=0.15,wall_side="right",wall_target_cm=26,run_time=20,run_distance=20000)   
    
    car.gyro_run(target_angle_deg=0,speed=0.15,run_time=10,run_distance=500)
 
    time.sleep(0)
    
    car.gyro_run_tof(target_angle_deg=0, speed=0.15,  run_time=10000, stop_dist_cm = 15, run_distance = 100000.0,direction = "R2")
    
    
    

    car.gyro_turn(steer_cmd=1,speed=-0.1, target_angle_deg=-78,max_time=20.0)
    
    car.motor_run(target_steering = 0, speed= -0.07,run_time=10,stop_dist_cm = 6,direction = "back",run_distance = 600)
#     car.gyro_run(target_angle_deg=-90,speed= -0.1,run_time= 3 ,run_distance=500)
    
    
    
    car.gyro_run(target_angle_deg=-90,speed= 0.1,run_time=10,run_distance=500)
    
    car.motor_run(target_steering = -1, speed= -0.08,run_time=10,stop_dist_cm = 10,direction = "back",run_distance = 600)
    
    car.motor_run(target_steering = 1, speed= 0.08,run_time=10,stop_dist_cm = 7,direction = "front",run_distance = 155)
    
#     car.gyro_turn(steer_cmd=-1,speed=-0.1, target_angle_deg=-5,max_time=20.0)
    
    car.motor_run(target_steering = -1, speed= -0.08,run_time=10,stop_dist_cm = 7,direction = "back",run_distance = 160)
    
    time.sleep(60)
    # ==========================================================
    # DEMO 2：单 ToF 墙距运行
    # 前进右墙：
    #   R2 = 墙距
    #   R1 = Parking
    #
    # 倒车时会自动交换：
    #   R1 = 墙距
    #   R2 = Parking
    # ==========================================================
    # car.gyro_single_tof_wall_run(
    #     target_angle_deg=0,
    #     speed=0.15,
    #     wall_side="right",
    #     wall_target_cm=26,
    #     run_time=20,
    #     run_distance=20000
    # )

    # ==========================================================
    # DEMO 3：Gyro Turn
    # ==========================================================
    # car.gyro_turn(
    #     steer_cmd=-1,
    #     speed=0.15,
    #     target_angle_deg=-45,
    #     max_time=10
    # )

    # ==========================================================
    # DEMO 4：AI Run
    # ==========================================================
    # if run_clockwise == "CCW":
    #     car.ai_run(stop_delta_deg=-1125)
    # else:
    #     car.ai_run(stop_delta_deg=1110)


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
