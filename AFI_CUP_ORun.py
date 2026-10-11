#!/usr/bin/env python3
"""
AFI CUP - CW / CCW Parking 外部运行流程 V01

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
        car.gyro_turn(steer_cmd=-1,speed=0.15, target_angle_deg= -40,max_time=20.0)
        
        car.gyro_run(
            target_angle_deg=-40,
            speed=0.15,
            run_time=2,
            run_distance=3000
        )
        
        
        car.gyro_turn(steer_cmd=1,speed=0.15, target_angle_deg= -20,max_time=20.0)
        car.gyro_run(
            target_angle_deg=5,
            speed=-0.3,
            run_time=4,
            run_distance=3000
        )
        car.ai_run(stop_delta_deg = -1080)
        
        
        car.gyro_run_tof(
            target_angle_deg=-1080,
            speed=-0.3,
            run_time=10000,
            stop_dist_cm= 45,
            run_distance=100000.0,
            direction="back")
        
        car.gyro_run(
            target_angle_deg=-1080,
            speed=0.15,
            run_time=3,
            run_distance=500
        )

        
        
        
        car.gyro_turn(
            steer_cmd=1,
            speed=-0.15,
            target_angle_deg= (-1080-90),
            max_time=20.0
        )
        
        car.calibrate_gyro_from_side_wall(wall_angle_deg = -1170)

        car.gyro_run_tof(
            target_angle_deg=(-1080-90),
            speed=-0.2,
            run_time=10000,
            stop_dist_cm= 15 ,
            run_distance=100000.0,
            direction="back")
        
        car.gyro_run(
            target_angle_deg=-1170,
            speed=0.15,
            run_time=2,
            run_distance=500
        )
        
        
        car.gyro_turn(
            steer_cmd=1,
            speed=0.15,
            target_angle_deg= (-1080-15),
            max_time=20.0
        )
        
        

        
        car.calibrate_gyro_from_side_wall(wall_angle_deg = -1080)

        
        print("[RUN] CCW Parking Program: RIGHT wall")

        car.gyro_single_tof_wall_run(
            target_angle_deg=-1080,
            speed=0.15,
            wall_side="right",
            wall_target_cm=32,
            run_time=20,
            run_distance=20000
        )

        car.gyro_run(
            target_angle_deg=-1080,
            speed=0.15,
            run_time=10,
            run_distance=500
        )

        car.gyro_run_tof(
            target_angle_deg=-1080,
            speed=0.15,
            run_time=10000,
            stop_dist_cm=15,
            run_distance=100000.0,
            direction="R2"
        )

        car.gyro_turn(
            steer_cmd=1,
            speed=-0.1,
            target_angle_deg=(-1080-78),
            max_time=20.0
        )

        car.motor_run(
            target_steering=0,
            speed=-0.07,
            run_time=10,
            stop_dist_cm=6,
            direction="back",
            run_distance=600
        )
        
        

        car.gyro_run(
            target_angle_deg=(-1080-90),
            speed=0.1,
            run_time=10,
            run_distance=500
        )

        car.motor_run(
            target_steering=-1,
            speed=-0.08,
            run_time=10,
            stop_dist_cm=10,
            direction="back",
            run_distance=600
        )

        car.motor_run(
            target_steering=1,
            speed=0.08,
            run_time=10,
            stop_dist_cm=7,
            direction="front",
            run_distance=155
        )

        car.motor_run(
            target_steering=-1,
            speed=-0.08,
            run_time=10,
            stop_dist_cm=7,
            direction="back",
            run_distance=160
        )

    else:
        #Start from parkinglot
        car.gyro_turn(steer_cmd=1,speed=0.15, target_angle_deg= 40,max_time=20.0)
        
        car.gyro_run(
            target_angle_deg=40,
            speed=0.15,
            run_time=1.5,
            run_distance=3000
        )
        car.gyro_turn(steer_cmd=-1,speed=0.15, target_angle_deg= 30,max_time=20.0)

        car.gyro_run(
            target_angle_deg=-
            5,
            speed=-0.3,
            run_time=4,
            run_distance=3000
        )
        #ai run
        car.ai_run(stop_delta_deg = 1080)
        
        
        
        #parking
        car.gyro_run_tof(
            target_angle_deg=1095,
            speed=-0.3,
            run_time=10000,
            stop_dist_cm= 50,#40
            run_distance=100000.0,
            direction="back")
        
        car.gyro_run(
            target_angle_deg=1080,
            speed=0.15,
            run_time=2,
            run_distance=500
        )
        
        car.gyro_turn(
            steer_cmd=-1,
            speed=-0.15,
            target_angle_deg= (1080+90),
            max_time=20.0
        )
        
        car.calibrate_gyro_from_side_wall(wall_angle_deg = 1170)

        
        car.gyro_run_tof(
            target_angle_deg=(1080+90),
            speed=-0.2,
            run_time=10000,
            stop_dist_cm= 15,
            run_distance=100000.0,
            direction="back")
        
        car.gyro_run(
            target_angle_deg=1080,
            speed=0.15,
            run_time=2,
            run_distance=500
        )
        
        car.gyro_turn(
            steer_cmd=-1,
            speed=0.15,
            target_angle_deg= (1080+15),
            max_time=20.0
        )
        
        car.calibrate_gyro_from_side_wall(wall_angle_deg = 1080)

        
        print("[RUN] CW Parking Program: LEFT wall")

        car.gyro_single_tof_wall_run(
            target_angle_deg=1080,
            speed=0.15,
            wall_side="left",
            wall_target_cm=29,
            run_time=20,
            run_distance=20000
        )

        car.gyro_run(
            target_angle_deg=1080,
            speed=0.15,
            run_time=10,
            run_distance=500
        )

        car.gyro_run_tof(
            target_angle_deg=1080,
            speed=0.15,
            run_time=10000,
            stop_dist_cm=15,
            run_distance=100000.0,
            direction="L2"
        )

        car.gyro_turn(
            steer_cmd=-1,
            speed=-0.1,
            target_angle_deg=(1080+78),
            max_time=20.0
        )

        car.motor_run(
            target_steering=0,
            speed=-0.07,
            run_time=10,
            stop_dist_cm=6,
            direction="back",
            run_distance=600
        )

        car.gyro_run(
            target_angle_deg=(1080+90),
            speed=0.1,
            run_time=10,
            run_distance=500
        )

        car.motor_run(
            target_steering=1,
            speed=-0.08,
            run_time=10,
            stop_dist_cm=10,
            direction="back",
            run_distance=600
        )

        car.motor_run(
            target_steering=-1,
            speed=0.08,
            run_time=10,
            stop_dist_cm=7,
            direction="front",
            run_distance=155
        )

        car.motor_run(
            target_steering=1,
            speed=-0.08,
            run_time=10,
            stop_dist_cm=7,
            direction="back",
            run_distance=160
        )

    time.sleep(60)



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


