#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
# 导入你的底层程序（注意这里不带 .py 后缀）
import Run_BY_Gyro_02 

def main_mission():
    robot = None
    try:
        print("====== 1. 初始化机器人硬件 ======")
        # 调用底层的初始化函数，获取硬件控制句柄 (robot 字典)
        robot = robot_controller.initialize_robot()

        print("====== 2. 开始执行自定义任务 ======")
        
        # 案例 A：调用原汁原味的按距离运行 (不用陀螺仪)
        print("\n--- 动作 A: 直行 20cm ---")
        robot_controller.runForDistanceByToF(
            robot=robot, 
            steering=0, 
            speed=15, 
            distance_cm=20, 
            keep_distance_cm=8
        )
        time.sleep(1)

        # 案例 B：调用你刚刚添加的 runForDistanceByToF_Gyro
        # 带有固定的 steering，且一旦达到目标角度就提前停车
        print("\n--- 动作 B: 右转并行驶，达到 45度 或 30cm 停止 ---")
        robot_controller.runForDistanceByToF_Gyro(
            robot=robot,
            steering=1,             # 向右打满
            speed=20, 
            distance_cm=30, 
            keep_distance_cm=10, 
            target_angle_deg=45     # 陀螺仪达到45度停止
        )
        time.sleep(1)

        # 案例 C：调用 GyroRun_Distance (PID 自动修正方向)
        # 不需要输入 steering，车子会自动微调方向盘保持在 45度 方向行驶 50cm
        print("\n--- 动作 C: 保持 45度 航向直行 50cm ---")
        robot_controller.GyroRun_Distance(
            robot=robot,
            speed=30, 
            distance_cm=50, 
            keep_distance_cm=12, 
            target_angle_deg=45     # PID 目标航向
        )
        time.sleep(1)
        
        # 案例 D：调用功能二 (沿墙行驶检测停车区)
        print("\n--- 动作 D: 沿墙寻找停车区 ---")
        robot_controller.detect_parking_lot(
            robot=robot, 
            speed=15
        )

        print("\n====== 任务全部完成！ ======")

    except KeyboardInterrupt:
        # 捕获 Ctrl+C，防止程序直接崩溃导致电机失控
        print("\n[中断] 用户手动停止了任务。")
        
    except Exception as e:
        # 捕获其他未知报错
        print(f"\n[错误] 程序运行发生异常: {e}")

    finally:
        # 【最重要的一步】: 无论任务是正常结束、报错还是被强退，
        # 都必须调用底层的 safe_shutdown_robot 来安全关闭电机扭矩和串口。
        print("\n====== 3. 执行硬件安全清理 ======")
        robot_controller.safe_shutdown_robot(robot, reason="上层任务运行结束")


if __name__ == "__main__":
    main_mission()