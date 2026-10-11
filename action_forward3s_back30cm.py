#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import robot_lib as R


def main():
    robot = None

    try:
        # 1. 初始化 Pico/6颗ToF/WT901/Dynamixel，并执行电机检测与恢复
        robot = R.initialize_robot()

        print("\n[ACTION] 动作1：直行前进3秒")
        R.run_for_time(
            robot,
            steering=0,
            speed=10,
            run_time=3.0
        )

        # run_for_time() 结束时已经把驱动速度归零。
        time.sleep(0.5)

        print("\n[ACTION] 动作2：直线倒退30厘米")
        R.runForDistance(
            robot,
            steering=0,
            speed=-10,
            distance_cm=30.0
        )

        R.calibrate_gyro_offset_from_right_wall(
            robot,
            wall_heading_deg=0
        )


        R.GyroRun_Distance(
        robot,
        speed=10,
        distance_cm=100,
        keep_distance_cm=10,
        target_angle_deg=0
    )
        
        
        print("\n[ACTION] 全部动作完成，准备结束程序。")

    except KeyboardInterrupt:
        print("\n[CTRL+C] 收到退出指令。")

    finally:
        # 无论正常结束、异常还是Ctrl+C，都执行全局安全停车/关闭
        R.safe_shutdown_robot(
            robot,
            reason="动作程序结束，执行全局安全关闭。"
        )


if __name__ == "__main__":
    main()
