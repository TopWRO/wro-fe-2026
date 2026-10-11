#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import robot_lib as R


def main():
    robot = None

    try:
        # 1) 初始化全部硬件
        robot = R.initialize_robot()

        # 2) 向前直行 30 cm
        print("\n[ACTION 1] 向前直行 30 cm")
        R.runForDistance(
            robot,
            steering=0,
            speed=20,
            distance_cm=30
        )

        time.sleep(0.5)

        # 3) 向后直行 3 秒
        print("\n[ACTION 2] 向后直行 3 秒")
        R.run_for_time(
            robot,
            steering=0,
            speed=-20,
            run_time=1
        )

        time.sleep(0.5)

        # 4) 利用右前/右后 ToF 对 gyro offset 做校准
        #    这面右墙平行时的标准航向设为 0°
        print("\n[ACTION 3] 利用右墙校准 Gyro Offset，标准航向 = 0°")
        R.calibrate_gyro_offset_from_right_wall(
            robot,
            wall_heading_deg=0
        )

        time.sleep(0.5)

        # 5) 使用修正后的 gyro，沿 0° 前进 8 秒
        print("\n[ACTION 4] 使用 GyroRun 沿 0° 前进 8 秒")
        R.GyroRun_Time(
            robot,
            speed=30,
            run_time=5,
            keep_distance_cm=10,
            target_angle_deg=0
        )

        print("\n[ACTION] 全部动作执行完成。")

    except KeyboardInterrupt:
        print("\n[CTRL+C] 收到退出指令。")

    except Exception as exc:
        print(f"\n[ERROR] 动作程序异常：{exc}")
        raise

    finally:
        R.safe_shutdown_robot(
            robot,
            reason="动作程序结束，执行安全关闭。"
        )


if __name__ == "__main__":
    main()
