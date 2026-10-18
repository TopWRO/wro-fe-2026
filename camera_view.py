#!/usr/bin/env python3
"""
WRO Future Engineers – 相机实时预览

功能：
  - 用与 AFI_CUP_V05.py / line_color_detect.py 相同的 PiCamera 参数取图
    （看到的画面就是模型/检测程序实际拿到的画面）
  - 在窗口里显示画面，左上角显示 FPS
  - Ctrl+C 退出（窗口里按 q 也可以）

用法示例：
  python3 camera_view.py              # 320x240 原图，窗口放大 2 倍显示
  python3 camera_view.py --scale 1    # 按原始大小显示
"""

import time
import argparse

import cv2

# =====================  相机参数（与 AFI_CUP_V05.py 保持一致） =====================
CAPTURE_W, CAPTURE_H = (320, 240)

CAMERA_FRAMERATE               = 30
PICAMERA_AWB_MODE              = 'off'
PICAMERA_EXPOSURE_MODE         = 'off'
PICAMERA_ISO                   = 100
PICAMERA_SHUTTER_SPEED         = 15000
PICAMERA_AWB_GAINS             = (1.2, 1.5)
PICAMERA_EXPOSURE_COMPENSATION = 0

WINDOW_NAME = "camera view"


# =====================  相机初始化（照搬原程序） =====================
def init_camera():
    from donkeycar.parts.camera import PiCamera

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


def main():
    ap = argparse.ArgumentParser(description="相机实时预览，Ctrl+C 退出")
    ap.add_argument("--scale", type=float, default=2.0,
                    help="窗口显示放大倍数（默认 2）")
    args = ap.parse_args()

    cam = init_camera()
    print("[View] showing camera, press Ctrl+C to quit")

    frames = 0
    fps = 0.0
    fps_t0 = time.monotonic()

    try:
        while True:
            frame = cam.run()
            if frame is None:
                continue

            # ---------- 帧率 ----------
            frames += 1
            now = time.monotonic()
            if now - fps_t0 >= 1.0:
                fps = frames / (now - fps_t0)
                frames = 0
                fps_t0 = now

            # DonkeyCar PiCamera 输出 RGB，OpenCV 显示要 BGR
            disp = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            if args.scale != 1.0:
                disp = cv2.resize(disp, None, fx=args.scale, fy=args.scale,
                                  interpolation=cv2.INTER_NEAREST)
            cv2.putText(disp, f"FPS {fps:.1f}", (6, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow(WINDOW_NAME, disp)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n[View] stopped by user")
    finally:
        try:
            cam.shutdown()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
