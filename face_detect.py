#!/usr/bin/env python3
"""
WRO Future Engineers – 相机人脸检测（默认运行 30 秒）

功能：
  - 用与 camera_view.py 相同的 PiCamera 取图
  - 用 OpenCV 自带的 Haar 级联分类器检测人脸（轻量，无需额外下载模型）
  - 窗口里给每张人脸画框，左上角显示 FPS、人脸数和剩余时间
  - 到时间自动结束，最后打印统计；Ctrl+C 或窗口里按 q 可提前退出
  - 不控制电机，可以放心运行

用法示例：
  python3 face_detect.py                    # 检测 30 秒
  python3 face_detect.py --duration 60      # 检测 60 秒
  python3 face_detect.py --auto-exposure    # 用相机自动曝光/白平衡（室内看脸更亮）
  python3 face_detect.py --no-window        # 不开窗口，只在终端打印
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

# =====================  人脸检测参数 =====================
CASCADE_FILE   = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
SCALE_FACTOR   = 1.1        # 图像金字塔缩放步长，越小越准但越慢
MIN_NEIGHBORS  = 5          # 越大误检越少，但可能漏检
MIN_FACE_SIZE  = (30, 30)   # 320x240 图像里最小的人脸尺寸（像素）

WINDOW_NAME = "face detect"


# =====================  相机初始化（照搬 camera_view.py） =====================
def init_camera(auto_exposure=False):
    from donkeycar.parts.camera import PiCamera

    cam = PiCamera(image_w=CAPTURE_W, image_h=CAPTURE_H, image_d=3)
    time.sleep(2)

    cam.camera.framerate = CAMERA_FRAMERATE
    if not auto_exposure:
        cam.camera.exposure_mode         = PICAMERA_EXPOSURE_MODE
        cam.camera.awb_mode              = PICAMERA_AWB_MODE
        cam.camera.iso                   = PICAMERA_ISO
        cam.camera.shutter_speed         = PICAMERA_SHUTTER_SPEED
        cam.camera.exposure_compensation = PICAMERA_EXPOSURE_COMPENSATION
        cam.camera.awb_gains             = PICAMERA_AWB_GAINS

    print(f"[Camera] initialized ({'auto' if auto_exposure else 'fixed'} exposure)")
    return cam


def main():
    ap = argparse.ArgumentParser(description="相机人脸检测，默认 30 秒")
    ap.add_argument("--duration", type=float, default=30.0,
                    help="检测时长（秒，默认 30）")
    ap.add_argument("--scale", type=float, default=2.0,
                    help="窗口显示放大倍数（默认 2）")
    ap.add_argument("--auto-exposure", action="store_true",
                    help="使用相机自动曝光/白平衡，而不是比赛用的固定参数")
    ap.add_argument("--no-window", action="store_true",
                    help="不显示窗口，只在终端打印检测结果")
    args = ap.parse_args()

    detector = cv2.CascadeClassifier(CASCADE_FILE)
    if detector.empty():
        raise SystemExit(f"[Face] failed to load cascade: {CASCADE_FILE}")

    cam = init_camera(args.auto_exposure)
    print(f"[Face] detecting for {args.duration:.0f} s, press Ctrl+C to quit early")

    frames = 0
    fps = 0.0
    fps_t0 = time.monotonic()
    total_frames = 0
    frames_with_face = 0
    max_faces = 0
    last_count = -1

    t_start = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            remaining = args.duration - (now - t_start)
            if remaining <= 0:
                break

            frame = cam.run()
            if frame is None:
                continue

            # ---------- 人脸检测（在灰度图上做） ----------
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            gray = cv2.equalizeHist(gray)
            faces = detector.detectMultiScale(gray,
                                              scaleFactor=SCALE_FACTOR,
                                              minNeighbors=MIN_NEIGHBORS,
                                              minSize=MIN_FACE_SIZE)
            count = len(faces)

            total_frames += 1
            if count > 0:
                frames_with_face += 1
            max_faces = max(max_faces, count)

            # 人脸数变化时才打印，避免刷屏
            if count != last_count:
                boxes = ", ".join(f"({x},{y},{w}x{h})" for (x, y, w, h) in faces)
                print(f"[Face] t={now - t_start:5.1f}s  faces={count}  {boxes}")
                last_count = count

            # ---------- 帧率 ----------
            frames += 1
            if now - fps_t0 >= 1.0:
                fps = frames / (now - fps_t0)
                frames = 0
                fps_t0 = now

            if args.no_window:
                continue

            # DonkeyCar PiCamera 输出 RGB，OpenCV 显示要 BGR
            disp = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            if args.scale != 1.0:
                disp = cv2.resize(disp, None, fx=args.scale, fy=args.scale,
                                  interpolation=cv2.INTER_NEAREST)
            s = args.scale
            for (x, y, w, h) in faces:
                cv2.rectangle(disp, (int(x * s), int(y * s)),
                              (int((x + w) * s), int((y + h) * s)), (0, 255, 0), 2)
            cv2.putText(disp, f"FPS {fps:.1f}  faces {count}  {remaining:4.1f}s",
                        (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow(WINDOW_NAME, disp)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n[Face] stopped by user")
    finally:
        try:
            cam.shutdown()
        except Exception:
            pass
        cv2.destroyAllWindows()

    elapsed = time.monotonic() - t_start
    pct = 100.0 * frames_with_face / total_frames if total_frames else 0.0
    print(f"[Face] done: {elapsed:.1f} s, {total_frames} frames "
          f"({total_frames / elapsed:.1f} fps avg), "
          f"face seen in {frames_with_face} frames ({pct:.0f}%), max {max_faces} at once")


if __name__ == "__main__":
    main()
