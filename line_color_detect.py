#!/usr/bin/env python3
"""
WRO Future Engineers – 地面色线检测（独立测试 / 标定脚本）

功能：
  - 用与 AFI_CUP_V05.py 完全相同的 PiCamera 参数取图
  - 在画面下方的 ROI（地面区域）里识别指定颜色：yellow / blue / green / black
  - 计算该颜色"最大连通色块"的面积（像素）
  - 面积 >= 阈值 且连续 N 帧满足时打印出来

用法示例：
  python3 line_color_detect.py --color blue
  python3 line_color_detect.py --color all --min-area 1200
  python3 line_color_detect.py --color yellow --show          # 接显示器 / VNC 时看画面
  python3 line_color_detect.py --calib                        # 标定：打印画面中心的 HSV 值

也可以在主程序里直接 import detect_color_area() 使用（见文件末尾说明）。
"""

import time
import argparse

import numpy as np
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

# =====================  检测参数  =====================
# ROI：只看画面里的地面部分，用比例表示 (top, bottom, left, right)
# 默认取下半部分，避免把黑色围墙当成黑线
ROI = (0.55, 1.0, 0.0, 1.0)

# 默认面积阈值（像素，基于 320x240 原图里的 ROI）
DEFAULT_MIN_AREA = 1500

# 连续多少帧超过阈值才算有效（防止单帧噪声误触发）
DEFAULT_CONFIRM_FRAMES = 2

# HSV 阈值（OpenCV：H 0-179, S 0-255, V 0-255）
# 关掉了自动白平衡，颜色会受 AWB_GAINS 和现场灯光影响，
# 请用 --calib 模式在实际场地上测一下再微调。
COLOR_RANGES = {
    "yellow": [((20,  80,  80), (35, 255, 255))],
    "blue":   [((100, 80,  50), (130, 255, 255))],
    "green":  [((40,  60,  40), (85, 255, 255))],
    "black":  [((0,    0,   0), (179, 255, 50))],   # 黑色主要靠亮度 V 判断
}

# 画框用的颜色（BGR，仅 --show 时使用）
DRAW_COLORS = {
    "yellow": (0, 255, 255),
    "blue":   (255, 0, 0),
    "green":  (0, 255, 0),
    "black":  (255, 255, 255),
}

KERNEL = np.ones((5, 5), np.uint8)


# =====================  相机初始化（照搬原程序） =====================
def init_camera():
    from donkeycar.parts.camera import PiCamera   # 放在函数里，方便没有相机时单独测试检测函数

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


# =====================  检测函数  =====================
def roi_bounds(h, w, roi=ROI):
    top, bottom, left, right = roi
    return int(h * top), int(h * bottom), int(w * left), int(w * right)


def color_mask(hsv, color):
    mask = None
    for lo, hi in COLOR_RANGES[color]:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    # 开运算去小噪点，闭运算补小洞
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
    return mask


def largest_blob(mask):
    # [-2] 兼容 OpenCV 3（返回3个值）和 OpenCV 4（返回2个值）
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not contours:
        return 0.0, None
    c = max(contours, key=cv2.contourArea)
    return float(cv2.contourArea(c)), c


def detect_color_area(rgb_frame, color, roi=ROI):
    """
    输入：相机原图（DonkeyCar PiCamera 输出的是 RGB 顺序）
    返回：(最大色块面积, mask, 轮廓, ROI左上角偏移(x0, y0))
    """
    h, w = rgb_frame.shape[:2]
    y0, y1, x0, x1 = roi_bounds(h, w, roi)
    hsv = cv2.cvtColor(rgb_frame[y0:y1, x0:x1], cv2.COLOR_RGB2HSV)
    mask = color_mask(hsv, color)
    area, contour = largest_blob(mask)
    return area, mask, contour, (x0, y0)


# =====================  主循环  =====================
def main():
    ap = argparse.ArgumentParser(description="WRO FE 地面色线检测")
    ap.add_argument("--color", default="blue",
                    choices=list(COLOR_RANGES.keys()) + ["all"])
    ap.add_argument("--min-area", type=float, default=DEFAULT_MIN_AREA,
                    help="面积阈值（像素）")
    ap.add_argument("--confirm", type=int, default=DEFAULT_CONFIRM_FRAMES,
                    help="连续多少帧超过阈值才打印")
    ap.add_argument("--verbose", action="store_true",
                    help="每帧都打印面积（调阈值用）")
    ap.add_argument("--show", action="store_true",
                    help="显示画面窗口（需要显示器或 VNC）")
    ap.add_argument("--calib", action="store_true",
                    help="标定模式：打印 ROI 中心 20x20 区域的 HSV 中位数")
    args = ap.parse_args()

    colors = list(COLOR_RANGES.keys()) if args.color == "all" else [args.color]
    counters = {c: 0 for c in colors}

    cam = init_camera()
    print(f"[Detect] colors={colors}, min_area={args.min_area}, "
          f"confirm={args.confirm}, ROI={ROI}")

    frames = 0
    fps_t0 = time.monotonic()
    calib_t = 0.0

    try:
        while True:
            frame = cam.run()
            if frame is None:
                continue
            frames += 1

            disp = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if args.show else None

            # ---------- 标定模式 ----------
            if args.calib:
                now = time.monotonic()
                if now - calib_t >= 0.5:
                    calib_t = now
                    h, w = frame.shape[:2]
                    y0, y1, x0, x1 = roi_bounds(h, w)
                    cy, cx = (y0 + y1) // 2, (x0 + x1) // 2
                    patch = frame[cy - 10:cy + 10, cx - 10:cx + 10]
                    hsv = cv2.cvtColor(patch, cv2.COLOR_RGB2HSV).reshape(-1, 3)
                    hh, ss, vv = np.median(hsv, axis=0).astype(int)
                    print(f"[Calib] 中心 HSV = H:{hh:3d}  S:{ss:3d}  V:{vv:3d}")
                if disp is not None:
                    h, w = frame.shape[:2]
                    y0, y1, x0, x1 = roi_bounds(h, w)
                    cy, cx = (y0 + y1) // 2, (x0 + x1) // 2
                    cv2.rectangle(disp, (cx - 10, cy - 10), (cx + 10, cy + 10), (0, 0, 255), 1)

            # ---------- 颜色检测 ----------
            for color in colors:
                area, mask, contour, (ox, oy) = detect_color_area(frame, color)

                if area >= args.min_area:
                    counters[color] += 1
                else:
                    counters[color] = 0

                if counters[color] >= args.confirm:
                    tag = "  <<< 触发" if counters[color] == args.confirm else ""
                    print(f"[LINE] {color:6s} 面积 = {area:7.0f} px{tag}")
                elif args.verbose:
                    print(f"[dbg]  {color:6s} 面积 = {area:7.0f} px")

                if disp is not None and contour is not None:
                    cv2.drawContours(disp, [contour], -1, DRAW_COLORS[color], 2,
                                     offset=(ox, oy))
                    cv2.putText(disp, f"{color}:{area:.0f}",
                                (ox + 4, oy + 14 + 14 * colors.index(color)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, DRAW_COLORS[color], 1)

            # ---------- 显示 ----------
            if disp is not None:
                h, w = frame.shape[:2]
                y0, y1, x0, x1 = roi_bounds(h, w)
                cv2.rectangle(disp, (x0, y0), (x1 - 1, y1 - 1), (0, 0, 255), 1)
                cv2.imshow("line detect", disp)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break

            # ---------- 帧率 ----------
            if args.verbose and time.monotonic() - fps_t0 >= 2.0:
                print(f"[FPS] {frames / (time.monotonic() - fps_t0):.1f}")
                frames = 0
                fps_t0 = time.monotonic()

    except KeyboardInterrupt:
        print("\n[Detect] stopped by user")
    finally:
        try:
            cam.shutdown()
        except Exception:
            pass
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


# =====================  在主程序里使用 =====================
# 把本文件和 AFI_CUP_V05.py 放在同一目录，然后在 CarController 的某个行驶循环里：
#
#   from line_color_detect import detect_color_area
#
#   raw = self.camera.run()
#   area, _, _, _ = detect_color_area(raw, "blue")
#   if area >= 1500:
#       print(f"[run] blue line detected, area={area:.0f}")
#       break    # 停车
