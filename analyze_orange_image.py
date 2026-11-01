#!/usr/bin/env python3
"""Pi Camera 实时颜色分析：鼠标拖框取样任意颜色；不连接电机。

实时预览：python3 analyze_orange_image.py
显示黑白掩膜：python3 analyze_orange_image.py --show-mask
无窗口：python3 analyze_orange_image.py --no-show
离线图片：python3 analyze_orange_image.py photo.jpg --output /tmp/orange.jpg
依赖：OpenCV、NumPy，与 line_color_detect.py 相同。
HSV 默认阈值仅为起点，需要按实际灯光调整。
实时窗口：左键拖框取色；c=取色模式，r=检测区域模式，f=全图检测，o=恢复橙色。
每次取色后统计首次匹配耗时和检测次数；测量到算法完成，不包含远程画面传输。
画面显示取图、处理和上一完整循环的秒数与 Hz；取图 Hz 是调用速率，不是传感器帧率。
"""

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np


def duration_metrics(seconds):
    """秒数及其倒数（Hz）；不可测的零时长不报告无限频率。"""
    return {"seconds": round(seconds, 9),
            "hz": round(1.0 / seconds, 3) if seconds > 0 else None}


def draw_performance(image, performance):
    """叠加性能数据，秒和 Hz 并列；完整循环只能在下一帧显示。"""
    for y, label, key in ((88, "Get(call)", "capture"),
                          (104, "Process", "processing"),
                          (120, "Loop(prev)", "previous_loop")):
        measured = performance[key]
        if measured is None:
            text = f"{label}: waiting..."
        else:
            hz_text = f"{measured['hz']:.1f} Hz" if measured["hz"] is not None else "-- Hz"
            text = f"{label}: {measured['seconds']:.6f} s | {hz_text}"
        # 相同字重的黑色阴影兼容不同 OpenCV 版本的文字间距。
        cv2.putText(image, text, (5, y + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(image, text, (4, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (255, 255, 255), 1, cv2.LINE_AA)


def sample_color_ranges(bgr_patch):
    """由选区学习 HSV 范围；忽略两端少量异常值，支持红色色相跨 179/0。

    尽量只框住目标颜色。固定容差允许小幅光照变化，但不保证复杂背景下唯一。
    """
    hsv = cv2.cvtColor(bgr_patch, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    lo, hi = np.percentile(hsv, (5, 95), axis=0)
    s0, v0 = max(0, int(lo[1]) - 30), max(0, int(lo[2]) - 30)
    s1, v1 = min(255, int(hi[1]) + 30), min(255, int(hi[2]) + 30)
    if hi[2] < 50:  # 很暗时色相、饱和度不可靠，用亮度识别黑色。
        return [((0, 0, v0), (179, 255, v1))]
    if hi[1] < 40:  # 白/灰等低饱和度颜色，不限制色相。
        return [((0, 0, v0), (179, s1, v1))]
    # 在色环中寻找包含 90% 样本的最短区间，再向两边扩 8。
    hues = np.sort(hsv[:, 0].astype(np.int16))
    n = len(hues)
    span = max(1, math.ceil(n * 0.9))
    doubled = np.concatenate((hues, hues + 180))
    widths = doubled[span - 1:span - 1 + n] - doubled[:n]
    start = int(np.argmin(widths))
    low, high = int(doubled[start]) - 8, int(doubled[start + span - 1]) + 8
    if high - low >= 179:
        return [((0, s0, v0), (179, s1, v1))]
    low, high = low % 180, high % 180
    if low <= high:
        return [((low, s0, v0), (high, s1, v1))]
    return [((0, s0, v0), (high, s1, v1)),
            ((low, s0, v0), (179, s1, v1))]


class DetectionTiming:
    """每次取色单独计时；只数松开鼠标后的实际检测调用，不数打印行数。"""

    def __init__(self, sample_id, pressed_at, released_at, ready_at):
        self.sample_id = sample_id
        self.pressed_at = pressed_at
        self.released_at = released_at
        self.ready_at = ready_at
        self.runs = 0
        self.first_match = None

    def record(self, has_target, finished_at):
        self.runs += 1
        if has_target and self.first_match is None:
            self.first_match = {
                "runs_to_first_match": self.runs,
                "press_to_first_match_ms": round(1000 * (finished_at - self.pressed_at), 3),
                "release_to_first_match_ms": round(1000 * (finished_at - self.released_at), 3),
            }
            return True
        return False

    def report(self):
        result = {"sample_id": self.sample_id,
                  "state": "matched" if self.first_match is not None else "waiting",
                  "drag_ms": round(1000 * (self.released_at - self.pressed_at), 3),
                  "sampling_ms": round(1000 * (self.ready_at - self.released_at), 3),
                  "runs_since_sample": self.runs,
                  "runs_to_first_match": None,
                  "press_to_first_match_ms": None,
                  "release_to_first_match_ms": None}
        if self.first_match is not None:
            result.update(self.first_match)
        return result


class MouseSelection:
    """鼠标坐标映射回原图；取样使用按下时的原始画面，避免拖动期间变化。"""

    def __init__(self, lower, upper):
        self.default_ranges = [(tuple(lower), tuple(upper))]
        self.ranges = self.default_ranges
        self.mode = "color"
        self.color_name = "Orange"
        self.roi = None
        self.frame = None
        self.display_size = None
        self.start = self.end = self.frozen = None
        self.pressed_at = None
        self.sample_id = 0
        self.timing = None
        self.message = "Drag inside a solid color to sample"

    def bounds(self):
        x0, x1 = sorted((self.start[0], self.end[0]))
        y0, y1 = sorted((self.start[1], self.end[1]))
        return x0, y0, x1 + 1, y1 + 1

    def on_mouse(self, event, x, y, flags, param):
        if self.frame is None or self.display_size is None:
            return
        h, w = self.frame.shape[:2]
        dw, dh = self.display_size
        point = (min(w - 1, max(0, int(x * w / dw))),
                 min(h - 1, max(0, int(y * h / dh))))
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pressed_at = time.perf_counter()
            self.start = self.end = point
            self.frozen = self.frame.copy()
        elif event == cv2.EVENT_MOUSEMOVE and self.start is not None:
            self.end = point
        elif event == cv2.EVENT_LBUTTONUP and self.start is not None:
            released_at = time.perf_counter()
            self.end = point
            x0, y0, x1, y1 = self.bounds()
            if self.mode == "roi":
                self.roi = (x0, y0, x1, y1)
                self.timing = None  # 检测范围已改变，旧的首次匹配测量失效。
                self.message = f"ROI: ({x0}, {y0}) - ({x1 - 1}, {y1 - 1})"
            else:
                # 单击时采样周围 5x5 像素，拖动时使用整个矩形。
                if x1 - x0 < 3 and y1 - y0 < 3:
                    x0, x1 = max(0, x0 - 2), min(w, x1 + 2)
                    y0, y1 = max(0, y0 - 2), min(h, y1 + 2)
                self.ranges = sample_color_ranges(self.frozen[y0:y1, x0:x1])
                self.sample_id += 1
                self.timing = DetectionTiming(self.sample_id, self.pressed_at,
                                               released_at, time.perf_counter())
                self.color_name = "Sampled"
                self.message = "Color sampled; drag again to replace"
                print(f"[Sample] id={self.sample_id} HSV ranges: {self.ranges}", flush=True)
            self.start = self.end = self.frozen = None
            self.pressed_at = None

    def key(self, key):
        if key in (ord("c"), ord("r"), ord("f"), ord("o"), 27):
            self.start = self.end = self.frozen = None
            self.pressed_at = None
        if key in (ord("c"), ord("r")):
            self.mode = "color" if key == ord("c") else "roi"
            self.message = ("Drag inside a solid color to sample" if self.mode == "color"
                            else "Drag to restrict detection to a rectangle")
        elif key == ord("f"):
            self.roi = None
            self.timing = None
            self.message = "Detection: full frame"
        elif key == ord("o"):
            self.ranges = self.default_ranges
            self.timing = None
            self.color_name = "Orange"
            self.message = "Default orange restored"


def analyze_orange(bgr, lower=(5, 100, 80), upper=(25, 255, 255), min_area=50,
                   hsv_ranges=None, roi=None):
    """输入 OpenCV BGR 图片，返回统计、去噪掩膜和标注图。

    面积为实际前景像素数；只保留达到 min_area 的连通区域。
    坐标以原图左上角为原点，bbox 为 [x, y, width, height]。
    center 为色块像素质心；最大的色块作为目标，target_center 为其坐标。
    """
    height, width = bgr.shape[:2]
    x0, y0, x1, y1 = roi if roi is not None else (0, 0, width, height)
    hsv = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    local_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    ranges = hsv_ranges if hsv_ranges is not None else [(lower, upper)]
    for lo, hi in ranges:
        local_mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    kernel = np.ones((3, 3), np.uint8)
    local_mask = cv2.morphologyEx(local_mask, cv2.MORPH_OPEN, kernel)
    local_mask = cv2.morphologyEx(local_mask, cv2.MORPH_CLOSE, kernel)
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[y0:y1, x0:x1] = local_mask
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
    regions = []
    keep = np.zeros(count, dtype=np.uint8)
    annotated = bgr.copy()
    for label in range(1, count):
        x, y, blob_width, blob_height, area = map(int, stats[label])
        if area < min_area:
            continue
        keep[label] = 255
        center = [round(float(v), 2) for v in centers[label]]
        regions.append({"area_pixels": area, "bbox": [x, y, blob_width, blob_height],
                        "center": center})
        cv2.rectangle(annotated, (x, y), (x + blob_width - 1, y + blob_height - 1),
                      (0, 255, 0), 1)
        cv2.putText(annotated, str(area), (x, max(12, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    regions.sort(key=lambda region: region["area_pixels"], reverse=True)
    target_center = regions[0]["center"] if regions else None
    if target_center is not None:
        cx, cy = (int(round(value)) for value in target_center)
        # 在原彩色图上叠加目标中心，不用掩膜替换画面。
        cv2.drawMarker(annotated, (cx, cy), (0, 0, 255),
                       cv2.MARKER_CROSS, 15, 2)
        cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), 1)
    center_text = (f"Target center: ({target_center[0]:.1f}, {target_center[1]:.1f})"
                   if target_center is not None else "Target center: none")
    cv2.putText(annotated, center_text, (4, 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    pixels = sum(region["area_pixels"] for region in regions)
    return {"width": width, "height": height, "matched_pixels": pixels,
            "matched_percent": round(100 * pixels / ((x1 - x0) * (y1 - y0)), 3),
            "roi": [x0, y0, x1, y1], "hsv_ranges": ranges,
            # 保留原离线接口字段；任意颜色请使用 matched_*。
            "orange_pixels": pixels,
            "orange_percent": round(100 * pixels / (width * height), 3),
            "region_count": len(regions),
            "target_center": target_center,
            "largest_region": regions[0] if regions else None,
            "regions": regions}, keep[labels], annotated


def preview_camera(args):
    # 复用项目已有相机参数；不导入驾驶/电机程序。
    from camera_view import init_camera

    cam = init_camera()
    frames, fps = 0, 0.0
    detection_runs_total = 0
    previous_loop = None
    fps_start = time.monotonic()
    last_report = fps_start - 1.0
    selection = MouseSelection(args.lower, args.upper)
    print("[Color] Drag to sample; c=color, r=ROI, f=full frame, o=orange, q=quit",
          flush=True)
    try:
        if not args.no_show:
            cv2.namedWindow("orange camera", cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback("orange camera", selection.on_mouse)
        while True:
            loop_start = time.perf_counter()
            rgb = cam.run()
            capture_finished = time.perf_counter()
            if rgb is None:
                time.sleep(0.01)
                continue
            # 处理计时包含 RGB 转换、颜色检测和基本目标标注，排除取图及窗口显示。
            analysis_start = capture_finished
            live_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            # 拖框时冻结取样画面；松开后立即恢复实时检测。
            bgr = selection.frozen if selection.frozen is not None else live_bgr
            result, mask, annotated = analyze_orange(
                bgr, args.lower, args.upper, args.min_area, selection.ranges, selection.roi)
            analysis_finished = time.perf_counter()
            detection_runs_total += 1
            result["analysis_ms"] = round(1000 * (analysis_finished - analysis_start), 3)
            result["performance"] = {
                "capture": duration_metrics(capture_finished - loop_start),
                "processing": duration_metrics(analysis_finished - analysis_start),
                "previous_loop": previous_loop,
                "frame_frozen": selection.frozen is not None,
            }
            result["detection_runs_total"] = detection_runs_total
            if selection.timing is not None and selection.frozen is None:
                first_match = selection.timing.record(result["target_center"] is not None,
                                                       analysis_finished)
                if first_match:
                    measured = selection.timing.report()
                    # 首次匹配立刻打印，避免受到每 0.5 秒输出统计的节流影响。
                    print(f"[Detection] sample={measured['sample_id']} "
                          f"press_to_match={measured['press_to_first_match_ms']:.3f} ms "
                          f"release_to_match={measured['release_to_first_match_ms']:.3f} ms "
                          f"runs={measured['runs_to_first_match']}", flush=True)
            result["sample_timing"] = selection.timing.report() if selection.timing else None
            result["color"] = selection.color_name
            frames += 1
            now = time.monotonic()
            if now - fps_start >= 1.0:
                fps = frames / (now - fps_start)
                frames, fps_start = 0, now
            if now - last_report >= 0.5:
                print(json.dumps(result, ensure_ascii=False), flush=True)
                last_report = now
            quit_requested = False
            if not args.no_show:
                cv2.putText(annotated,
                            f"{selection.color_name} {result['matched_percent']:.1f}%  Avg {fps:.1f} Hz",
                            (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                timing = result["sample_timing"]
                if timing is not None:
                    if timing["state"] == "matched":
                        text = (f"Release->match: {timing['release_to_first_match_ms'] / 1000:.6f} s"
                                f" | {timing['runs_to_first_match']} runs")
                        press_text = f"Press->match: {timing['press_to_first_match_ms'] / 1000:.6f} s"
                    else:
                        text = f"Waiting for match | {timing['runs_since_sample']} runs"
                        press_text = ""
                    cv2.putText(annotated, text, (4, 54),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                    cv2.putText(annotated, press_text, (4, 70),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                draw_performance(annotated, result["performance"])
                if selection.roi is not None:
                    x0, y0, x1, y1 = selection.roi
                    cv2.rectangle(annotated, (x0, y0), (x1 - 1, y1 - 1), (255, 255, 0), 1)
                if selection.start is not None:
                    x0, y0, x1, y1 = selection.bounds()
                    cv2.rectangle(annotated, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 255), 1)
                h, w = bgr.shape[:2]
                cv2.putText(annotated, f"Mode: {selection.mode} | c=color r=ROI f=full o=orange",
                            (4, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1)
                cv2.putText(annotated, selection.message, (4, h - 7),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1)
                selection.frame = bgr
                selection.display_size = (max(1, round(w * args.scale)),
                                          max(1, round(h * args.scale)))
                views = [("orange camera", annotated)]
                if args.show_mask:
                    views.append(("orange mask", mask))
                for title, frame in views:
                    if args.scale != 1.0:
                        frame = cv2.resize(frame, selection.display_size,
                                           interpolation=cv2.INTER_NEAREST)
                    cv2.imshow(title, frame)
                key = cv2.waitKey(1) & 0xFF
                quit_requested = key == ord("q")
                selection.key(key)
            # 包含取图、处理、日志输出、窗口提交和鼠标/键盘事件处理。
            # 本轮窗口已经提交，因此将结果留到下一帧叠加，避免漏掉显示开销。
            previous_loop = duration_metrics(time.perf_counter() - loop_start)
            if quit_requested:
                break
    except KeyboardInterrupt:
        print("\n[Orange] stopped")
    finally:
        try:
            cam.shutdown()
        finally:
            if not args.no_show:
                cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, nargs="?", help="可选图片路径；不填则启动 Pi Camera")
    parser.add_argument("--lower", type=int, nargs=3, default=(5, 100, 80),
                        metavar=("H", "S", "V"), help="HSV 下限，默认 5 100 80")
    parser.add_argument("--upper", type=int, nargs=3, default=(25, 255, 255),
                        metavar=("H", "S", "V"), help="HSV 上限，默认 25 255 255")
    parser.add_argument("--min-area", type=int, default=50, help="最小色块像素数")
    parser.add_argument("--output", type=Path, help="保存标注图（不能覆盖原图）")
    parser.add_argument("--mask", type=Path, help="保存黑白掩膜图")
    parser.add_argument("--scale", type=float, default=2.0, help="实时窗口放大倍数，默认 2")
    parser.add_argument("--no-show", action="store_true", help="实时模式只打印统计，不开窗口")
    parser.add_argument("--show-mask", action="store_true", help="额外显示黑白掩膜，默认只显示彩色标注画面")
    args = parser.parse_args()
    if not math.isfinite(args.scale) or args.scale <= 0:
        parser.error("--scale 必须为大于 0 的有限数")
    if args.min_area < 1:
        parser.error("--min-area 必须大于 0")
    if any(not 0 <= lo <= hi <= limit for lo, hi, limit in
           zip(args.lower, args.upper, (179, 255, 255))):
        parser.error("HSV 须满足 0 <= 下限 <= 上限；H 最大 179，S/V 最大 255")
    if args.image is None:
        if args.output or args.mask:
            parser.error("--output / --mask 仅适用于输入图片的离线模式")
        preview_camera(args)
        return
    paths = [p.resolve() for p in (args.image, args.output, args.mask) if p]
    if len(paths) != len(set(paths)):
        parser.error("输入、标注图和掩膜图必须使用不同路径")
    bgr = cv2.imread(str(args.image))
    if bgr is None:
        parser.error(f"无法读取图片：{args.image}")
    result, mask, annotated = analyze_orange(bgr, args.lower, args.upper, args.min_area)
    for path, data in ((args.output, annotated), (args.mask, mask)):
        if path:
            try:
                saved = cv2.imwrite(str(path), data)
            except cv2.error as exc:
                parser.error(f"无法保存 {path}：{exc}")
            if not saved:
                parser.error(f"无法保存：{path}（请检查目录和图片扩展名）")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
