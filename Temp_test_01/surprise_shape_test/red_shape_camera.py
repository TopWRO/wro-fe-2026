#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""640x480 shape measurements using Recording_N02.py's DonkeyCar PiCamera.

Run inside the SAME Raspberry Pi Python environment as Recording_N02.py:
    python3 red_shape_camera.py
    python3 red_shape_camera.py --image model.jpg --no-window

Camera frames are RGB. Only the DISPLAY/SAVED PNG is converted to BGR.
This diagnostic measures apparent pixel dimensions; it does not infer cm.
"""

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import sys
import time

import cv2
import numpy as np


CAPTURE_W, CAPTURE_H = 640, 480
CAMERA_FRAMERATE = 30
LOOP_HZ = 20
SHUTTER_US = 1500
AWB_GAINS = (1.2, 1.5)
VIEW_TITLE = "Red shape - Recording_N02 camera"
MASK_TITLE = "Red mask / HSV controls"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "captures"


@dataclass
class Settings:
    # Same red HSV defaults as Recording_N02.py, including the hue wraparound.
    hue_width: int = 10
    s_min: int = 100
    v_min: int = 70
    min_area: int = 300
    min_size: int = 12
    kernel: int = 3


def detect_red(rgb, settings):
    """Return cleaned mask and up to three largest independent red regions.

    Do not swap RGB channels, resize the frame, or merge distant components.
    minAreaRect compensates for modest image-plane roll, NOT 3-D perspective.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    low = cv2.inRange(hsv, (0, settings.s_min, settings.v_min),
                      (settings.hue_width, 255, 255))
    high = cv2.inRange(hsv, (180 - settings.hue_width, settings.s_min,
                           settings.v_min), (179, 255, 255))
    mask = cv2.bitwise_or(low, high)
    if settings.kernel:
        kernel = np.ones((settings.kernel, settings.kernel), np.uint8)
        # Bridge tiny highlight holes, then remove isolated noise. Small kernel
        # preserves sharp corners better than heavy blur/large morphology.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
    frame_h, frame_w = mask.shape
    detections = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True):
        area = float(cv2.contourArea(contour))
        x, y, w, h = cv2.boundingRect(contour)
        if area < settings.min_area or min(w, h) < settings.min_size:
            continue
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        edges = np.roll(box, -1, axis=0) - box
        lengths = np.linalg.norm(edges, axis=1)
        if min(lengths) < 1:
            continue
        # Preserve screen-horizontal width and screen-vertical height. Sorting
        # edges into long/short would lose W/H>1 for the short, wide model.
        horizontal = int(np.argmax(np.abs(edges[:, 0]) / lengths))
        width = float(lengths[horizontal])
        height = float(lengths[(horizontal + 1) % 4])
        polygon = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        vertices = len(polygon)
        outline = "QUAD" if vertices == 4 and cv2.isContourConvex(polygon) else (
            "POLYGON-{}".format(vertices) if 3 <= vertices <= 6 else "IRREGULAR/ROUND")
        clipped = x <= 0 or y <= 0 or x + w >= frame_w or y + h >= frame_h
        metrics = {
            "id": len(detections) + 1,
            "bbox_xywh": [int(x), int(y), int(w), int(h)],
            "axis_width_px": int(w), "axis_height_px": int(h),
            "axis_w_over_h": float(w / h),
            "oriented_width_px": width, "oriented_height_px": height,
            "oriented_w_over_h": width / height,
            "oriented_h_over_w": height / width,
            "contour_area_px": area,
            "rectangle_fill": float(area / (width * height)),
            "polygon_vertices": vertices, "outline": outline,
            "touches_image_border": bool(clipped),
        }
        detections.append({"metrics": metrics, "contour": contour,
                           "box": box, "polygon": polygon})
        if len(detections) == 3:
            break
    return mask, detections


def render_view(rgb, detections, fps=0.0):
    """Annotate a copy: original RGB pixels remain untouched."""
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    for detection in reversed(detections):
        m = detection["metrics"]
        x, y, w, h = m["bbox_xywh"]
        cv2.drawContours(bgr, [detection["contour"]], -1, (255, 0, 255), 2)
        cv2.polylines(bgr, [np.rint(detection["box"]).astype(np.int32)],
                      True, (0, 255, 255), 1)
        cv2.rectangle(bgr, (x, y), (x + w - 1, y + h - 1), (0, 220, 0), 1)
        for point in detection["polygon"][:, 0]:
            cv2.circle(bgr, tuple(int(v) for v in point), 3, (255, 255, 0), -1)
        label = "#{} W/H={:.3f}".format(m["id"], m["oriented_w_over_h"])
        origin = (x, max(18, y - 7))
        cv2.putText(bgr, label, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(bgr, label, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    panel = np.full((max(480, bgr.shape[0]), 400, 3), 28, np.uint8)
    lines = ["RED SHAPE / largest region #1",
             "Frame: {}x{} | loop {:.1f} Hz".format(rgb.shape[1], rgb.shape[0], fps)]
    if detections:
        m = detections[0]["metrics"]
        lines += ["", "Axis-aligned bounding box:",
                  "W={} px  H={} px".format(m["axis_width_px"], m["axis_height_px"]),
                  "W/H = {:.4f}".format(m["axis_w_over_h"]), "",
                  "Oriented bounding box (yellow):",
                  "W={:.1f} px  H={:.1f} px".format(m["oriented_width_px"], m["oriented_height_px"]),
                  "W/H = {:.4f}".format(m["oriented_w_over_h"]),
                  "H/W = {:.4f}".format(m["oriented_h_over_w"]),
                  "Contour: {} | {} corners".format(m["outline"], m["polygon_vertices"]),
                  "Rectangle fill: {:.1%}".format(m["rectangle_fill"]),
                  "Contour area: {:.0f} px^2".format(m["contour_area_px"]),
                  "CLIPPED: ratio unreliable" if m["touches_image_border"] else "Border: complete silhouette"]
    else:
        lines += ["", "NO RED TARGET", "Check HSV sliders / light / size."]
    lines += ["", "Pixel ratio only; no cm estimate.",
              "Face-on references W/H:", "5cm: 1.000 | 10cm: 0.500",
              "15cm: 0.333 (5cm wide)",
              "S: save   Q/ESC: exit"]
    for i, line in enumerate(lines):
        cv2.putText(panel, line, (12, 23 + i * 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.48, (235, 235, 235), 1, cv2.LINE_AA)
    canvas = np.zeros((panel.shape[0], bgr.shape[1] + panel.shape[1], 3), np.uint8)
    canvas[:bgr.shape[0], :bgr.shape[1]] = bgr
    canvas[:, bgr.shape[1]:] = panel
    return canvas


def write_image(path, image):
    # imencode + tofile also works when the Windows output path contains Chinese.
    ok, encoded = cv2.imencode(path.suffix, image)
    if not ok:
        raise OSError("Could not encode image: {}".format(path))
    encoded.tofile(str(path))


def save_snapshot(output_dir, label, rgb, mask, detections, view, settings):
    output_dir = Path(output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    now = datetime.now().astimezone()
    safe_label = "".join(c if c.isalnum() or c in "-_" else "_" for c in label)
    prefix = output_dir / (now.strftime("%Y%m%d_%H%M%S_%f") + "_" + safe_label)
    write_image(Path(str(prefix) + "_raw.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    write_image(Path(str(prefix) + "_mask.png"), mask)
    write_image(Path(str(prefix) + "_view.png"), view)
    data = {"timestamp": now.isoformat(), "label": label,
            "frame_size_wh": [rgb.shape[1], rgb.shape[0]],
            "input_channel_order": "RGB", "settings": asdict(settings),
            "targets": [d["metrics"] for d in detections]}
    Path(str(prefix) + "_metrics.json").write_text(
        json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    print("\nSaved snapshot: {}_*".format(prefix))
    return prefix


class ShapeAnalyzer:
    """Non-threaded DonkeyCar part; OpenCV windows stay in the main thread."""

    def __init__(self, settings, show_window=True, output_dir=DEFAULT_OUTPUT, label="red"):
        self.settings = settings
        self.show_window = show_window
        self.output_dir, self.label = output_dir, label
        self.last_print = 0.0
        self.last_frame_time = None
        self.started = time.monotonic()
        self.printed_size = False
        if show_window:
            cv2.namedWindow(VIEW_TITLE, cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow(MASK_TITLE, cv2.WINDOW_AUTOSIZE)
            for name, value, maximum in [("Hue +/-", settings.hue_width, 40),
                                         ("S min", settings.s_min, 255),
                                         ("V min", settings.v_min, 255),
                                         ("Min area", settings.min_area, 20000)]:
                cv2.createTrackbar(name, MASK_TITLE, value, maximum, lambda _: None)

    def run(self, rgb):
        now = time.monotonic()
        if rgb is None:
            if now - self.started > 10:
                raise RuntimeError("No camera image after 10 seconds. Check the camera/DonkeyCar environment.")
            return
        if not self.printed_size:
            print("Actual frame: {}x{}, RGB".format(rgb.shape[1], rgb.shape[0]))
            if rgb.shape[:2] != (CAPTURE_H, CAPTURE_W):
                print("WARNING: camera did not return requested 640x480; measuring actual pixels.")
            self.printed_size = True
        if self.show_window:
            self.settings.hue_width = max(1, cv2.getTrackbarPos("Hue +/-", MASK_TITLE))
            self.settings.s_min = cv2.getTrackbarPos("S min", MASK_TITLE)
            self.settings.v_min = cv2.getTrackbarPos("V min", MASK_TITLE)
            self.settings.min_area = max(1, cv2.getTrackbarPos("Min area", MASK_TITLE))
        mask, detections = detect_red(rgb, self.settings)
        fps = 0.0 if self.last_frame_time is None else 1.0 / max(now - self.last_frame_time, 1e-6)
        self.last_frame_time = now
        if now - self.last_print >= 0.25:
            if detections:
                m = detections[0]["metrics"]
                print("RED #1: W={:.1f}px H={:.1f}px W/H={:.4f} H/W={:.4f} {}{}     ".format(
                    m["oriented_width_px"], m["oriented_height_px"],
                    m["oriented_w_over_h"], m["oriented_h_over_w"], m["outline"],
                    " [CLIPPED]" if m["touches_image_border"] else ""), end="\r", flush=True)
            else:
                print("NO RED TARGET                                                         ", end="\r", flush=True)
            self.last_print = now
        if self.show_window:
            view = render_view(rgb, detections, fps)
            cv2.imshow(VIEW_TITLE, view)
            cv2.imshow(MASK_TITLE, mask)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                raise KeyboardInterrupt
            if key in (ord("s"), ord("S")):
                save_snapshot(self.output_dir, self.label, rgb, mask, detections, view, self.settings)
            if cv2.getWindowProperty(VIEW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
                raise KeyboardInterrupt

    def shutdown(self):
        if self.show_window:
            cv2.destroyAllWindows()
        print()


def configure_camera(cam):
    """Use the source's settings; apply real controls on Picamera2 backends.

    PiCamera RGB output is unchanged in both DonkeyCar implementations.
    AnalogueGain=1.0 is a starting setting, not an exact ISO100 conversion.
    """
    native = cam.camera
    if callable(getattr(native, "set_controls", None)):
        native.set_controls({"AeEnable": False, "AwbEnable": False,
                             "ExposureTime": SHUTTER_US, "AnalogueGain": 1.0,
                             "ColourGains": AWB_GAINS,
                             "FrameDurationLimits": (33333, 33333)})
        return "Picamera2: shutter=15000us, gain=1.0, AWB=(1.2,1.5), 30fps"
    native.framerate = CAMERA_FRAMERATE
    native.exposure_mode = "off"
    native.awb_mode = "off"
    native.iso = 100
    native.shutter_speed = SHUTTER_US
    native.exposure_compensation = 0
    native.awb_gains = AWB_GAINS
    return "PiCamera legacy: exact Recording_N02.py manual camera settings"


def run_live(settings, args, show_window):
    # Lazy imports: --image does not need DonkeyCar or Raspberry Pi hardware.
    try:
        import donkeycar as dk
        from donkeycar.parts.camera import PiCamera
    except ImportError as exc:
        raise RuntimeError("Use the same Python/virtualenv that runs Recording_N02.py. "
                           "Camera import failed: {}".format(exc)) from exc
    car = dk.vehicle.Vehicle()
    cam = PiCamera(image_w=CAPTURE_W, image_h=CAPTURE_H, image_d=3)
    try:
        time.sleep(2)  # Same warm-up as Recording_N02.py.
        print(configure_camera(cam))
        time.sleep(0.5)  # Allow manual settings to settle before measurements.
        analyzer = ShapeAnalyzer(settings, show_window, args.output_dir, args.label)
    except BaseException:
        cam.shutdown()
        raise
    # Same threaded camera + main-thread processing as the reference program.
    # No resize to 160x120 and no center_crop stage.
    car.add(cam, outputs=["cam/image_array"], threaded=True)
    car.add(analyzer, inputs=["cam/image_array"], outputs=[])
    car.start(rate_hz=LOOP_HZ)  # DonkeyCar's start() shuts parts down in finally.


def run_image(settings, args, show_window):
    path = Path(args.image).expanduser()
    bgr = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError("Cannot read image: {}".format(path))
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    # Preserve native dimensions so a non-4:3 photograph isn't stretched.
    mask, detections = detect_red(rgb, settings)
    view = render_view(rgb, detections)
    print(json.dumps([d["metrics"] for d in detections], indent=2))
    save_snapshot(args.output_dir, args.label, rgb, mask, detections, view, settings)
    if show_window:
        try:
            cv2.imshow(VIEW_TITLE, view)
            cv2.imshow(MASK_TITLE, mask)
            print("Press any key in the image window to exit.")
            cv2.waitKey(0)
        finally:
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--image", help="Analyze an existing photograph instead of opening PiCamera")
    parser.add_argument("--no-window", action="store_true", help="Terminal only (SSH without a desktop)")
    parser.add_argument("--label", default="red", help="Snapshot label, e.g. low5, normal10, tall15")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--hue-width", type=int, default=10, help="Red hue tolerance around 0/179; default 10")
    parser.add_argument("--s-min", type=int, default=100, help="Minimum saturation; default same as source")
    parser.add_argument("--v-min", type=int, default=70, help="Minimum brightness; default same as source")
    parser.add_argument("--min-area", type=int, default=300, help="Minimum contour area, px^2")
    parser.add_argument("--kernel", type=int, default=3, choices=[0, 3, 5], help="Morphology size; 0 disables")
    args = parser.parse_args()
    if not (1 <= args.hue_width <= 40 and 0 <= args.s_min <= 255 and 0 <= args.v_min <= 255):
        parser.error("Hue width must be 1..40; S/V must be 0..255")
    if not 1 <= args.min_area <= 20000:
        parser.error("Min area must be 1..20000")
    settings = Settings(args.hue_width, args.s_min, args.v_min, args.min_area, kernel=args.kernel)
    show_window = not args.no_window
    if show_window and sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        print("DISPLAY not set: terminal-only mode. Use a Pi desktop/VNC to see live images.")
        show_window = False
    def request_stop(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_stop)
    try:
        if args.image:
            run_image(settings, args, show_window)
        else:
            run_live(settings, args, show_window)
    except KeyboardInterrupt:
        print("\nStopped.")
    except (RuntimeError, OSError, cv2.error) as exc:
        print("\nERROR: {}".format(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
