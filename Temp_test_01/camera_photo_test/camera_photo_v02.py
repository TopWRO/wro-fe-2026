#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Photo capture v02: 160x120 photos exactly as the driving model sees them.

Same image path as Recording_N01.py and ai_run in AFI_CUP_V05.py:
    DonkeyCar PiCamera 320x240  ->  cv2.resize to 160x120

Every S press saves TWO lossless PNGs of the same instant:
    photo_<time>_160.png    160x120, the image the model gets
    photo_<time>_320.png    320x240, the camera frame before resizing
into:   photos_160/round<N>/<color>_<pillar>/

Run in the same Raspberry Pi Python environment as Recording_N02.py
(Thonny green Run button, or: python3 camera_photo_v02.py).

Keys (click the preview window first):
    S          save one photo pair
    1 / 2 / 3  pillar height: low 5 cm / stand 10 cm / tall 15 cm
    R / G      pillar colour: red / green
    N          next round (1 -> 2 -> 3 -> 1); re-place car and pillar for a new round
    Q / Esc    quit
"""

import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import sys
import time

import cv2
import numpy as np


CAPTURE_W, CAPTURE_H = 320, 240   # same as Recording_N01.py / AFI_CUP_V05.py
MODEL_W, MODEL_H = 160, 120       # same as CROP_W, CROP_H: image sent to the model
PREVIEW_SCALE = 4                 # 160x120 shown at 640x480 (display only, never saved)
CAMERA_FPS, LOOP_HZ = 30, 20
SHUTTER_US = 15000                # Same as Recording_N02.py
AWB_GAINS = (1.2, 1.5)            # Same as Recording_N02.py

# Labels at start-up. Change them here, or with the keys while the program runs.
COLOR = "red"                     # "red" or "green"
PILLAR = "stand"                  # "low" (5 cm), "stand" (10 cm), "tall" (15 cm)
ROUND = 1                         # 1..3

COLORS = ("red", "green")
PILLARS = ("low", "stand", "tall")
MAX_ROUND = 3
WINDOW = "Photo v02 160x120 | S: save | Q / Esc: exit"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "photos_160"


def configure_camera(cam):
    """Exact attribute assignments and order of Recording_N02.py:add_camera().

    Nothing else is set, so the photos get the same camera behavior as the
    recording and competition programs.
    """
    camera = cam.camera
    camera.framerate = CAMERA_FPS
    camera.exposure_mode = "off"
    camera.awb_mode = "off"
    camera.iso = 100
    camera.shutter_speed = SHUTTER_US
    camera.exposure_compensation = 0
    camera.awb_gains = AWB_GAINS


def to_model_image(rgb):
    """Identical to center_crop() in Recording_N01.py / AFI_CUP_V05.py."""
    return cv2.resize(rgb, (MODEL_W, MODEL_H))


def write_png(path, bgr):
    ok, encoded = cv2.imencode(".png", bgr)
    if not ok:
        raise OSError("Could not encode photo")
    encoded.tofile(str(path))


def save_pair(rgb_capture, rgb_model, folder):
    """Save the model image and the camera frame it was made from."""
    folder.mkdir(parents=True, exist_ok=True)
    prefix = folder / datetime.now().strftime("photo_%Y%m%d_%H%M%S_%f")
    # DonkeyCar gives RGB; PNG writing expects BGR.
    write_png(Path(str(prefix) + "_160.png"), cv2.cvtColor(rgb_model, cv2.COLOR_RGB2BGR))
    write_png(Path(str(prefix) + "_320.png"), cv2.cvtColor(rgb_capture, cv2.COLOR_RGB2BGR))
    return prefix


class PhotoPreview:
    """Main-thread preview part; camera runs in DonkeyCar's background thread."""

    def __init__(self, output_dir, color=COLOR, pillar=PILLAR, round_no=ROUND):
        self.output_dir = Path(output_dir).expanduser()
        self.color, self.pillar, self.round_no = color, pillar, round_no
        self.started = time.monotonic()
        self.printed_size = False
        self.saved = self.count_saved()
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        self.print_label()

    def folder(self):
        return self.output_dir / "round{}".format(self.round_no) / "{}_{}".format(self.color, self.pillar)

    def count_saved(self):
        return len(list(self.folder().glob("*_160.png")))

    def print_label(self):
        self.saved = self.count_saved()
        print("Now: round {} | {} {} | {} photos already in {}".format(
            self.round_no, self.color, self.pillar, self.saved, self.folder()))

    def view(self, rgb_model):
        """Enlarged model image plus a status bar below it (nothing drawn on the image)."""
        bgr = cv2.cvtColor(rgb_model, cv2.COLOR_RGB2BGR)
        big = cv2.resize(bgr, None, fx=PREVIEW_SCALE, fy=PREVIEW_SCALE,
                         interpolation=cv2.INTER_NEAREST)
        bar = np.full((48, big.shape[1], 3), 28, np.uint8)
        status = "round {} | {} {} | saved {}".format(self.round_no, self.color, self.pillar, self.saved)
        keys = "S save | 1 low  2 stand  3 tall | R red  G green | N next round | Q quit"
        cv2.putText(bar, status, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(bar, keys, (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (190, 190, 190), 1)
        return np.vstack([big, bar])

    def handle_key(self, key, rgb_capture, rgb_model):
        if key in (ord("s"), ord("S")):
            try:
                prefix = save_pair(rgb_capture, rgb_model, self.folder())
                self.saved += 1
                print("Saved #{}: {}_160.png + _320.png".format(self.saved, prefix))
            except (OSError, cv2.error) as exc:
                print("Could not save photo: {}".format(exc), file=sys.stderr)
        elif key in (ord("1"), ord("2"), ord("3")):
            self.pillar = PILLARS[key - ord("1")]
            self.print_label()
        elif key in (ord("r"), ord("R")):
            self.color = "red"
            self.print_label()
        elif key in (ord("g"), ord("G")):
            self.color = "green"
            self.print_label()
        elif key in (ord("n"), ord("N")):
            self.round_no = self.round_no % MAX_ROUND + 1
            self.print_label()
        elif key in (ord("q"), ord("Q"), 27):
            raise KeyboardInterrupt

    def run(self, rgb):
        if rgb is None:
            if time.monotonic() - self.started > 10:
                raise RuntimeError("No camera image after 10 seconds")
            return
        if not self.printed_size:
            print("Camera image: {}x{} -> model image: {}x{}".format(
                rgb.shape[1], rgb.shape[0], MODEL_W, MODEL_H))
            if rgb.shape[:2] != (CAPTURE_H, CAPTURE_W):
                print("WARNING: camera did not return the requested {}x{}".format(CAPTURE_W, CAPTURE_H))
            self.printed_size = True

        rgb_model = to_model_image(rgb)
        cv2.imshow(WINDOW, self.view(rgb_model))
        self.handle_key(cv2.waitKey(1) & 0xFF, rgb, rgb_model)
        if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            raise KeyboardInterrupt

    def shutdown(self):
        cv2.destroyAllWindows()


def run_camera(args):
    try:
        import donkeycar as dk
        from donkeycar.parts.camera import PiCamera
    except ImportError as exc:
        raise RuntimeError("Activate the same Python environment as Recording_N02.py. "
                           "Camera import failed: {}".format(exc)) from exc

    car = dk.vehicle.Vehicle()
    cam = PiCamera(image_w=CAPTURE_W, image_h=CAPTURE_H, image_d=3)
    try:
        time.sleep(2)
        configure_camera(cam)
        print("Click the preview window, then press S to save; Q / Esc to exit.")
        preview = PhotoPreview(args.output_dir, args.color, args.pillar, args.round)
    except BaseException:
        cam.shutdown()
        raise

    car.add(cam, outputs=["cam/image_array"], threaded=True)
    car.add(preview, inputs=["cam/image_array"], outputs=[])
    car.start(rate_hz=LOOP_HZ)     # DonkeyCar shuts down its parts in finally.


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--color", choices=COLORS, default=COLOR, help="Pillar colour at start-up")
    parser.add_argument("--pillar", choices=PILLARS, default=PILLAR, help="Pillar height at start-up")
    parser.add_argument("--round", type=int, choices=range(1, MAX_ROUND + 1), default=ROUND,
                        help="Round number at start-up")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT, help="Folder for saved photos")
    args = parser.parse_args()
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        parser.error("A desktop/VNC session is required for the preview and S key; DISPLAY is not set")

    def request_stop(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_stop)
    try:
        run_camera(args)
    except KeyboardInterrupt:
        print("\nStopped.")
    except (RuntimeError, OSError, cv2.error) as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
