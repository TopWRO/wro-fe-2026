#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plain 640x480 DonkeyCar PiCamera preview. Press S/s to save a PNG.

Use the same Raspberry Pi Python environment as Recording_N02.py:
    python3 camera_photo.py
"""

import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import sys
import time

import cv2


WIDTH, HEIGHT = 640, 480
BRIGHTNESS = None                # None: leave unchanged, like Recording_N02.py
CONTRAST = None                  # None: leave unchanged, like Recording_N02.py
CAMERA_FPS, LOOP_HZ = 30, 20
SHUTTER_US = 15000               # Same as Recording_N02.py
AWB_GAINS = (1.2, 1.5)          # Same as Recording_N02.py
WINDOW = "Camera preview | S: save photo | Q / Esc: exit"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "photos"


def configure_camera(cam, brightness=None, contrast=None):
    """Reproduce Recording_N02.py's attribute assignments and their order.

    Do NOT translate the source's legacy attributes into Picamera2 exposure/
    white-balance controls. That would change its actual behavior on backends
    where these attributes do not control the sensor. Optional brightness and
    contrast are applied only when the user explicitly requests them.
    """
    camera = cam.camera
    # Exact order and values from Recording_N02.py:add_camera().
    camera.framerate = CAMERA_FPS
    camera.exposure_mode = "off"
    camera.awb_mode = "off"
    camera.iso = 100
    camera.shutter_speed = SHUTTER_US
    camera.exposure_compensation = 0
    camera.awb_gains = AWB_GAINS
    optional_controls = {}
    if brightness is not None:
        if callable(getattr(camera, "set_controls", None)):
            optional_controls["Brightness"] = (brightness - 50) / 50.0
        else:
            camera.brightness = brightness
    if contrast is not None:
        if callable(getattr(camera, "set_controls", None)):
            optional_controls["Contrast"] = (contrast + 100) / 100.0
        else:
            camera.contrast = contrast
    if optional_controls:
        camera.set_controls(optional_controls)
    return "{}.{} (Recording_N02 attribute settings)".format(
        type(camera).__module__, type(camera).__name__)


def save_photo(bgr, output_dir):
    """Save precisely the displayed frame, without annotations or resizing."""
    output_dir = Path(output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / (datetime.now().strftime("photo_%Y%m%d_%H%M%S_%f") + ".png")
    ok, encoded = cv2.imencode(".png", bgr)
    if not ok:
        raise OSError("Could not encode photo")
    # Works for paths containing Chinese on Windows as well as on Raspberry Pi.
    encoded.tofile(str(path))
    return path


class PhotoPreview:
    """Main-thread preview part; camera runs in DonkeyCar's background thread."""

    def __init__(self, output_dir, expected_size=(WIDTH, HEIGHT)):
        self.output_dir = output_dir
        self.expected_size = expected_size
        self.started = time.monotonic()
        self.printed_size = False
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)

    def run(self, rgb):
        if rgb is None:
            if time.monotonic() - self.started > 10:
                raise RuntimeError("No camera image after 10 seconds")
            return
        if not self.printed_size:
            print("Actual image: {}x{}".format(rgb.shape[1], rgb.shape[0]))
            if rgb.shape[:2] != self.expected_size[::-1]:
                print("WARNING: camera did not return the requested {}x{}".format(*self.expected_size))
            self.printed_size = True

        # DonkeyCar gives RGB; OpenCV preview/PNG expects BGR.
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.imshow(WINDOW, bgr)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("s"), ord("S")):
            try:
                print("Saved: {}".format(save_photo(bgr, self.output_dir)))
            except (OSError, cv2.error) as exc:
                print("Could not save photo: {}".format(exc), file=sys.stderr)
        elif key in (ord("q"), ord("Q"), 27):
            raise KeyboardInterrupt
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
    cam = PiCamera(image_w=args.width, image_h=args.height, image_d=3)
    try:
        time.sleep(2)
        backend = configure_camera(cam, args.brightness, args.contrast)
        print("{} | brightness={} contrast={}".format(backend, args.brightness, args.contrast))
        print("None = no override; RGB -> BGR matches Recording_N02.py.")
        print("DonkeyCar camera module: {}".format(sys.modules[PiCamera.__module__].__file__))
        print("Click the preview window, then press S to save; Q / Esc to exit.")
        print("Photo folder: {}".format(args.output_dir.expanduser().resolve()))
        preview = PhotoPreview(args.output_dir, (args.width, args.height))
    except BaseException:
        cam.shutdown()
        raise

    car.add(cam, outputs=["cam/image_array"], threaded=True)
    car.add(preview, inputs=["cam/image_array"], outputs=[])
    car.start(rate_hz=LOOP_HZ)     # DonkeyCar shuts down its parts in finally.


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--brightness", type=int, default=BRIGHTNESS, help="0..100; omitted = unchanged like recording")
    parser.add_argument("--contrast", type=int, default=CONTRAST, help="-100..100; omitted = unchanged like recording")
    parser.add_argument("--width", type=int, default=WIDTH, help="Capture width; default 640; recording used 320")
    parser.add_argument("--height", type=int, default=HEIGHT, help="Capture height; default 480; recording used 240")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT, help="Folder for saved photos")
    args = parser.parse_args()
    if args.brightness is not None and not 0 <= args.brightness <= 100:
        parser.error("--brightness must be 0..100")
    if args.contrast is not None and not -100 <= args.contrast <= 100:
        parser.error("--contrast must be -100..100")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width and --height must be positive")
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
