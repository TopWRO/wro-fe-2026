#!/usr/bin/env python3
"""Red-only shape preview; camera settings match the corrected photo program."""
import argparse
from dataclasses import asdict
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import sys
import time

import cv2
import numpy as np
from red_geometry import Settings, detect_red
from camera_photo import configure_camera

WINDOW = "Red shapes | S save | Q exit"
MASK_WINDOW = "Red mask: white=selected / black=ignored"
OUTPUT = Path(__file__).resolve().parent / "captures"


def analyze(rgb, settings):
    mask, detections = detect_red(rgb, settings)
    detections.sort(key=lambda d: d['metrics']['bbox_xywh'][0])
    for index, detection in enumerate(detections, 1):
        detection['metrics']['id'] = index
    return mask, detections


def draw(rgb, detections):
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    h, w = bgr.shape[:2]
    footer_h = 66 + 23 * len(detections)
    view = np.full((h + footer_h, max(640, w), 3), 28, np.uint8)
    view[:h, :w] = bgr
    for detection in detections:
        m = detection['metrics']
        x, y, bw, bh = m['bbox_xywh']
        cv2.drawContours(view, [detection['contour']], -1, (255, 0, 255), 2)
        cv2.rectangle(view, (x, y), (x + bw - 1, y + bh - 1), (0, 255, 0), 1)
        cv2.polylines(view, [np.rint(detection['box']).astype(np.int32)], True, (0, 255, 255), 1)
        for point in detection['polygon'][:, 0]:
            cv2.circle(view, tuple(int(v) for v in point), 2, (255, 255, 0), -1)
        pos = (x, min(h - 5, y + 18))
        cv2.putText(view, 'RED #{}'.format(m['id']), pos, cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 0, 0), 3)
        cv2.putText(view, 'RED #{}'.format(m['id']), pos, cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)
        line = '#{} W={} H={} | W/H={:.3f} | rotated={:.3f} | corners={}{}'.format(
            m['id'], bw, bh, m['axis_w_over_h'], m['oriented_w_over_h'],
            m['polygon_vertices'], ' CLIPPED' if m['touches_image_border'] else '')
        cv2.putText(view, line, (8, h + 48 + 23 * (m['id'] - 1)), cv2.FONT_HERSHEY_SIMPLEX,
                    .46, (235, 235, 235), 1, cv2.LINE_AA)
    cv2.putText(view, 'Red regions: {} | pixel ratios, not physical height'.format(len(detections)),
                (8, h + 22), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)
    return view


def write_png(path, array):
    ok, encoded = cv2.imencode('.png', array)
    if not ok:
        raise OSError('PNG encoding failed')
    encoded.tofile(str(path))


def save(output, rgb, mask, detections, view, settings):
    output = Path(output).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    prefix = output / datetime.now().strftime('red_%Y%m%d_%H%M%S_%f')
    write_png(Path(str(prefix) + '_raw.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    write_png(Path(str(prefix) + '_mask.png'), mask)
    write_png(Path(str(prefix) + '_result.png'), view)
    data = {'settings': asdict(settings), 'size_wh': [rgb.shape[1], rgb.shape[0]],
            'targets': [d['metrics'] for d in detections]}
    Path(str(prefix) + '.json').write_text(json.dumps(data, indent=2), encoding='utf-8')
    print('Saved: {}_*'.format(prefix))


class Preview:
    def __init__(self, args, settings):
        self.args, self.settings = args, settings
        self.last_print = 0
        self.started = time.monotonic()
        if not args.no_window:
            cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow(MASK_WINDOW, cv2.WINDOW_AUTOSIZE)

    def run(self, rgb):
        if rgb is None:
            if time.monotonic() - self.started > 10:
                raise RuntimeError('No camera image after 10 seconds')
            return
        mask, detections = analyze(rgb, self.settings)
        if time.monotonic() - self.last_print >= .5:
            print('Red targets: {} | '.format(len(detections)) + ' | '.join(
                '#{} W/H={:.3f} rotated={:.3f}'.format(d['metrics']['id'],
                    d['metrics']['axis_w_over_h'], d['metrics']['oriented_w_over_h']) for d in detections))
            self.last_print = time.monotonic()
        if not self.args.no_window:
            view = draw(rgb, detections)
            cv2.imshow(WINDOW, view)
            cv2.imshow(MASK_WINDOW, mask)
            key = cv2.waitKey(1) & 255
            if key in (ord('s'), ord('S')):
                save(self.args.output_dir, rgb, mask, detections, view, self.settings)
            if key in (ord('q'), ord('Q'), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                raise KeyboardInterrupt

    def shutdown(self):
        if not self.args.no_window:
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, help='Analyze a photo without opening the camera')
    parser.add_argument('--no-window', action='store_true')
    parser.add_argument('--output-dir', type=Path, default=OUTPUT)
    parser.add_argument('--s-min', type=int, default=100)
    parser.add_argument('--v-min', type=int, default=70)
    parser.add_argument('--hue-width', type=int, default=10)
    parser.add_argument('--min-area', type=int, default=150)
    args = parser.parse_args()
    if not 0 <= args.s_min <= 255 or not 0 <= args.v_min <= 255 or not 1 <= args.hue_width <= 30 or args.min_area < 1:
        parser.error('S/V must be 0..255, hue-width 1..30, min-area > 0')
    settings = Settings(hue_width=args.hue_width, s_min=args.s_min, v_min=args.v_min, min_area=args.min_area)
    if sys.platform.startswith('linux') and not os.environ.get('DISPLAY'):
        args.no_window = True
    if args.image:
        bgr = cv2.imdecode(np.fromfile(str(args.image.expanduser()), np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            raise OSError('Could not read photo')
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        mask, detections = analyze(rgb, settings)
        view = draw(rgb, detections)
        save(args.output_dir, rgb, mask, detections, view, settings)
        print(json.dumps([d['metrics'] for d in detections], indent=2))
        if not args.no_window:
            try:
                cv2.imshow(WINDOW, view)
                cv2.imshow(MASK_WINDOW, mask)
                cv2.waitKey(0)
            finally:
                cv2.destroyAllWindows()
        return

    import donkeycar as dk
    from donkeycar.parts.camera import PiCamera
    car = dk.vehicle.Vehicle()
    cam = PiCamera(image_w=640, image_h=480, image_d=3)
    try:
        time.sleep(2)
        print(configure_camera(cam))  # Same corrected settings; no optional overrides.
        preview = Preview(args, settings)
    except BaseException:
        cam.shutdown()
        raise
    car.add(cam, outputs=['cam/image_array'], threaded=True)
    car.add(preview, inputs=['cam/image_array'], outputs=[])
    car.start(rate_hz=20)


if __name__ == '__main__':
    def stop(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ImportError, cv2.error) as exc:
        print('ERROR: {}'.format(exc), file=sys.stderr)
        sys.exit(1)
