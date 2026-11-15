#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V2: live green texture with two-face base geometry for oblique/near pillars.
Run in the SAME Raspberry Pi Python environment as Recording_N02.py.
S: save; R: sample a visible real green pillar; D: debug overlay; Q/Esc: quit.
"""
import argparse
import base64
from dataclasses import dataclass, asdict
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import sys
import time
import cv2
import numpy as np

REFERENCE_PNG_BASE64 = (
    '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'
)

@dataclass
class Settings:
    # Same red HSV defaults as Recording_N02.py, including the hue wraparound.
    hue_width: int = 10
    s_min: int = 100
    v_min: int = 70
    min_area: int = 150
    min_size: int = 12
    kernel: int = 3

def detect_red(rgb, settings):
    """Return cleaned mask and up to twenty largest independent red regions.

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
        if len(detections) == 20:
            break
    return mask, detections


@dataclass
class Rules:
    normal_min: float = 0.44
    normal_max: float = 0.60
    valid_min: float = 0.20
    valid_max: float = 1.60
    edge_pixels: int = 2
    edge_s_min: int = 40
    edge_v_min: int = 15
    base_geometry: bool = True


def ordered_corners(box):
    top, bottom = np.array_split(box[np.argsort(box[:, 1])], 2)
    top, bottom = top[np.argsort(top[:, 0])], bottom[np.argsort(bottom[:, 0])]
    return np.float32([top[0], top[1], bottom[1], bottom[0]])


def base_geometry(detection):
    """Estimate square footprint width from a visible two-face V-shaped base.

    Works in the target's rotated-box coordinates. With two visible faces,
    their horizontal spans a,b approximately encode the two projections of a
    square footprint; hypot(a,b) reduces the widening due to oblique viewing.
    This is an image heuristic, not calibrated lens or 3-D reconstruction.
    Only accept a pronounced central bottom vertex with BOTH base ends visible.
    """
    result = {'base_v_found': False, 'base_geometry_ratio': None, 'base_corner_points': []}
    m = detection['metrics']
    raw_ratio = m['oriented_w_over_h']
    if not .20 <= raw_ratio <= 1.30:
        return result
    hull = cv2.convexHull(detection['contour'])
    polygon = cv2.approxPolyDP(hull, .015 * cv2.arcLength(hull, True), True).reshape(-1, 2)
    if len(polygon) < 5:
        return result
    transform = cv2.getPerspectiveTransform(ordered_corners(detection['box']),
                                           np.float32([[0,0],[1,0],[1,1],[0,1]]))
    normalized = cv2.perspectiveTransform(polygon.astype(np.float32).reshape(-1,1,2),
                                         transform).reshape(-1,2)
    top_vertices = normalized[normalized[:,1] <= .25, 1]
    if len(top_vertices) < 2:
        return result
    top = float(np.median(top_vertices))
    for i, apex in enumerate(normalized):
        ai, bi = (i-1) % len(polygon), (i+1) % len(polygon)
        if normalized[ai,0] > normalized[bi,0]:
            ai, bi = bi, ai
        left, right = normalized[ai], normalized[bi]
        # Flat bottoms and one small clipped/rounded corner must not count as V.
        if not (.25 <= apex[0] <= .75 and apex[1] >= .94
                and left[0] <= apex[0]-.15 and right[0] >= apex[0]+.15
                and .60 <= left[1] <= apex[1]-.06
                and .60 <= right[1] <= apex[1]-.06):
            continue
        ratio = raw_ratio * float(np.hypot(apex[0]-left[0], right[0]-apex[0])) / (float(apex[1])-top)
        if not .20 <= ratio <= 1.60:
            continue
        return {'base_v_found': True, 'base_geometry_ratio': ratio,
                'base_corner_points': polygon[[ai,i,bi]].astype(int).tolist()}
    return result


def classify(detection, rules, frame_shape):
    """Use clear base geometry when available; otherwise retain aspect rules."""
    m = detection['metrics']
    m.update(base_geometry(detection) if rules.base_geometry else
             {'base_v_found': False, 'base_geometry_ratio': None, 'base_corner_points': []})
    raw = m['oriented_w_over_h']
    ratio = m['base_geometry_ratio'] if m['base_v_found'] else raw
    m['classification_ratio'] = ratio
    m['decision_reason'] = 'two_face_base' if m['base_v_found'] else 'aspect_ratio'
    m['side_clip_allowed'] = False
    clipped = m['touches_image_border']
    if clipped and m['base_v_found'] and ratio < rules.normal_min:
        fh, fw = frame_shape[:2]
        x,y,w,h = m['bbox_xywh']
        points = np.asarray(m['base_corner_points'])
        # Allow only side clipping with the complete base and top/bottom visible.
        # Never extrapolate a missing top, bottom, or base corner.
        m['side_clip_allowed'] = bool(y > 2 and y+h < fh-2 and
            np.all(points[:,0] > 2) and np.all(points[:,0] < fw-3))
    if ((clipped and not m['side_clip_allowed'])
            or not rules.valid_min <= raw <= rules.valid_max
            or not rules.valid_min <= ratio <= rules.valid_max):
        m['decision_reason'] = 'clipped_or_invalid'
        return 'UNKNOWN-KEEP'
    if ratio < rules.normal_min:
        return 'TALL-TEXTURE'
    if ratio > rules.normal_max:
        return 'SHORT-TEXTURE'
    return 'NORMAL-KEEP'


def prepare_reference(bgra):
    """Keep real photographed shading; extend exterior with nearest green pixel."""
    alpha = bgra[:, :, 3]
    contours = cv2.findContours(alpha, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not contours or cv2.countNonZero(alpha) < 100:
        raise ValueError('Green reference is too small')
    contour = max(contours, key=cv2.contourArea)
    if min(cv2.minAreaRect(contour)[1]) < 8:
        raise ValueError('Green reference is too narrow')
    rgb = cv2.cvtColor(bgra[:, :, :3], cv2.COLOR_BGR2RGB)
    _, labels = cv2.distanceTransformWithLabels(
        np.where(alpha != 0, 0, 255).astype(np.uint8), cv2.DIST_L2, 5,
        labelType=cv2.DIST_LABEL_PIXEL)
    texture = rgb[alpha != 0][labels - 1]
    corners = ordered_corners(cv2.boxPoints(cv2.minAreaRect(contour)))
    return {'texture': texture, 'corners': corners, 'cutout_bgra': bgra.copy()}


def built_in_reference():
    """The actual third green pillar in the user's approved photograph."""
    data = np.frombuffer(base64.b64decode(REFERENCE_PNG_BASE64), np.uint8)
    return prepare_reference(cv2.imdecode(data, cv2.IMREAD_UNCHANGED))


def sample_reference(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (40, 60, 30), (85, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    fh, fw = mask.shape
    candidates = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if cv2.contourArea(c) >= 150 and min(w, h) >= 12 and x > 0 and y > 0 and x+w < fw and y+h < fh:
            candidates.append(c)
    if not candidates:
        raise ValueError('No complete real green pillar found; existing reference retained')
    c = max(candidates, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    inside = np.zeros(mask.shape, np.uint8)
    cv2.drawContours(inside, [c], -1, 255, cv2.FILLED)
    alpha = cv2.bitwise_and(inside, mask)[y:y+h, x:x+w]
    bgra = np.dstack([cv2.cvtColor(rgb[y:y+h, x:x+w], cv2.COLOR_RGB2BGR), alpha])
    return prepare_reference(bgra)


def process_frame(rgb, reference, settings=None, rules=None):
    """RGB in, clean RGB out. No input mutation or feedback from converted pixels.

    Shape detection uses the original stricter thresholds. Only the conversion
    edge ring uses relaxed dark-red thresholds, limited to edge_pixels pixels.
    """
    settings, rules = settings or Settings(), rules or Rules()
    _, detections = detect_red(rgb, settings)
    detections.sort(key=lambda d: d['metrics']['bbox_xywh'][0])
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    edge_hue = max(settings.hue_width, 20)
    edge_red = cv2.bitwise_or(
        cv2.inRange(hsv, (0, rules.edge_s_min, rules.edge_v_min), (edge_hue, 255, 255)),
        cv2.inRange(hsv, (160, rules.edge_s_min, rules.edge_v_min), (179, 255, 255)))
    # Slightly purple dark-red fringe is allowed; green/gray background isn't.
    channels = rgb.astype(np.float32)
    plausible_red = (channels[:, :, 0] > channels[:, :, 1]*1.10) & (
        channels[:, :, 0] >= channels[:, :, 2]*0.85)
    edge_red[~plausible_red] = 0
    protected = np.zeros(hsv.shape[:2], np.uint8)
    for index, d in enumerate(detections, 1):
        m = d['metrics']
        m['id'] = index
        decision = classify(d, rules, rgb.shape)
        m['decision'] = decision
        m['changed_pixels'] = 0
        if decision.endswith('KEEP'):
            cv2.drawContours(protected, [d['contour']], -1, 255, cv2.FILLED)
    out = rgb.copy()
    changed = np.zeros(hsv.shape[:2], np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                      (2*rules.edge_pixels+1, 2*rules.edge_pixels+1))
    for d in detections:
        if not d['metrics']['decision'].endswith('TEXTURE'):
            continue
        region = np.zeros(hsv.shape[:2], np.uint8)
        cv2.drawContours(region, [d['contour']], -1, 255, cv2.FILLED)
        # Fill the whole detected silhouette, including very dark internal gaps.
        # Outside it, expand only into nearby plausible dark-red pixels.
        if rules.edge_pixels:
            ring_red = cv2.bitwise_and(cv2.dilate(region, kernel), edge_red)
            region = cv2.bitwise_or(region, ring_red)
        region[protected != 0] = 0
        x, y, w, h = cv2.boundingRect(region)
        if not w or not h:
            continue
        destination = ordered_corners(d['box']) - np.float32([x, y])
        transform = cv2.getPerspectiveTransform(reference['corners'], destination)
        fitted = cv2.warpPerspective(reference['texture'], transform, (w, h),
                                    flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        selected = region[y:y+h, x:x+w] != 0
        out[y:y+h, x:x+w][selected] = fitted[selected]
        changed = cv2.bitwise_or(changed, region)
        d['metrics']['changed_pixels'] = int(cv2.countNonZero(region))
    return out, changed, detections


def debug_view(rgb, detections):
    view = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    for d in detections:
        m = d['metrics']
        cv2.drawContours(view, [d['contour']], -1, (255, 0, 255), 1)
        x, y, _, _ = m['bbox_xywh']
        if m['base_v_found']:
            points = np.asarray(m['base_corner_points'], np.int32)
            cv2.polylines(view, [points], False, (0,255,255), 2)
            for point in points:
                cv2.circle(view, tuple(point), 3, (255,255,0), -1)
        label = '#{} r{:.2f} use{:.2f} {}'.format(
            m['id'], m['oriented_w_over_h'], m['classification_ratio'], m['decision'])
        cv2.putText(view, label, (x, max(16, y-4)), cv2.FONT_HERSHEY_SIMPLEX, .4, (0,0,0), 3)
        cv2.putText(view, label, (x, max(16, y-4)), cv2.FONT_HERSHEY_SIMPLEX, .4, (255,255,255), 1)
    return view


def write_png(path, pixels):
    ok, encoded = cv2.imencode('.png', pixels)
    if not ok:
        raise OSError('Could not encode PNG')
    encoded.tofile(str(path))


def save_result(output_dir, rgb, out, mask, detections, reference, settings, rules, source=None):
    output_dir = Path(output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    prefix = output_dir / datetime.now().strftime('texture_%Y%m%d_%H%M%S_%f')
    write_png(Path(str(prefix)+'_raw.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    write_png(Path(str(prefix)+'_converted.png'), cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
    write_png(Path(str(prefix)+'_mask.png'), mask)
    write_png(Path(str(prefix)+'_debug.png'), debug_view(out, detections))
    write_png(Path(str(prefix)+'_reference.png'), reference['cutout_bgra'])
    Path(str(prefix)+'.json').write_text(json.dumps({
        'source': source, 'settings': asdict(settings), 'rules': asdict(rules),
        'targets': [d['metrics'] for d in detections]}, indent=2), encoding='utf-8')
    print('Saved: {}_*'.format(prefix))
    return prefix


def configure_camera(cam):
    # Exact Recording_N02.py settings/order. No added Picamera2 controls.
    camera = cam.camera
    camera.framerate = 30
    camera.exposure_mode = 'off'
    camera.awb_mode = 'off'
    camera.iso = 100
    camera.shutter_speed = 15000
    camera.exposure_compensation = 0
    camera.awb_gains = (1.2, 1.5)


class LivePreview:
    ORIGINAL = 'Original 640x480'
    CONVERTED = 'Green texture | S save | R reference | D debug | Q exit'
    DEBUG = 'Shape debug'

    def __init__(self, args, reference, settings, rules):
        self.args, self.reference, self.settings, self.rules = args, reference, settings, rules
        self.show_debug = args.debug
        self.started = self.last_print = time.monotonic()
        for title in (self.ORIGINAL, self.CONVERTED):
            cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)

    def run(self, rgb):
        now = time.monotonic()
        if rgb is None:
            if now-self.started > 10:
                raise RuntimeError('No camera image after 10 seconds')
            return
        out, mask, detections = process_frame(rgb, self.reference, self.settings, self.rules)
        cv2.imshow(self.ORIGINAL, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        cv2.imshow(self.CONVERTED, cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
        if self.show_debug:
            cv2.imshow(self.DEBUG, debug_view(out, detections))
        if now-self.last_print >= 1:
            print(' | '.join('#{} raw={:.3f} use={:.3f} {} {}'.format(d['metrics']['id'],
                d['metrics']['oriented_w_over_h'], d['metrics']['classification_ratio'],
                d['metrics']['decision'], d['metrics']['decision_reason']) for d in detections)
                or 'No red target')
            self.last_print = now
        key = cv2.waitKey(1) & 255
        if key in (ord('s'), ord('S')):
            try:
                save_result(self.args.output_dir, rgb, out, mask, detections,
                            self.reference, self.settings, self.rules)
            except (OSError, cv2.error) as exc:
                print('Save failed: {}'.format(exc), file=sys.stderr)
        elif key in (ord('r'), ord('R')):
            try:
                self.reference = sample_reference(rgb)  # Always original, never converted.
                print('Green reference updated from ORIGINAL frame; held until next R.')
            except ValueError as exc:
                print(str(exc))
        elif key in (ord('d'), ord('D')):
            if self.show_debug:
                cv2.destroyWindow(self.DEBUG)
            self.show_debug = not self.show_debug
        elif key in (ord('q'), ord('Q'), 27):
            raise KeyboardInterrupt
        if cv2.getWindowProperty(self.CONVERTED, cv2.WND_PROP_VISIBLE) < 1:
            raise KeyboardInterrupt

    def shutdown(self):
        cv2.destroyAllWindows()


def read_rgb(path):
    bgr = cv2.imdecode(np.fromfile(str(Path(path).expanduser()), np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise OSError('Cannot read image: {}'.format(path))
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, help='Photo test instead of camera')
    parser.add_argument('--reference-image', type=Path, help='Photo containing a real green reference pillar')
    parser.add_argument('--no-window', action='store_true', help='Photo test only')
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parent/'captures')
    parser.add_argument('--normal-min', type=float, default=.44)
    parser.add_argument('--normal-max', type=float, default=.60)
    parser.add_argument('--edge-pixels', type=int, default=2, help='Dark red edge search radius, 0..6')
    parser.add_argument('--edge-s-min', type=int, default=40)
    parser.add_argument('--edge-v-min', type=int, default=15)
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--disable-base-geometry', action='store_true', help='Compare against the old aspect-only rules')
    args = parser.parse_args()
    if not .20 < args.normal_min < args.normal_max < 1.60:
        parser.error('Require .20 < normal-min < normal-max < 1.60')
    if not 0 <= args.edge_pixels <= 6 or not 0 <= args.edge_s_min <= 255 or not 0 <= args.edge_v_min <= 255:
        parser.error('edge-pixels: 0..6; edge-s-min/edge-v-min: 0..255')
    settings = Settings()
    rules = Rules(normal_min=args.normal_min, normal_max=args.normal_max,
                  edge_pixels=args.edge_pixels, edge_s_min=args.edge_s_min, edge_v_min=args.edge_v_min,
                  base_geometry=not args.disable_base_geometry)
    reference = sample_reference(read_rgb(args.reference_image)) if args.reference_image else built_in_reference()
    print('Reference: {}'.format(args.reference_image or 'embedded real green pillar from approved photo'))
    if args.image:
        rgb = read_rgb(args.image)
        out, mask, ds = process_frame(rgb, reference, settings, rules)
        save_result(args.output_dir, rgb, out, mask, ds, reference, settings, rules, str(args.image))
        if not args.no_window:
            try:
                cv2.imshow('Photo result', cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
                cv2.waitKey(0)
            finally:
                cv2.destroyAllWindows()
        return
    if args.no_window:
        parser.error('--no-window is only available with --image')
    if sys.platform.startswith('linux') and not os.environ.get('DISPLAY'):
        parser.error('Use Raspberry Pi desktop/VNC for live preview; DISPLAY is not set')
    import donkeycar as dk
    from donkeycar.parts.camera import PiCamera
    car = dk.vehicle.Vehicle()
    cam = PiCamera(image_w=640, image_h=480, image_d=3)
    try:
        time.sleep(2)
        configure_camera(cam)
        preview = LivePreview(args, reference, settings, rules)
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
    except (OSError, ValueError, RuntimeError, ImportError, cv2.error) as exc:
        print('ERROR: {}'.format(exc), file=sys.stderr)
        sys.exit(1)
