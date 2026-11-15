"""Red-only RGB segmentation and apparent pixel geometry. No camera settings."""
from dataclasses import dataclass
import cv2
import numpy as np

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
