#!/usr/bin/env python3
"""分析本地图片中的橙色区域（不连接相机或电机）。

用法：python3 analyze_orange_image.py photo.jpg --output /tmp/orange.jpg
依赖：OpenCV、NumPy，与 line_color_detect.py 相同。
HSV 默认阈值仅为起点，需要按实际灯光调整。
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def analyze_orange(bgr, lower=(5, 100, 80), upper=(25, 255, 255), min_area=50):
    """输入 OpenCV BGR 图片，返回统计、去噪掩膜和标注图。

    面积为实际前景像素数；只保留达到 min_area 的连通区域。
    坐标以原图左上角为原点，bbox 为 [x, y, width, height]。
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
    regions = []
    keep = np.zeros(count, dtype=np.uint8)
    annotated = bgr.copy()
    for label in range(1, count):
        x, y, width, height, area = map(int, stats[label])
        if area < min_area:
            continue
        keep[label] = 255
        center = [round(float(v), 2) for v in centers[label]]
        regions.append({"area_pixels": area, "bbox": [x, y, width, height],
                        "center": center})
        cv2.rectangle(annotated, (x, y), (x + width - 1, y + height - 1),
                      (0, 255, 0), 1)
        cv2.putText(annotated, str(area), (x, max(12, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    regions.sort(key=lambda region: region["area_pixels"], reverse=True)
    pixels = sum(region["area_pixels"] for region in regions)
    height, width = bgr.shape[:2]
    return {"width": width, "height": height, "orange_pixels": pixels,
            "orange_percent": round(100 * pixels / (width * height), 3),
            "region_count": len(regions),
            "largest_region": regions[0] if regions else None,
            "regions": regions}, keep[labels], annotated


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="输入图片路径")
    parser.add_argument("--lower", type=int, nargs=3, default=(5, 100, 80),
                        metavar=("H", "S", "V"), help="HSV 下限，默认 5 100 80")
    parser.add_argument("--upper", type=int, nargs=3, default=(25, 255, 255),
                        metavar=("H", "S", "V"), help="HSV 上限，默认 25 255 255")
    parser.add_argument("--min-area", type=int, default=50, help="最小色块像素数")
    parser.add_argument("--output", type=Path, help="保存标注图（不能覆盖原图）")
    parser.add_argument("--mask", type=Path, help="保存黑白掩膜图")
    args = parser.parse_args()
    if args.min_area < 1:
        parser.error("--min-area 必须大于 0")
    if any(not 0 <= lo <= hi <= limit for lo, hi, limit in
           zip(args.lower, args.upper, (179, 255, 255))):
        parser.error("HSV 须满足 0 <= 下限 <= 上限；H 最大 179，S/V 最大 255")
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
