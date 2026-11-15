"""Synthetic-image and camera-wiring checks; no real hardware is accessed."""

import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

import red_shape_camera as shape


def frame():
    return np.full((480, 640, 3), 30, np.uint8)


def rectangle(width, height, color=(220, 15, 15), x=200, y=70):
    rgb = frame()
    rgb[y:y + height, x:x + width] = color
    return rgb


class ShapeChecks(unittest.TestCase):
    def detect(self, rgb, settings=None):
        return shape.detect_red(rgb, settings or shape.Settings())

    def test_three_face_on_model_ratios(self):
        for width, height, ratio in [(100, 100, 1.0), (100, 200, 0.5), (100, 300, 1 / 3)]:
            with self.subTest(height=height):
                _, targets = self.detect(rectangle(width, height))
                self.assertEqual(len(targets), 1)
                m = targets[0]['metrics']
                self.assertEqual(m['axis_width_px'], width)
                self.assertEqual(m['axis_height_px'], height)
                self.assertAlmostEqual(m['oriented_w_over_h'], ratio, delta=0.006)
                self.assertEqual(m['polygon_vertices'], 4)

    def test_rgb_red_is_detected_but_rgb_blue_is_not(self):
        self.assertTrue(self.detect(rectangle(100, 200, (255, 0, 0)))[1])
        self.assertFalse(self.detect(rectangle(100, 200, (0, 0, 255)))[1])

    def test_hue_wraparound_and_brightness_variations(self):
        for hue in (0, 5, 175, 179):
            for brightness in (80, 150, 255):
                hsv = np.uint8([[[hue, 220, brightness]]])
                color = tuple(int(v) for v in cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)[0, 0])
                with self.subTest(hue=hue, value=brightness):
                    self.assertTrue(self.detect(rectangle(80, 160, color))[1])

    def test_dark_red_can_be_recovered_with_lower_v(self):
        rgb = rectangle(100, 200, (45, 0, 0))
        self.assertFalse(self.detect(rgb)[1])
        self.assertTrue(self.detect(rgb, shape.Settings(v_min=30))[1])

    def test_small_noise_and_nonred_background_are_filtered(self):
        rgb = frame()
        rgb[50:400, 400:600] = (10, 180, 10)
        rgb[100:104, 100:104] = (255, 0, 0)
        rgb[150:350, 200:300] = (200, 5, 5)
        _, targets = self.detect(rgb)
        self.assertEqual(len(targets), 1)
        self.assertEqual(targets[0]['metrics']['bbox_xywh'], [200, 150, 100, 200])

    def test_separate_objects_remain_separate_and_largest_is_primary(self):
        rgb = rectangle(80, 160)
        rgb[100:180, 400:440] = (220, 15, 15)
        _, targets = self.detect(rgb)
        self.assertEqual(len(targets), 2)
        self.assertGreater(targets[0]['metrics']['contour_area_px'],
                           targets[1]['metrics']['contour_area_px'])
        self.assertEqual(targets[0]['metrics']['axis_width_px'], 80)

    def test_oriented_ratio_handles_camera_roll_and_keeps_wide_shapes(self):
        for size, angle, ratio in [((100, 200), 18, 0.5), ((150, 100), -12, 1.5)]:
            rgb = frame()
            points = cv2.boxPoints(((320, 240), size, angle))
            cv2.fillConvexPoly(rgb, np.rint(points).astype(np.int32), (230, 0, 0))
            _, targets = self.detect(rgb)
            with self.subTest(size=size):
                self.assertAlmostEqual(targets[0]['metrics']['oriented_w_over_h'],
                                       ratio, delta=0.025)

    def test_oblique_six_corner_silhouette_is_not_rejected(self):
        rgb = frame()
        points = np.array([[260, 60], [320, 80], [320, 330],
                           [260, 350], [200, 330], [200, 80]], np.int32)
        cv2.fillConvexPoly(rgb, points, (220, 5, 5))
        _, targets = self.detect(rgb)
        self.assertTrue(targets)
        self.assertGreaterEqual(targets[0]['metrics']['polygon_vertices'], 5)

    def test_border_truncation_is_flagged(self):
        _, targets = self.detect(rectangle(100, 200, x=0))
        self.assertTrue(targets[0]['metrics']['touches_image_border'])

    def test_no_target_does_not_keep_previous_result(self):
        self.assertTrue(self.detect(rectangle(100, 200))[1])
        self.assertEqual(self.detect(frame())[1], [])

    def test_mask_render_and_save_do_not_change_original_rgb(self):
        rgb = rectangle(100, 200)
        original = rgb.copy()
        mask, targets = self.detect(rgb)
        view = shape.render_view(rgb, targets)
        with tempfile.TemporaryDirectory() as temp:
            with contextlib.redirect_stdout(io.StringIO()):
                prefix = shape.save_snapshot(temp, 'normal10', rgb, mask, targets,
                                             view, shape.Settings())
            raw = cv2.imdecode(np.fromfile(str(prefix) + '_raw.png', np.uint8), cv2.IMREAD_COLOR)
            np.testing.assert_array_equal(cv2.cvtColor(raw, cv2.COLOR_BGR2RGB), original)
            data = json.loads(Path(str(prefix) + '_metrics.json').read_text(encoding='utf-8'))
            self.assertEqual(data['frame_size_wh'], [640, 480])
            self.assertEqual(data['targets'][0]['axis_width_px'], 100)
        np.testing.assert_array_equal(rgb, original)

    def test_legacy_camera_settings_match_source(self):
        native = SimpleNamespace()
        shape.configure_camera(SimpleNamespace(camera=native))
        self.assertEqual(native.framerate, 30)
        self.assertEqual(native.shutter_speed, 15000)
        self.assertEqual(native.iso, 100)
        self.assertEqual(native.awb_gains, (1.2, 1.5))
        self.assertEqual(native.exposure_mode, 'off')
        self.assertEqual(native.awb_mode, 'off')

    def test_picamera2_uses_real_controls_instead_of_unused_attributes(self):
        saved = {}
        native = SimpleNamespace(set_controls=lambda values: saved.update(values))
        shape.configure_camera(SimpleNamespace(camera=native))
        self.assertEqual(saved['ExposureTime'], 15000)
        self.assertFalse(saved['AeEnable'])
        self.assertFalse(saved['AwbEnable'])
        self.assertEqual(saved['ColourGains'], (1.2, 1.5))
        self.assertFalse(hasattr(native, 'shutter_speed'))

    def test_live_graph_uses_rgb_640x480_and_only_camera_and_analyzer(self):
        events = {}

        class FakeCamera:
            def __init__(self, **kwargs):
                events['camera_kwargs'] = kwargs
                self.camera = SimpleNamespace()

            def shutdown(self):
                events['camera_stopped'] = True

        class FakeVehicle:
            def __init__(self):
                self.parts = []
                events['parts'] = self.parts

            def add(self, part, **kwargs):
                self.parts.append((part, kwargs))

            def start(self, **kwargs):
                events['loop'] = kwargs
                try:
                    self.parts[1][0].run(rectangle(100, 200))
                finally:
                    for part, _ in self.parts:
                        part.shutdown()

        dk = ModuleType('donkeycar')
        dk.vehicle = SimpleNamespace(Vehicle=FakeVehicle)
        parts = ModuleType('donkeycar.parts')
        camera_module = ModuleType('donkeycar.parts.camera')
        camera_module.PiCamera = FakeCamera
        modules = {'donkeycar': dk, 'donkeycar.parts': parts,
                   'donkeycar.parts.camera': camera_module}
        args = SimpleNamespace(output_dir=shape.DEFAULT_OUTPUT, label='test')
        with patch.dict(sys.modules, modules), patch.object(shape.time, 'sleep'), \
                contextlib.redirect_stdout(io.StringIO()):
            shape.run_live(shape.Settings(), args, show_window=False)
        self.assertEqual(events['camera_kwargs'], {'image_w': 640, 'image_h': 480, 'image_d': 3})
        self.assertEqual(len(events['parts']), 2)
        self.assertTrue(events['parts'][0][1]['threaded'])
        self.assertEqual(events['parts'][1][1]['inputs'], ['cam/image_array'])
        self.assertEqual(events['loop'], {'rate_hz': 20})
        self.assertTrue(events['camera_stopped'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
