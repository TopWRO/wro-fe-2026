"""Offline graph/image checks. All camera, controller, motor and Tub parts are fake.

Run from the project root:
    python -B -m unittest discover -s recording_color_swap/tests -v
"""
import builtins
import contextlib
import importlib.util
import io
import os
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np


NEW = Path(__file__).resolve().parents[1]
PROJECT = NEW.parent
ORIGINAL = PROJECT / "Temp_test_01/bidirectional_area_model/green_red_live_area_model_320x240.py"


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


processor = load_file("pillar_color_processor", NEW / "pillar_color_processor.py")
original = load_file("_original_bidirectional_for_tests", ORIGINAL)


def scene(offset=0):
    """Separate red/green upright pillars against a colorless background."""
    rgb = np.full((240, 320, 3), 220, np.uint8)
    rgb[40:190, 35 + offset:70 + offset] = (180, 10, 15)
    rgb[60:180, 190 + offset:230 + offset] = (12, 100, 30)
    return rgb


class FakeVehicle:
    def __init__(self):
        self.parts = []
        self.mem = {}

    def add(self, part, inputs=None, outputs=None, threaded=False, run_condition=None):
        self.parts.append(dict(part=part, inputs=inputs or [], outputs=outputs or [],
                               threaded=threaded, run_condition=run_condition))

    def start(self, rate_hz):
        self.started_hz = rate_hz

    def stop(self):
        self.stopped = True

    def run_one(self):
        for entry in self.parts:
            if entry["run_condition"] and not self.mem.get(entry["run_condition"]):
                continue
            part = entry["part"]
            method = part.run_threaded if entry["threaded"] else part.run
            result = method(*(self.mem.get(name) for name in entry["inputs"]))
            outputs = entry["outputs"]
            if len(outputs) == 1:
                self.mem[outputs[0]] = result
            elif outputs:
                for name, value in zip(outputs, result):
                    self.mem[name] = value
        return self.mem


class FakeCamera:
    frame = None

    def __init__(self, **kwargs):
        self.camera = SimpleNamespace()
        self.configuration = kwargs

    def run_threaded(self):
        return self.frame


class FakeLambda:
    def __init__(self, function):
        self.run = function


class FakeJoystick:
    values = (0.4, 0.7, "user")

    def run(self):
        return self.values


class FakeMotor:
    def __init__(self, max_speed=1.0):
        self.max_speed = max_speed
        self.calls = []

    def run(self, angle, throttle):
        self.calls.append((angle, throttle))


class FakeTub:
    def __init__(self, base_path, inputs, types):
        self.base_path, self.inputs, self.types = base_path, inputs, types
        self.rows = []
        self.tub = SimpleNamespace(delete_last_n_records=lambda count: None)

    def run(self, *values):
        row = dict(zip(self.inputs, values))
        row["cam/image_array"] = row["cam/image_array"].copy()
        self.rows.append(row)
        return len(self.rows)


class FakePilot:
    def __init__(self, **kwargs):
        self.input_shape = kwargs["input_shape"]

    def load(self, path):
        self.model_path = path

    def run(self, rgb):
        return 0.1, 0.2


def fake_dependencies():
    names = ["donkeycar", "donkeycar.parts", "donkeycar.parts.camera",
             "donkeycar.parts.transform", "donkeycar.parts.tub_v2",
             "donkeycar.parts.keras", "pygame", "dynamixel_sdk"]
    modules = {name: ModuleType(name) for name in names}
    modules["donkeycar"].vehicle = SimpleNamespace(Vehicle=FakeVehicle)
    modules["donkeycar.parts.camera"].PiCamera = FakeCamera
    modules["donkeycar.parts.transform"].Lambda = FakeLambda
    modules["donkeycar.parts.tub_v2"].TubWriter = FakeTub
    modules["donkeycar.parts.keras"].KerasInterpreter = lambda: object()
    modules["donkeycar.parts.keras"].KerasLinear = FakePilot
    modules["dynamixel_sdk"].__all__ = []
    return modules


@contextlib.contextmanager
def recorder_environment(forbid_processor=False):
    """No production hardware SDK is imported, initialized or constructed."""
    real_import = builtins.__import__

    def guarded_import(name, *args, **kwargs):
        if forbid_processor and name.endswith("pillar_color_processor"):
            raise AssertionError("Disabled recording imported the color processor")
        return real_import(name, *args, **kwargs)

    stubs = fake_dependencies()
    stubs["pillar_color_processor"] = processor
    with patch.dict(sys.modules, stubs), patch.object(builtins, "__import__", guarded_import):
        recorder = load_file("_recording_n03_for_tests", NEW / "recording_n03_color_swap.py")
        with patch.object(recorder, "PS4Joystick", FakeJoystick), \
                patch.object(recorder, "DynamixelDrive", FakeMotor), \
                patch.object(recorder.time, "sleep"), \
                patch.dict(os.environ, {"DISPLAY": ""}), \
                contextlib.redirect_stdout(io.StringIO()):
            FakeCamera.frame = scene()
            FakeJoystick.values = (0.4, 0.7, "user")
            yield recorder


def graph_entry(car, output=None, part_type=None):
    return next(entry for entry in car.parts
                if (output is None or output in entry["outputs"]) and
                (part_type is None or isinstance(entry["part"], part_type)))


class ProcessorChecks(unittest.TestCase):
    def test_original_algorithm_equivalence_across_moving_frames(self):
        new = processor.PillarColorProcessor()
        trackers = {color: original.Stabilizer(original.Rules()) for color in ("red", "green")}
        for index, offset in enumerate((0, 2, 4, 6, 4, 2)):
            rgb = scene(offset)
            unchanged = rgb.copy()
            expected, mask, detections = original.process_frame(rgb, tracker=trackers)
            actual = new.run(rgb)
            with self.subTest(frame=index):
                np.testing.assert_array_equal(actual, expected)
                np.testing.assert_array_equal(new.last_changed_mask, mask)
                np.testing.assert_array_equal(rgb, unchanged)
                self.assertEqual([d["metrics"] for d in new.last_detections],
                                 [d["metrics"] for d in detections])

    def test_both_directions_change_without_moving_geometry_or_background(self):
        for label in ("tall", "low"):
            with self.subTest(label=label), \
                    patch.object(processor, "infer_exported", return_value=(label, 1.0)):
                part = processor.PillarColorProcessor()
                rgb = scene()
                saved = rgb.copy()
                out = part.run(rgb)
                self.assertEqual(out.shape, (240, 320, 3))
                self.assertEqual(out.dtype, np.uint8)
                self.assertGreater(int(out[100, 50, 1]), int(out[100, 50, 0]))
                self.assertGreater(int(out[100, 210, 0]), int(out[100, 210, 1]))
                np.testing.assert_array_equal(rgb, saved)
                np.testing.assert_array_equal(out[part.last_changed_mask == 0],
                                              rgb[part.last_changed_mask == 0])
                self.assertEqual(len(part.last_detections), 2)
                for detection in part.last_detections:
                    metrics = detection["metrics"]
                    self.assertEqual(metrics["render_bbox_xywh"], metrics["bbox_xywh"])
                    self.assertNotEqual(metrics["source_color"], metrics["target_color"])

    def test_standard_and_uncertain_targets_keep_original_pixels(self):
        for prediction in (("stand", 1.0), ("tall", 0.0)):
            with self.subTest(prediction=prediction), \
                    patch.object(processor, "infer_exported", return_value=prediction):
                part = processor.PillarColorProcessor()
                rgb = scene()
                np.testing.assert_array_equal(part.run(rgb), rgb)
                self.assertFalse(part.last_changed_mask.any())

    def test_standard_green_is_protected_next_to_special_red(self):
        def classify(model, metrics):
            return ("tall" if metrics["source_color"] == "red" else "stand"), 1.0
        with patch.object(processor, "infer_exported", side_effect=classify):
            rgb = scene()
            part = processor.PillarColorProcessor()
            out = part.run(rgb)
            np.testing.assert_array_equal(out[60:180, 190:230], rgb[60:180, 190:230])
            self.assertGreater(int(out[100, 50, 1]), int(out[100, 50, 0]))

    def test_tracker_state_is_per_instance_and_resettable(self):
        first, second = processor.PillarColorProcessor(), processor.PillarColorProcessor()
        first.run(scene())
        self.assertTrue(first.tracker["red"].tracks)
        self.assertFalse(second.tracker["red"].tracks)
        self.assertIsNot(first.tracker["red"], second.tracker["red"])
        first.reset()
        self.assertFalse(first.tracker["red"].tracks)
        self.assertIsNone(first.last_changed_mask)
        self.assertEqual(first.last_detections, [])

    def test_none_and_invalid_input_contract(self):
        part = processor.PillarColorProcessor()
        self.assertIsNone(part.run(None))
        for rgb in (np.zeros((120, 160, 3), np.uint8),
                    np.zeros((240, 320, 3), np.float32)):
            with self.subTest(shape=rgb.shape, dtype=rgb.dtype), self.assertRaises(ValueError):
                part.run(rgb)
        with self.assertRaises(TypeError):
            part.run([1, 2, 3])


class RecordingGraphChecks(unittest.TestCase):
    def test_ide_run_without_arguments_uses_source_boolean(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled), \
                    recorder_environment(forbid_processor=not enabled) as recorder:
                recorder.ENABLE_COLOR_SWAP = enabled
                original_builder = recorder.build_vehicle_recording
                vehicles = []

                def capture_vehicle(*args, **kwargs):
                    vehicle = original_builder(*args, **kwargs)
                    vehicles.append(vehicle)
                    return vehicle

                with patch.object(recorder, "build_vehicle_recording", side_effect=capture_vehicle), \
                        patch.object(recorder, "build_vehicle_driving",
                                     side_effect=AssertionError("IDE Run started driving")), \
                        patch.object(recorder.sys, "argv", [str(NEW / "recording_n03_color_swap.py")]), \
                        patch.object(recorder.signal, "signal"):
                    recorder.main()
                self.assertEqual(len(vehicles), 1)
                car = vehicles[0]
                self.assertEqual(car.started_hz, recorder.DRIVE_LOOP_HZ)
                self.assertTrue(car.stopped)
                self.assertEqual(any("cam/color_swapped" in p["outputs"] for p in car.parts), enabled)
                car.run_one()
                self.assertEqual(graph_entry(car, part_type=FakeTub)["part"].rows[0]
                                 ["cam/image_array"].shape, (120, 160, 3))

    def test_disabled_is_original_resize_and_never_loads_processor(self):
        with recorder_environment(forbid_processor=True) as recorder:
            car = recorder.build_vehicle_recording(show_window=False, enable_color_swap=False)
            resize = graph_entry(car, output="cam/image_array")
            self.assertEqual(resize["inputs"], ["cam/raw"])
            car.run_one()
            tub = graph_entry(car, part_type=FakeTub)["part"]
            expected = cv2.resize(FakeCamera.frame, (160, 120))
            np.testing.assert_array_equal(tub.rows[0]["cam/image_array"], expected)
            self.assertFalse(any("cam/color_swapped" in p["outputs"] for p in car.parts))
            self.assertFalse(any(p["threaded"] for p in car.parts if
                                 p["outputs"] == ["cam/image_array"]))

    def test_enabled_converts_native_frame_before_resize_and_tub(self):
        with recorder_environment() as recorder, \
                patch.object(processor, "infer_exported", return_value=("tall", 1.0)):
            car = recorder.build_vehicle_recording(show_window=False, enable_color_swap=True)
            convert = graph_entry(car, output="cam/color_swapped")
            resize = graph_entry(car, output="cam/image_array")
            tub_entry = graph_entry(car, part_type=FakeTub)
            self.assertEqual(convert["inputs"], ["cam/raw"])
            self.assertFalse(convert["threaded"])
            self.assertIsNone(convert["run_condition"])
            self.assertEqual(resize["inputs"], ["cam/color_swapped"])
            self.assertLess(car.parts.index(convert), car.parts.index(resize))
            self.assertLess(car.parts.index(resize), car.parts.index(tub_entry))
            expected_processor = processor.PillarColorProcessor()
            raw = FakeCamera.frame.copy()
            expected = cv2.resize(expected_processor.run(raw), (160, 120))
            car.run_one()
            tub = tub_entry["part"]
            np.testing.assert_array_equal(tub.rows[0]["cam/image_array"], expected)
            self.assertFalse(np.array_equal(expected, cv2.resize(raw, (160, 120))))
            np.testing.assert_array_equal(FakeCamera.frame, raw)
            self.assertEqual(tub.rows[0]["cam/image_array"].shape, (120, 160, 3))
            self.assertEqual((tub.rows[0]["user/angle"], tub.rows[0]["user/throttle"],
                              tub.rows[0]["user/mode"]), FakeJoystick.values)
            self.assertEqual(tub.inputs, ["cam/image_array", "user/angle",
                                          "user/throttle", "user/mode"])
            self.assertEqual(tub.types, ["image_array", "float", "float", "str"])
            motor = graph_entry(car, part_type=FakeMotor)["part"]
            self.assertEqual(motor.max_speed, recorder.RECORD_MAX_SPEED)
            self.assertEqual(motor.calls[-1], FakeJoystick.values[:2])

    def test_original_throttle_threshold_and_continuous_processing(self):
        with recorder_environment() as recorder:
            car = recorder.build_vehicle_recording(show_window=False, enable_color_swap=True)
            part = graph_entry(car, output="cam/color_swapped")["part"]
            tub = graph_entry(car, part_type=FakeTub)["part"]
            with patch.object(part, "run", wraps=part.run) as process:
                for throttle, expected_records in ((0.04, 0), (0.05, 0), (0.06, 1), (-0.07, 2)):
                    FakeJoystick.values = (0.3, throttle, "user")
                    car.run_one()
                    self.assertEqual(len(tub.rows), expected_records)
                self.assertEqual(process.call_count, 4)
            self.assertEqual([r["user/throttle"] for r in tub.rows], [0.06, -0.07])

    def test_external_bool_overrides_editable_default(self):
        with recorder_environment() as recorder:
            recorder.ENABLE_COLOR_SWAP = True
            disabled = recorder.build_vehicle_recording(show_window=False, enable_color_swap=False)
            self.assertFalse(any("cam/color_swapped" in p["outputs"] for p in disabled.parts))
            default = recorder.build_vehicle_recording(show_window=False)
            self.assertTrue(any("cam/color_swapped" in p["outputs"] for p in default.parts))
            with self.assertRaises(TypeError):
                recorder.build_vehicle_recording(show_window=False, enable_color_swap="false")

    def test_namespace_package_import_supports_external_on_and_off(self):
        stubs = fake_dependencies()
        stubs["recording_color_swap.pillar_color_processor"] = processor
        with patch.dict(sys.modules, stubs), \
                patch.object(sys, "path", [str(PROJECT), *sys.path]):
            sys.modules.pop("recording_color_swap.recording_n03_color_swap", None)
            recorder = importlib.import_module("recording_color_swap.recording_n03_color_swap")
            self.assertEqual(recorder.__package__, "recording_color_swap")
            with patch.object(recorder, "PS4Joystick", FakeJoystick), \
                    patch.object(recorder, "DynamixelDrive", FakeMotor), \
                    patch.object(recorder.time, "sleep"), \
                    contextlib.redirect_stdout(io.StringIO()):
                FakeCamera.frame = scene()
                FakeJoystick.values = (0.4, 0.7, "user")
                with patch.object(recorder, "_create_color_processor",
                                  side_effect=AssertionError("Disabled external call loaded processor")):
                    disabled = recorder.build_vehicle_recording(
                        show_window=False, enable_color_swap=False)
                enabled = recorder.build_vehicle_recording(
                    show_window=False, enable_color_swap=True)
                self.assertEqual(graph_entry(disabled, output="cam/image_array")["inputs"],
                                 ["cam/raw"])
                self.assertIsInstance(graph_entry(enabled, output="cam/color_swapped")["part"],
                                      processor.PillarColorProcessor)
                enabled.run_one()
                row = graph_entry(enabled, part_type=FakeTub)["part"].rows[0]
                self.assertEqual(row["cam/image_array"].shape, (120, 160, 3))
                self.assertEqual(row["user/throttle"], 0.7)

    def test_cli_switches_and_custom_tub_directory(self):
        with recorder_environment() as recorder:
            parser = recorder.build_argument_parser()
            self.assertIsNone(parser.parse_args(["--recording"]).color_swap)
            self.assertTrue(parser.parse_args(["--recording", "--color-swap"]).color_swap)
            recorder.ENABLE_COLOR_SWAP = True
            parser = recorder.build_argument_parser()
            self.assertFalse(parser.parse_args(["--recording", "--no-color-swap"]).color_swap)
            car = recorder.build_vehicle_recording(show_window=False, enable_color_swap=False,
                                                    data_path="/tmp/offline-only-no-files")
            self.assertEqual(graph_entry(car, part_type=FakeTub)["part"].base_path,
                             "/tmp/offline-only-no-files")

    def test_driving_does_not_enable_recording_processor(self):
        with recorder_environment(forbid_processor=True) as recorder:
            recorder.ENABLE_COLOR_SWAP = True
            car = recorder.build_vehicle_driving("/tmp/fake-model-never-opened.h5")
            resize = graph_entry(car, output="cam/image_array")
            self.assertEqual(resize["inputs"], ["cam/raw"])
            self.assertFalse(any("cam/color_swapped" in p["outputs"] for p in car.parts))

    def test_driving_cli_ignores_recording_default(self):
        with recorder_environment(forbid_processor=True) as recorder:
            recorder.ENABLE_COLOR_SWAP = True
            car = FakeVehicle()
            with patch.object(recorder, "build_vehicle_driving", return_value=car) as driving, \
                    patch.object(recorder.Path, "is_file", return_value=True), \
                    patch.object(recorder.signal, "signal"):
                recorder.main(["--driving", "--model", "/tmp/fake-model.h5"])
            driving.assert_called_once_with("/tmp/fake-model.h5")
            self.assertEqual(car.started_hz, recorder.DRIVE_LOOP_HZ)
            self.assertTrue(car.stopped)


if __name__ == "__main__":
    unittest.main(verbosity=2)
