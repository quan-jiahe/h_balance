"""Real mark positions, UART units, filter dynamics and stale-sample behavior."""

import importlib.util
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
with patch.object(sys, "path", [str(ROOT / "k230")] + sys.path):
    import pipe_calibration as scale
    spec = importlib.util.spec_from_file_location("pipe_main_test", ROOT / "k230/main.py")
    app = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(app)


class PipeCalibrationTests(unittest.TestCase):
    def test_measured_marks_and_between_marks(self):
        for index, preview_x in enumerate(scale.PIPE_TICK_X):
            camera_x = preview_x * 640 / 448
            self.assertAlmostEqual(scale.pixel_to_mm(camera_x), (10 - index) * 10)
        for index in range(20):
            midpoint = (scale.PIPE_TICK_X[index] + scale.PIPE_TICK_X[index + 1]) / 2
            self.assertAlmostEqual(scale.pixel_to_mm(midpoint * 640 / 448), 95 - index * 10)
        self.assertGreater(scale.pixel_to_mm(0), 100)
        self.assertLess(scale.pixel_to_mm(639), -100)

    def test_invalid_pixels_and_malformed_scale(self):
        for x in (-1, 640, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                scale.pixel_to_mm(x)
        with patch.object(scale, "PIPE_TICK_X", (1,) * 21):
            with self.assertRaises(ValueError):
                scale.pixel_to_mm(320)

    def test_confirmation_gate_and_signed_mm_wire_packet(self):
        tracker = app.BallTracker()
        detection = (scale.PIPE_TICK_X[9] * 640 / 448, 240, 90)
        with patch.object(scale, "CALIBRATION_CONFIRMED", False):
            self.assertEqual(tracker.update(detection, 40), app.LOST_MEASUREMENT)
        with patch.object(scale, "CALIBRATION_CONFIRMED", True):
            measured = tracker.update(detection, 40)
        self.assertAlmostEqual(measured[0], 10)
        packet = app.build_frame(7, 123, measured)
        self.assertEqual(struct.unpack("<BBHIhhBB", packet[2:-2]), (1, 12, 7, 123, 10, 0, 90, 1))
        self.assertEqual(int.from_bytes(packet[-2:], "little"), app.crc16(packet[2:-2]))

    def test_filters_preserve_raw_position_and_reset_after_loss(self):
        tracker = app.BallTracker()
        zero = (scale.PIPE_TICK_X[10] * 640 / 448, 240, 90)
        right = (scale.PIPE_TICK_X[11] * 640 / 448, 240, 90)
        with patch.object(scale, "CALIBRATION_CONFIRMED", True):
            self.assertEqual(tracker.update(zero, 40), (0, 0, 90, app.RAW_VALID))
            measured = tracker.update(right, 100)
            x, speed, *_ = measured
            self.assertAlmostEqual(x, -10)
            self.assertAlmostEqual(speed, -100, delta=6)
            packet = app.build_frame(8, 223, measured)
            version, size, sequence, timestamp, position, velocity, confidence, status = struct.unpack(
                "<BBHIhhBB", packet[2:-2])
            self.assertEqual((version, size, sequence, timestamp, position, confidence, status),
                             (1, 12, 8, 223, -10, 90, app.RAW_VALID))
            self.assertEqual(velocity, round(speed))
            self.assertEqual(int.from_bytes(packet[-2:], "little"), app.crc16(packet[2:-2]))
            self.assertEqual(tracker.update(None, 40), app.LOST_MEASUREMENT)
            self.assertEqual(tracker.update(right, 40)[1], 0)
            self.assertEqual(tracker.update(zero, 501), app.LOST_MEASUREMENT)
            self.assertEqual(tracker.update(zero, 40)[1], 0)
            self.assertEqual(tracker.update(right, 0), app.LOST_MEASUREMENT)
            self.assertEqual(tracker.update((320, 240, 49), 40), app.LOST_MEASUREMENT)

    def test_existing_pixel_mode_is_explicit_and_keeps_version_two(self):
        self.assertIsNone(app.CENTER_BALANCE_X_PX)
        measured = app.center_measurement((330, 240, 90), 320)
        self.assertEqual(measured, (10, 0, 90, 1))
        self.assertEqual(app.build_frame(0, 0, measured, center=True)[2], 2)
        self.assertEqual(app.center_measurement((float("nan"), 240, 90), 320), app.LOST_MEASUREMENT)


if __name__ == "__main__":
    unittest.main()
