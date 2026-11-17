"""Native Yahboom output decoding and safe handoff to the existing UART tracker."""

import importlib.util
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


detector = load("steel_detector_test", "k230/steel_ball_detector.py")
protocol = load("ball_protocol_test", "k230/main.py")


class SteelDetectionTests(unittest.TestCase):
    def test_native_xywh_order_and_strongest_steel(self):
        result = [[[10, 20, 20, 10], [100, 200, 40, 20], [0, 0, 30, 30]],
                  [0, 0, 1], [0.6, 0.9, 0.99]]
        self.assertEqual(detector.select_ball(result), (120, 210, 90))
        self.assertEqual(detector.select_ball(result, roi=(0, 0, 50, 50)), (20, 25, 60))

    def test_rejects_empty_wrong_layout_invalid_and_outside(self):
        for result in (None, [], [[], [], []], [[[0, 0, 2, 2, 0.9, 0]]],
                       [[[0, 0, 2, 2]], [], [0.9]]):
            self.assertIsNone(detector.select_ball(result))
        for box, score in (([0, 0, 2, 2], 0.49), ([0, 0, 2, 2], float("nan")),
                           ([0, 0, 2, 2], 1.01), ([640, 0, 2, 2], 0.9),
                           ([0, 0, -1, 2], 0.9), ([float("inf"), 0, 2, 2], 0.9),
                           ([1, 2, 3], 0.9)):
            with self.subTest(box=box, score=score):
                self.assertIsNone(detector.select_ball([[box], [0], [score]]))

    def test_real_detection_stays_lost_until_mm_calibration(self):
        observation = detector.select_ball([[[100, 200, 40, 20]], [0], [0.9]])
        tracker = protocol.BallTracker()
        self.assertEqual(tracker.update(observation, 40, None, None), protocol.LOST_MEASUREMENT)
        self.assertEqual(tracker.update(observation, 40, 100, 0.5), (10, 0, 90, protocol.RAW_VALID))
        self.assertEqual(tracker.update(None, 40, 100, 0.5), protocol.LOST_MEASUREMENT)

    def test_runtime_configuration_rgbp_input_and_cleanup(self):
        model = Mock()
        model.run.return_value = [[[100, 200, 40, 20]], [0], [0.9]]
        factory = Mock(return_value=model)
        with patch.dict(sys.modules, {"libs": SimpleNamespace(__path__=[]),
                                    "libs.YOLO": SimpleNamespace(YOLO11=factory)}):
            instance = detector.SteelBallDetector()
        self.assertEqual(factory.call_args.kwargs["labels"], ["steel"])
        self.assertEqual(factory.call_args.kwargs["model_input_size"], [320, 320])
        model.config_preprocess.assert_called_once_with()
        frame = Mock()
        self.assertEqual(instance.detect(frame), (120, 210, 90))
        model.run.assert_called_once_with(frame.to_numpy_ref.return_value)
        instance.deinit()
        model.deinit.assert_called_once_with()

    def test_model_download_verification_preserves_existing_file(self):
        downloader = load("model_downloader_test", "tools/download_steel_ball_model.py")
        data = b"verified model bytes"
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "k230/kmodel").mkdir(parents=True)
            manifest = {"filename": "model.kmodel", "url": "https://example.invalid/model",
                        "size": len(data), "sha256": hashlib.sha256(data).hexdigest()}
            (root / "k230/model_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
            target = root / "k230/kmodel/model.kmodel"
            target.write_bytes(b"existing")
            with patch.object(downloader, "__file__", str(root / "tools/download.py")):
                with patch.object(downloader.urllib.request, "urlopen", return_value=io.BytesIO(b"bad")):
                    with self.assertRaises(ValueError):
                        downloader.main()
                self.assertEqual(target.read_bytes(), b"existing")
                with patch.object(downloader.urllib.request, "urlopen", return_value=io.BytesIO(data)):
                    downloader.main()
                self.assertEqual(target.read_bytes(), data)
                with patch.object(downloader.urllib.request, "urlopen") as fetch:
                    downloader.main()
                    fetch.assert_not_called()


if __name__ == "__main__":
    unittest.main()
