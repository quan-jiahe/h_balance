import importlib.util
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("tuner", Path(__file__).with_name("tune_balance.py"))
tuner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tuner)


class TuningTests(unittest.TestCase):
    def test_map_address_and_exact_struct_size(self):
        self.assertEqual(tuner.symbol_address(" g_balance_tuning 0x20000004 Data 56 center_balance.o(.data)"), 0x20000004)
        for text in ("", "g_balance_tuning 0x08000000 Data 56 x", "g_balance_tuning 0x20000000 Data 48 x"):
            with self.assertRaises(ValueError):
                tuner.symbol_address(text)

    def test_checks_finite_float32_and_math_domain(self):
        for name, value in (("direction", -1), ("kp", "nan"), ("ki", 1e39),
                            ("accel", 0), ("accel", 1e-60), ("max_offset", -1),
                            ("max_speed", -1), ("ref_accel", -1), ("brake_speed_gain", -1)):
            with self.assertRaises(ValueError):
                tuner.validate(name, value)
        for name, value in (("kp", 5.5), ("ki", -1), ("brake", 0.5),
                            ("accel", 400), ("max_offset", 651), ("ref_accel", 0)):
            self.assertEqual(tuner.validate(name, value), value)

    def test_firmware_signature_and_gain_decode(self):
        values = (0.25, 0.7, 0.875, 1.5, 1.6, 72, 88, 120, 0, 0, 150, 0.5)
        data = struct.pack(tuner.FORMAT, 0x42414C31, 2, *values)
        self.assertAlmostEqual(tuner.decode(data)["kv"], 0.7)
        for bad in (data[4:], b"\0" * 52, data[:4] + struct.pack("<I", 1) + data[8:]):
            with self.assertRaises(ValueError):
                tuner.decode(bad)

    def test_cli_checks_signature_before_write_and_verifies_ram(self):
        defaults = (0.25, 0.7, 0.875, 1.5, 1.6, 72, 88, 120, 0, 0, 150, 0.5)
        for good in (False, True):
            memory = bytearray(struct.pack(tuner.FORMAT, 0x42414C31 if good else 0, 2, *defaults))
            writes = []

            def fake_cli(arguments, **kwargs):
                if "-u" in arguments:
                    Path(arguments[-1]).write_bytes(memory)
                else:
                    index = arguments.index("-w32")
                    address, word = (int(v, 16) for v in arguments[index + 1:index + 3])
                    writes.append(address)
                    struct.pack_into("<I", memory, address - 0x20000008, word)
                self.assertIn("mode=HOTPLUG", arguments)
                return SimpleNamespace(returncode=0, stdout="", stderr="")

            with tempfile.TemporaryDirectory() as folder:
                mapfile = Path(folder) / "firmware.map"
                mapfile.write_text(" g_balance_tuning 0x20000008 Data 56 center_balance.o")
                args = ["tune_balance.py", "--map", str(mapfile), "--set", "kv=8.5"]
                with patch.object(sys, "argv", args), patch.object(tuner.subprocess, "run", fake_cli), patch("builtins.print"):
                    if good:
                        tuner.main()
                        self.assertEqual(writes, [0x20000014])
                        self.assertAlmostEqual(tuner.decode(memory)["kv"], 8.5)
                    else:
                        with self.assertRaises(ValueError):
                            tuner.main()
                        self.assertEqual(writes, [])


if __name__ == "__main__":
    unittest.main()
