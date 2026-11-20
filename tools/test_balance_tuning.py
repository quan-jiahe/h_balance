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
        self.assertEqual(tuner.symbol_address(" g_balance_tuning 0x20000004 Data 52 center_balance.o(.data)"), 0x20000004)
        for text in ("", "g_balance_tuning 0x08000000 Data 52 x", "g_balance_tuning 0x20000000 Data 48 x"):
            with self.assertRaises(ValueError):
                tuner.symbol_address(text)

    def test_rejects_unknown_nonfinite_and_unsafe_gains(self):
        for name, value in (("direction", -1), ("kp", "nan"), ("accel", 0), ("max_offset", 651)):
            with self.assertRaises(ValueError):
                tuner.validate(name, value)

    def test_firmware_signature_and_gain_decode(self):
        values = (0.25, 0.7, 0.875, 1.5, 1.6, 72, 88, 120, 0, 0, 150)
        data = struct.pack(tuner.FORMAT, 0x42414C31, 1, *values)
        self.assertAlmostEqual(tuner.decode(data)["kv"], 0.7)
        for bad in (data[4:], b"\0" * 52, data[:4] + struct.pack("<I", 2) + data[8:]):
            with self.assertRaises(ValueError):
                tuner.decode(bad)

    def test_cli_checks_signature_before_write_and_verifies_ram(self):
        defaults = (0.25, 0.7, 0.875, 1.5, 1.6, 72, 88, 120, 0, 0, 150)
        for good in (False, True):
            memory = bytearray(struct.pack(tuner.FORMAT, 0x42414C31 if good else 0, 1, *defaults))
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
                mapfile.write_text(" g_balance_tuning 0x20000008 Data 52 center_balance.o")
                args = ["tune_balance.py", "--map", str(mapfile), "--set", "kv=0.85"]
                with patch.object(sys, "argv", args), patch.object(tuner.subprocess, "run", fake_cli), patch("builtins.print"):
                    if good:
                        tuner.main()
                        self.assertEqual(writes, [0x20000014])
                        self.assertAlmostEqual(tuner.decode(memory)["kv"], 0.85)
                    else:
                        with self.assertRaises(ValueError):
                            tuner.main()
                        self.assertEqual(writes, [])


if __name__ == "__main__":
    unittest.main()
