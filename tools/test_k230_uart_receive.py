"""Exercise real received packets with this firmware's bytearray deletion limit."""

import importlib.util
from pathlib import Path
import struct
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location(
    "k230_receive_test", Path(__file__).resolve().parents[1] / "k230/main.py")
protocol = importlib.util.module_from_spec(spec)
spec.loader.exec_module(protocol)


class MicroPythonBytearray(bytearray):
    def __delitem__(self, index):
        raise TypeError("'bytearray' object doesn't support item deletion")

    def __getitem__(self, index):
        value = super().__getitem__(index)
        return type(self)(value) if isinstance(index, slice) else value


def packet(payload):
    body = bytes((1, len(payload))) + payload
    return b"\xA5\x5A" + body + struct.pack("<H", protocol.crc16(body))


def menu():
    lines = ("F407 RX:OK N:123", "OBS:120 LOST", "KEY:UP V:1", "RUN:ON E:0 C:0")
    return packet(b"\x50" + b"".join(line.encode().ljust(22, b"\0") for line in lines))


class ReceiveTests(unittest.TestCase):
    def setUp(self):
        self.compatibility = patch.object(protocol, "bytearray", MicroPythonBytearray, create=True)
        self.compatibility.start()
        self.addCleanup(self.compatibility.stop)
        self.parser = protocol.Commands()

    def test_fragmented_menu_and_commands_without_bytearray_deletion(self):
        data = menu()
        for offset in range(0, len(data), 13):
            self.assertIsNone(self.parser.feed(data[offset:offset + 13]))
        self.assertEqual(self.parser.menu_revision, 1)
        self.assertEqual(self.parser.menu_lines[0], "F407 RX:OK N:123")
        self.assertEqual(self.parser.menu_lines[2], "KEY:UP V:1")
        self.assertEqual(self.parser.feed(packet(bytes((protocol.STOP,)))), protocol.STOP)
        self.assertEqual(self.parser.feed(packet(bytes((protocol.START,)))), protocol.START)
        self.assertEqual(len(self.parser.buffer), 0)

    def test_noise_bad_header_bad_crc_and_continuous_menus_recover(self):
        corrupt = bytearray(menu())
        corrupt[-1] ^= 1
        stream = b"noise\xA5\x5A\x02\x59" + bytes(corrupt) + menu() * 20
        for offset in range(0, len(stream), 64):
            self.parser.feed(stream[offset:offset + 64])
        self.assertEqual(self.parser.menu_revision, 20)
        self.assertEqual(self.parser.menu_lines[3], "RUN:ON E:0 C:0")
        self.assertEqual(len(self.parser.buffer), 0)
        self.parser.feed(b"trailing noise\xA5")
        self.parser.feed(menu()[1:])
        self.assertEqual(self.parser.menu_revision, 21)


if __name__ == "__main__":
    unittest.main()
