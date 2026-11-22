"""Read/write live balance gains through ST-Link. Requires the matching Keil map.

Examples: py tools/tune_balance.py --show
          py tools/tune_balance.py --set kp=0.20 kv=0.65
RAM edits last until reset. Permanent defaults are in center_balance.c.
"""

import argparse
import math
from pathlib import Path
import re
import struct
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CLI = Path(r"D:\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe")
MAP = ROOT / "stm32f407_uart_test/MDK-ARM/Listings/CenterBalance/F407_K230_Center_Balance.map"
FIELDS = ("kp", "kv", "ki", "gain", "brake", "max_speed", "accel", "ref_accel", "kff", "kff2", "max_offset", "brake_speed_gain")
FORMAT = "<II12f"
FLOAT32_MAX = float.fromhex("0x1.fffffep127")


def symbol_address(text):
    match = re.search(r"^\s*g_balance_tuning\s+(0x[0-9a-fA-F]+)\s+Data\s+56\b", text, re.MULTILINE)
    if not match:
        raise ValueError("Matching g_balance_tuning symbol not found; build the center target first")
    address = int(match[1], 16)
    if not (0x20000000 <= address <= 0x2001FFFF - struct.calcsize(FORMAT) + 1) or address % 4:
        raise ValueError("Tuning symbol is outside F407 SRAM")
    return address


def validate(name, value):
    if name not in FIELDS:
        raise ValueError("Unknown parameter: " + name)
    value = float(value)
    if not -FLOAT32_MAX <= value <= FLOAT32_MAX:
        raise ValueError(name + " must fit in a finite float32")
    value = struct.unpack("<f", struct.pack("<f", value))[0]
    if name == "accel" and value <= 0:
        raise ValueError("accel must be positive")
    if name in ("max_speed", "ref_accel", "max_offset", "brake_speed_gain") and value < 0:
        raise ValueError(name + " must be nonnegative")
    return value


def decode(data):
    if len(data) != struct.calcsize(FORMAT):
        raise ValueError("Incomplete RAM read")
    magic, version, *values = struct.unpack(FORMAT, data)
    if magic != 0x42414C31 or version != 2:
        raise ValueError("Board firmware does not match this tuning map; no write performed")
    return {name: validate(name, value) for name, value in zip(FIELDS, values)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--set", nargs="+", default=[], metavar="NAME=VALUE")
    parser.add_argument("--map", type=Path, default=MAP)
    parser.add_argument("--cli", type=Path, default=CLI)
    args = parser.parse_args()
    if not args.show and not args.set:
        parser.print_help()
        return
    changes = {}
    for setting in args.set:
        name, value = setting.split("=", 1)
        changes[name] = validate(name, value)
    address = symbol_address(args.map.read_text(errors="replace"))

    def command(*arguments):
        result = subprocess.run([str(args.cli), "-c", "port=SWD", "freq=100", "mode=HOTPLUG", *arguments],
                                capture_output=True, text=True, errors="replace", timeout=30)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)

    def read():
        with tempfile.TemporaryDirectory(prefix="balance-tune-") as folder:
            output = Path(folder) / "params.bin"
            command("-u", hex(address), str(struct.calcsize(FORMAT)), str(output))
            return decode(output.read_bytes())

    read()  # Validate the firmware signature before any write.
    for name, value in changes.items():
        offset = 8 + 4 * FIELDS.index(name)
        word = struct.unpack("<I", struct.pack("<f", value))[0]
        command("-w32", hex(address + offset), hex(word))
    actual = read()
    for name, requested in changes.items():
        if not math.isclose(actual[name], requested, rel_tol=1e-6, abs_tol=1e-7):
            raise RuntimeError("Write verification failed: " + name)
    for name, value in actual.items():
        print("%-12s = %.6g" % (name, value))
    print("RAM parameters only; reset restores compiled defaults. Direction is fixed at -1.")


if __name__ == "__main__":
    main()
