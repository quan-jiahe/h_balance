"""Build the real C parser with Keil, execute on a Cortex-M emulator, check K230 decoding.

Optional PC tooling: Python package unicorn; Keil MDK with Arm Compiler 6.
No emulator or Python package is needed to build/flash the normal firmware.
"""

import argparse
import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_INTR
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_XPSR


def load_hex(path):
    memory = {}
    base = 0
    for line in path.read_text().splitlines():
        record = bytes.fromhex(line.removeprefix(":"))
        assert sum(record) & 255 == 0, "HEX checksum"
        size, offset, kind = record[0], int.from_bytes(record[1:3], "big"), record[3]
        assert len(record) == size + 5
        if kind == 4:
            base = int.from_bytes(record[4:6], "big") << 16
        elif kind == 0:
            for index, byte in enumerate(record[4:4 + size]):
                memory[base + offset + index] = byte
    return memory


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("--keil", required=True, type=Path, help="Path to UV4.exe")
    options = args.parse_args()
    root = Path(__file__).resolve().parents[1]
    repo = root.parent
    sys.path.insert(0, str(repo / "k230"))
    build = root / "Tests/build"
    build.mkdir(parents=True, exist_ok=True)
    spec = importlib.util.spec_from_file_location("k230_protocol", repo / "k230/main.py")
    k230 = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(k230)
    fixtures = {
        "up": k230.build_key_frame(k230.KEY_UP),
        "down": k230.build_key_frame(k230.KEY_DOWN),
        "ok": k230.build_key_frame(k230.KEY_OK),
        "back": k230.build_key_frame(k230.KEY_BACK),
        "jog_forward": k230.build_key_frame(k230.TOUCH_KEYS[4]),
        "jog_back": k230.build_key_frame(k230.TOUCH_KEYS[5]),
        "valid": k230.build_frame(0x1234, 0xDEADBEEF, (123, -45, 87, k230.RAW_VALID)),
        "lost": k230.build_frame(2, 200, k230.LOST_MEASUREMENT),
        "center_left": k230.build_frame(3, 300, (-30, 0, 90, 1)),
        "center_right": k230.build_frame(4, 400, (30, 0, 90, 1)),
        "center_lost": k230.build_frame(5, 500, k230.LOST_MEASUREMENT),
        "center_near_left": k230.build_frame(6, 600, (-1, 0, 90, 1)),
        "center_near_right": k230.build_frame(7, 700, (1, 0, 90, 1)),
        "center_fast": k230.build_frame(8, 800, (0, 40, 90, 1)),
        "center_edge": k230.build_frame(9, 900, (101, 0, 90, 1)),
        "center_edge_left": k230.build_frame(11, 950, (-101, 0, 90, 1)),
        "pixel_only": k230.build_frame(10, 1000, (30, 0, 90, 1), center=True),
    }
    (build / "fixtures.h").write_text("\n".join(
        "static const uint8_t fixture_%s[] = {%s};" %
        (name, ",".join(str(byte) for byte in data)) for name, data in fixtures.items()
    ))
    original = root / "MDK-ARM/F407_K230_UART.uvprojx"
    tree = ET.parse(original)
    targets = tree.find("./Targets")
    test_target = targets.find("./Target[TargetName='F407_K230_UART']")
    for target in list(targets):
        if target is not test_target:
            targets.remove(target)
    for tag in ("FilePath", "ScatterFile"):
        for element in tree.findall(".//" + tag):
            element.text = str((original.parent / element.text.replace("\\", "/")).resolve())
    includes = tree.find(".//Cads/VariousControls/IncludePath")
    includes.text = ";".join(str((original.parent / part.replace("\\", "/")).resolve())
                             for part in includes.text.split(";")) + ";" + str(build)
    # Keep the growing test harness within the installed 32 KiB linker license.
    controls = tree.find(".//Cads/VariousControls/MiscControls")
    controls.text = (controls.text or "") + " -Oz"
    files = tree.find(".//Group[GroupName='Application']/Files")
    for element in list(files):
        if element.findtext("FileName") in ("board.c", "vofa_uart.c"):
            files.remove(element)
        elif element.findtext("FileName") == "main.c":
            element.find("FileName").text = "test_link.c"
            element.find("FilePath").text = str(root / "Tests/test_link.c")
    tree.find(".//OutputName").text = "ProtocolTest"
    tree.find(".//OutputDirectory").text = str(build / "Objects") + "\\"
    tree.find(".//ListingPath").text = str(build / "Listings") + "\\"
    project = build / "ProtocolTest.uvprojx"
    tree.write(project, encoding="UTF-8", xml_declaration=True)
    log = build / "build.log"
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = subprocess.SW_HIDE
    process = subprocess.run([str(options.keil), "-r", str(project), "-j0", "-o", str(log)],
                             startupinfo=startup, timeout=90)
    print(log.read_text(errors="replace"))
    assert process.returncode == 0, "Keil test build failed"
    memory = load_hex(build / "Objects/ProtocolTest.hex")
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.mem_map(0x08000000, 0x80000)
    cpu.mem_map(0x20000000, 0x20000)
    cpu.mem_map(0xE000E000, 0x1000)
    flash = bytes(memory.get(0x08000000 + index, 255)
                  for index in range(max(memory) - 0x08000000 + 1))
    cpu.mem_write(0x08000000, flash)
    stack, entry = struct.unpack_from("<II", flash)
    cpu.reg_write(UC_ARM_REG_SP, stack)
    cpu.reg_write(UC_ARM_REG_XPSR, 0x01000000)
    cpu.hook_add(UC_HOOK_INTR, lambda emulator, interrupt, user: emulator.emu_stop())
    cpu.mem_write(0xE000ED88, struct.pack("<I", 0x00F00000))
    cpu.emu_start(entry, 0, count=600000000)
    failures, checks, done, first_failure = struct.unpack("<IIII", cpu.mem_read(0x2001E000, 16))
    model_results = []
    for index in range(18):
        values = struct.unpack('<8f', cpu.mem_read(0x2001E800 + index * 32, 32))
        print('PLANT case=%d phase=%d x=%.2f v=%.2f steps=%.0f peak=%.2f tail_rms=%.2f tail_speed=%.2f hold=%.2f neutral=%.0f' % (index//3, index%3, *values))
        model_results.append(dict(case=index//3, phase=index%3, **dict(zip(
            ('x_mm', 'v_mm_s', 'motor_steps', 'peak_abs_x_mm', 'tail_rms_x_mm',
             'tail_mean_abs_v_mm_s', 'hold_fraction', 'plant_neutral_steps'), values))))
    assert done == 0xF4072300, "C test did not finish"
    for side in range(2):
        entered_ms, hold_peak, edge_peak = struct.unpack('<3f', cpu.mem_read(0x2001EA40 + side * 12, 12))
        print('MODE3 side=%s entry_ms=%.0f hold_peak_mm=%.2f edge_peak_mm=%.2f'
              % ('-' if side == 0 else '+', entered_ms, hold_peak, edge_peak))
    assert failures == 0, f"C test failure bitmask: {failures:#x}, first failed check: {first_failure}"
    (build / 'adaptive_model_results.json').write_text(json.dumps(model_results, indent=2))
    decoder = k230.Commands()
    for offset, expected in ((0x100, "RX:OK"), (0x180, "RX:LOST")):
        packet = bytes(cpu.mem_read(0x2001E000 + offset, 95))
        previous = decoder.menu_revision
        for index in range(0, 95, 7):
            decoder.feed(packet[index:index + 7])
        assert decoder.menu_revision == previous + 1
        assert expected in decoder.menu_lines[0]
        assert decoder.menu_lines[1] == "OBS:1 LOST"
        assert decoder.menu_lines[2] == "KEY:UP V:1"
        assert decoder.menu_lines[3] == "STEP:+1"
        bad_crc = bytearray(packet)
        bad_crc[-1] ^= 1
        decoder.feed(bad_crc)
        assert decoder.menu_revision == previous + 1
    assert decoder.feed(bytes(cpu.mem_read(0x2001E200, 7))) == k230.STOP
    assert decoder.feed(bytes(cpu.mem_read(0x2001E208, 7))) == k230.START
    decoder.feed(bytes(cpu.mem_read(0x2001E300, 95)))
    assert decoder.menu_lines[1:] == ["MOTOR:STOP", "RANGE:0-650", "STEP:+0"]
    decoder.feed(bytes(cpu.mem_read(0x2001E380, 95)))
    assert decoder.menu_lines[1:3] == ["BAL: S2 ZERO", "X:+30 V:+0 mm/s"]
    vofa_packet = bytes(cpu.mem_read(0x2001E480, 76))
    assert vofa_packet[-4:] == bytes.fromhex("0000807f")
    channels = struct.unpack("<18f", vofa_packet[:-4])
    assert channels[:2] == (12, -7) and channels[2:6] == (310, 310, 1, 1)
    assert channels[6] == -3 and abs(channels[7] - 0.5) < 1e-5
    assert abs(channels[8] - 0.7) < 1e-5 and abs(channels[9] - 0.875) < 1e-5
    assert channels[12:15] == (30, 40, 120)
    print("PASS: VOFA JustFloat 18 channels, parameter lock, bounds, errors, timeout and nonblocking telemetry")
    print(f"PASS: {checks} C checks; real K230 packets -> C parser -> real K230 decoder")
    print("PASS: bad CRC, noise, truncated frames, key limits, tick wrap and link timeout")
    print("PASS: jog direction, one pulse per valid command, count and pulse-fault handling")
    print("PASS: full upward sweep, gravity leveling, return, travel limits, stall, IMU loss and stop")
    print("PASS: two 0-650 pulse cycles with IMU offline, variable speed, no dwell, stop and pulse fault")
    print("PASS: mm auto-centering, supplied velocity, fixed sign, latched position, STOP, loss and timeout")
    print("PASS: stalled-ball ramp, zero-crossing/STOP/loss continuity, frozen learning and saturation recovery")
    print("PASS: six idealized mechanical models, 66 ms vision delay, unknown equilibrium, kick and tilt change")
    print("PASS: startup travel 0 -> 650 -> configured level, mode confirmation, fresh vision handoff and cancel")


if __name__ == "__main__":
    main()
