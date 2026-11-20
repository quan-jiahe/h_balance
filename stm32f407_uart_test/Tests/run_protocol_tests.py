"""Build the real C parser with Keil, execute on a Cortex-M emulator, check K230 decoding.

Optional PC tooling: Python package unicorn; Keil MDK with Arm Compiler 6.
No emulator or Python package is needed to build/flash the normal firmware.
"""

import argparse
import importlib.util
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
        "center_left": k230.build_frame(3, 300, k230.center_measurement((200, 250, 90), 320), center=True),
        "center_right": k230.build_frame(4, 400, k230.center_measurement((440, 250, 90), 320), center=True),
        "center_lost": k230.build_frame(5, 500, k230.LOST_MEASUREMENT, center=True),
        "center_near_left": k230.build_frame(6, 600, k230.center_measurement((318, 250, 90), 320), center=True),
        "center_near_right": k230.build_frame(7, 700, k230.center_measurement((322, 250, 90), 320), center=True),
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
    files = tree.find(".//Group[GroupName='Application']/Files")
    for element in list(files):
        if element.findtext("FileName") == "board.c":
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
    cpu.emu_start(entry, 0, count=35000000)
    failures, checks, done, first_failure = struct.unpack("<IIII", cpu.mem_read(0x2001E000, 16))
    assert done == 0xF4072300, "C test did not finish"
    assert failures == 0, f"C test failure bitmask: {failures:#x}, first failed check: {first_failure}"
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
    assert decoder.menu_lines[1:3] == ["BAL: ACTIVE", "DX:+120px DIR:+1"]
    print(f"PASS: {checks} C checks; real K230 packets -> C parser -> real K230 decoder")
    print("PASS: bad CRC, noise, truncated frames, key limits, tick wrap and link timeout")
    print("PASS: jog direction, one pulse per valid command, count and pulse-fault handling")
    print("PASS: full upward sweep, gravity leveling, return, travel limits, stall, IMU loss and stop")
    print("PASS: two 0-650 pulse cycles with IMU offline, variable speed, no dwell, stop and pulse fault")
    print("PASS: center arming, pixel protocol, sign, travel limit, STOP, vision loss and timeout")
    print("PASS: startup travel 0 -> 650 -> configured level, vision handoff and cancel")


if __name__ == "__main__":
    main()
