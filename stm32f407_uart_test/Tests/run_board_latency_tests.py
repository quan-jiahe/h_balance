"""Execute real board.c against virtual TIM1/DWT/UART registers; never access USB."""
import argparse
from pathlib import Path
import re
import struct
import subprocess
import xml.etree.ElementTree as ET

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_INTR, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_XPSR
from run_protocol_tests import load_hex


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--keil', required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    original = root / 'MDK-ARM/F407_K230_UART.uvprojx'
    build = root / 'Tests/build/board_latency'
    build.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(original)
    targets = tree.find('Targets')
    for target in list(targets)[1:]: targets.remove(target)
    for tag in ('FilePath', 'ScatterFile'):
        for node in tree.findall('.//' + tag):
            node.text = str((original.parent / node.text.replace('\\', '/')).resolve())
    include = tree.find('.//Cads/VariousControls/IncludePath')
    include.text = ';'.join(str((original.parent / x.replace('\\', '/')).resolve()) for x in include.text.split(';'))
    controls = tree.find('.//Cads/VariousControls/MiscControls')
    # Unicorn's memory hooks can resume with stale IT state around a timed register read.
    controls.text += ' -mrestrict-it'
    files = tree.find(".//Group[GroupName='Application']/Files")
    for node in list(files):
        if node.findtext('FileName') == 'main.c':
            node.find('FileName').text = 'test_board_latency.c'
            node.find('FilePath').text = str(root / 'Tests/test_board_latency.c')
        elif node.findtext('FileName') != 'board.c': files.remove(node)
    tree.find('.//OutputName').text = 'BoardLatency'
    tree.find('.//OutputDirectory').text = str(build / 'Objects') + '\\'
    tree.find('.//ListingPath').text = str(build / 'Listings') + '\\'
    project = build / 'BoardLatency.uvprojx'
    tree.write(project, encoding='UTF-8', xml_declaration=True)
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = subprocess.SW_HIDE
    log = build / 'build.log'
    result = subprocess.run([args.keil, '-r', str(project), '-j0', '-o', str(log)], startupinfo=startup, timeout=90)
    print(log.read_text(errors='replace'))
    assert result.returncode == 0, 'Board test build failed'
    symbols = (build / 'Listings/BoardLatency.map').read_text(errors='replace')
    def address(name):
        return int(re.search(r'^\s*' + name + r'\s+(0x[0-9a-fA-F]+)\s', symbols, re.M)[1], 16)
    memory = load_hex(build / 'Objects/BoardLatency.hex')
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    for base, size in ((0x08000000, 0x80000), (0x20000000, 0x20000), (0x40000000, 0x30000),
                       (0xE0001000, 0x1000), (0xE000E000, 0x1000)):
        cpu.mem_map(base, size)
    flash = bytes(memory.get(0x08000000 + i, 255) for i in range(max(memory) - 0x08000000 + 1))
    cpu.mem_write(0x08000000, flash)
    stack, entry = struct.unpack_from('<II', flash)
    cpu.reg_write(UC_ARM_REG_SP, stack)
    cpu.reg_write(UC_ARM_REG_XPSR, 0x01000000)
    cpu.mem_write(0x40023800, struct.pack('<I', 3))
    cycles = wall_cycles = 0
    timer_end = None
    completions, directions, starts = [], [], []
    def word(addr): return struct.unpack('<I', cpu.mem_read(addr, 4))[0]
    def write_word(addr, value): cpu.mem_write(addr, struct.pack('<I', value & 0xFFFFFFFF))
    def timed_read(cpu, access, addr, size, value, user):
        nonlocal cycles, wall_cycles, timer_end
        slept = word(address('test_sleep_ms'))
        if slept:
            wall_cycles += slept * 16000
            write_word(address('test_sleep_ms'), 0)
        cycles += 80  # Deterministic 5 us virtual progress per poll.
        wall_cycles += 80
        write_word(address('ticks_ms'), wall_cycles // 16000)
        write_word(0xE0001004, cycles)
        if timer_end is not None and wall_cycles >= timer_end and not word(address('test_timer_stuck')):
            write_word(0x40010000, word(0x40010000) & ~1)
            write_word(0x40010010, word(0x40010010) | 1)
            completions.append(wall_cycles)
            timer_end = None
    def peripheral_write(cpu, access, addr, size, value, user):
        nonlocal cycles, timer_end
        if addr == 0x40010000:
            if value & 1:
                starts.append(wall_cycles)
                timer_end = wall_cycles + 3200  # Existing 100 us setup + 100 us STEP.
            else: timer_end = None
        elif addr == 0x40021018: directions.append((wall_cycles, value))
        elif addr == 0xE0001004: cycles = value
    for addr in (0xE0001004, 0x40010000, address('ticks_ms')):
        cpu.hook_add(UC_HOOK_MEM_READ, timed_read, begin=addr, end=addr)
    for addr in (0x40010000, 0x40021018, 0xE0001004):
        cpu.hook_add(UC_HOOK_MEM_WRITE, peripheral_write, begin=addr, end=addr)
    cpu.hook_add(UC_HOOK_INTR, lambda cpu, *unused: cpu.emu_stop())
    cpu.emu_start(entry, 0, count=5000000)
    failures, checks, done, first = struct.unpack('<IIII', cpu.mem_read(0x2001E000, 16))
    assert done == 0xF407B04D, 'Board test did not finish'
    assert failures == 0, (failures, checks, first)
    assert len(starts) == 8 and len(completions) == 7
    assert 32000 <= starts[1] - starts[0] < 33600, 'Same-direction pulse interval exceeded 2 ms'
    assert len(directions) == 9
    for index, (at, direction) in enumerate(directions[1:]):
        if index and direction != directions[index][1]:
            assert at - completions[index - 1] >= 32000, 'DIR hold violated'
    print(f'PASS: {checks} real-board driver checks, short pulse call, DIR hold, WFI, timer fault and cycle wrap')
    print('PASS: nonblocking USART3, FIFO order/wrap, whole-frame overflow, IRQ mask and simultaneous RX/TX')


if __name__ == '__main__':
    main()
