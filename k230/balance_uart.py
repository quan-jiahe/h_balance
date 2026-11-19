"""Shared CanMV decoder for the F407 UART menu and stream commands."""

START = 0x30
STOP = 0x31
MENU_MESSAGE = 0x50
MENU_ROWS = 4
MENU_COLS = 22


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


class Commands:
    def __init__(self):
        self.buffer = bytearray()
        self.menu_lines = ["等待主控菜单", "请连接主控并更新配套程序", "", ""]
        self.menu_revision = 0

    def feed(self, data):
        self.buffer.extend(data)
        command = None
        while self.buffer:
            index = self.buffer.find(b"\xA5\x5A")
            if index < 0:
                self.buffer = self.buffer[-1:] if self.buffer[-1] == 0xA5 else bytearray()
                break
            # MicroPython bytearray supports slicing but not item/slice deletion.
            self.buffer = self.buffer[index:]
            if len(self.buffer) < 4:
                break
            payload_size = self.buffer[3]
            if self.buffer[2] != 1 or payload_size not in (1, 1 + MENU_ROWS * MENU_COLS):
                self.buffer = self.buffer[1:]
                continue
            packet_size = payload_size + 6
            if len(self.buffer) < packet_size:
                break
            packet = self.buffer[:packet_size]
            received_crc = packet[-2] | (packet[-1] << 8)
            if crc16(packet[2:-2]) != received_crc:
                self.buffer = self.buffer[1:]
                continue
            self.buffer = self.buffer[packet_size:]
            if payload_size == 1 and packet[4] in (START, STOP):
                command = packet[4]
            elif payload_size > 1 and packet[4] == MENU_MESSAGE:
                try:
                    lines = [
                        bytes(packet[5 + row * MENU_COLS:5 + (row + 1) * MENU_COLS])
                        .split(b"\0", 1)[0].decode("ascii")
                        for row in range(MENU_ROWS)
                    ]
                except UnicodeError:
                    continue
                self.menu_lines = lines
                self.menu_revision += 1
        return command
