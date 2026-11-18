"""Yahboom K230 steel-ball YOLO11, touch menu and UART; mm calibration pending."""

import math
import struct
import time

WIDTH = 640
HEIGHT = 480
BAUDRATE = 115200
FRAME_PERIOD_MS = 20

# Fill these only after measuring the real camera/platform geometry.
X_ZERO_PX = None
MM_PER_PIXEL = None  # Signed scale: positive x must agree with the controller.

RAW_VALID = 0x01
LOST = 0x20
START = 0x30
STOP = 0x31
KEY_UP = 0x40
KEY_DOWN = 0x41
KEY_OK = 0x42
KEY_BACK = 0x43
MENU_SYNC = 0x44
MENU_MESSAGE = 0x50
MENU_ROWS = 4
MENU_COLS = 22
LOST_MEASUREMENT = (0, 0, 0, LOST)


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def build_frame(sequence, timestamp_ms, measurement):
    x_mm, velocity_mm_s, confidence, status = measurement
    body = struct.pack(
        "<BBHIhhBB", 1, 12, sequence & 0xFFFF, timestamp_ms & 0xFFFFFFFF,
        int(round(x_mm)), int(round(velocity_mm_s)), int(confidence), status,
    )
    return b"\xA5\x5A" + body + struct.pack("<H", crc16(body))


def build_key_frame(key):
    if key not in (KEY_UP, KEY_DOWN, KEY_OK, KEY_BACK, MENU_SYNC):
        raise ValueError("unknown touch event")
    body = bytes((1, 1, key))
    return b"\xA5\x5A" + body + struct.pack("<H", crc16(body))


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


class BallTracker:
    def __init__(self):
        self.reset()

    def reset(self):
        self.last_x = None
        self.velocity = 0.0

    def update(self, detection, elapsed_ms, zero_px, mm_per_pixel):
        # Detection contract: (center_x_px, center_y_px, confidence_0_to_100).
        if detection is None or zero_px is None or mm_per_pixel is None:
            self.reset()
            return LOST_MEASUREMENT
        try:
            x_px, y_px, confidence = detection
            values = (x_px, y_px, confidence, zero_px, mm_per_pixel)
            if not all(math.isfinite(value) for value in values):
                raise ValueError("non-finite detection/calibration")
            if not (0 <= x_px < WIDTH and 0 <= y_px < HEIGHT
                    and 50 <= confidence <= 100 and 0 <= zero_px < WIDTH
                    and mm_per_pixel != 0):
                raise ValueError("invalid detection/calibration")
            # shortcut: one-axis affine calibration; use a homography for perspective distortion.
            x_mm = (x_px - zero_px) * mm_per_pixel
            velocity = 0.0
            if self.last_x is not None:
                if not 0 < elapsed_ms <= 500:
                    raise ValueError("invalid/stale sampling interval")
                raw_velocity = (x_mm - self.last_x) * 1000.0 / elapsed_ms
                velocity = 0.35 * raw_velocity + 0.65 * self.velocity
            if not (-32768 <= x_mm <= 32767 and -32768 <= velocity <= 32767):
                raise ValueError("measurement exceeds wire format")
        except (TypeError, ValueError, OverflowError):
            self.reset()
            return LOST_MEASUREMENT
        self.last_x = x_mm
        self.velocity = velocity
        return x_mm, velocity, confidence, RAW_VALID


def write_frame(uart, packet):
    offset = 0
    while offset < len(packet):
        written = uart.write(packet[offset:])
        if written is None or written <= 0:
            raise OSError("UART write stalled")
        offset += written


def run():
    # Keep hardware imports here so the actual protocol code can be tested on PC.
    import gc
    import os
    from machine import FPIOA, UART
    from media.sensor import Sensor, CAM_CHN_ID_0, CAM_CHN_ID_2
    from media.display import Display
    from media.media import MediaManager
    from touch_ui import TouchMenu, PREVIEW_WIDTH, PREVIEW_HEIGHT
    from steel_ball_detector import SteelBallDetector

    sensor = None
    detector = None
    uart = None
    display_ready = False
    media_ready = False
    ui = None
    sequence = 0
    uptime_ms = 0
    enabled = True  # Stream on boot; the controller can send START/STOP.
    tracker = BallTracker()
    commands = Commands()

    os.exitpoint(os.EXITPOINT_ENABLE)
    try:
        fpioa = FPIOA()
        fpioa.set_function(9, fpioa.UART1_TXD, ie=0, oe=1)
        fpioa.set_function(10, fpioa.UART1_RXD, ie=1, oe=0)
        uart = UART(
            UART.UART1, baudrate=BAUDRATE, bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE, timeout=0,
        )
        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(width=PREVIEW_WIDTH, height=PREVIEW_HEIGHT, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
        sensor.set_framesize(width=WIDTH, height=HEIGHT, chn=CAM_CHN_ID_2)
        sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
        Display.init(Display.ST7701, width=WIDTH, height=HEIGHT, osd_num=1, to_ide=True)
        display_ready = True
        MediaManager.init()
        media_ready = True
        sensor.run()
        detector = SteelBallDetector(WIDTH, HEIGHT)
        ui = TouchMenu(lambda index: write_frame(
            uart, build_key_frame((KEY_UP, KEY_DOWN, KEY_OK, KEY_BACK)[index]),
        ))
        previous_ms = time.ticks_ms()
        last_sync_ms = time.ticks_add(previous_ms, -1000)
        last_menu_ms = None
        print("K230 preview: UART1 IO9/IO10, 115200 8N1")
        print("Steel-ball Model2 loaded: pixel detection active; mm calibration pending")

        while True:
            os.exitpoint()
            loop_start = time.ticks_ms()
            command = commands.feed((uart.read(64) or b"") if uart.any() else b"")
            if ui.menu_revision != commands.menu_revision:
                last_menu_ms = loop_start
                ui.menu_revision = commands.menu_revision
            if command is not None:
                enabled = command == START
                tracker.reset()
            if time.ticks_diff(loop_start, last_sync_ms) >= 1000:
                write_frame(uart, build_key_frame(MENU_SYNC))
                last_sync_ms = loop_start
            frame = sensor.snapshot(chn=CAM_CHN_ID_2)
            now_ms = time.ticks_ms()
            elapsed_ms = time.ticks_diff(now_ms, previous_ms)
            previous_ms = now_ms
            uptime_ms = (uptime_ms + elapsed_ms) & 0xFFFFFFFF
            detection = detector.detect(frame) if enabled else None
            measurement = tracker.update(detection, elapsed_ms, X_ZERO_PX, MM_PER_PIXEL)
            write_frame(uart, build_frame(sequence, uptime_ms, measurement))
            sequence = (sequence + 1) & 0xFFFF

            if not enabled:
                label = "STOPPED - LOST"
            elif X_ZERO_PX is None or MM_PER_PIXEL is None:
                label = ("YOLO X:%.0f Y:%.0f C:%d%%" % detection
                         if detection is not None else "YOLO: NO BALL")
                label += "\nCALIBRATION PENDING"
            elif measurement[3] == LOST:
                label = "NO VALID BALL - LOST"
            else:
                label = "X:%+.1f mm V:%+.1f mm/s" % measurement[:2]
            connected = last_menu_ms is not None and time.ticks_diff(loop_start, last_menu_ms) < 3000
            preview = sensor.snapshot(chn=CAM_CHN_ID_0)
            ui.update(commands.menu_lines, label, connected, preview,
                      detector.last_box if enabled else None)
            gc.collect()
            remaining_ms = FRAME_PERIOD_MS - time.ticks_diff(time.ticks_ms(), loop_start)
            if remaining_ms > 0:
                time.sleep_ms(remaining_ms)
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        if uart is not None:
            try:
                write_frame(uart, build_frame(sequence, uptime_ms, LOST_MEASUREMENT))
            except Exception as exc:
                print("Final LOST frame failed:", exc)
        if ui is not None:
            try:
                ui.close()
            except Exception as exc:
                print("Touch UI cleanup failed:", exc)
        for resource, method in (
            (detector, "deinit"),
            (sensor, "stop"), (Display if display_ready else None, "deinit"),
            (MediaManager if media_ready else None, "deinit"), (uart, "deinit"),
        ):
            if resource is not None:
                try:
                    getattr(resource, method)()
                except Exception as exc:
                    print("Cleanup failed:", method, exc)


if __name__ == "__main__":
    try:
        run()
    except Exception as error:
        import sys
        try:
            with open("/sdcard/balance_error.log", "w") as log:
                sys.print_exception(error, log)
        except OSError:
            pass
        raise
