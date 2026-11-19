"""Collect training photos to the Yahboom CanMV K230 data partition."""

import os
import time

from machine import FPIOA, UART
from media.display import Display
from media.media import MediaManager
from media.sensor import CAM_CHN_ID_0, Sensor
from balance_uart import Commands


DATA_DIR = "/data"
PHOTO_DIR = DATA_DIR + "/steel_ball_photos"
INTERVAL_MS = 500
JPEG_QUALITY = 95
STATUS_TIMEOUT_MS = 3000


def create_batch():
    if "steel_ball_photos" not in os.listdir(DATA_DIR):
        os.mkdir(PHOTO_DIR)
    existing = os.listdir(PHOTO_DIR)
    index = 1
    while "batch_%03d" % index in existing:
        index += 1
    folder = "%s/batch_%03d" % (PHOTO_DIR, index)
    os.mkdir(folder)
    return folder


def run():
    folder = create_batch()
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=640, height=480, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    count = 0
    last_capture_ms = None
    last_status_ms = None
    menu = Commands()
    uart = None
    display_ready = False
    media_ready = False

    os.exitpoint(os.EXITPOINT_ENABLE)
    try:
        fpioa = FPIOA()
        fpioa.set_function(10, fpioa.UART1_RXD, ie=1, oe=0)
        uart = UART(UART.UART1, baudrate=115200, bits=UART.EIGHTBITS,
                    parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE, timeout=0)
        Display.init(Display.ST7701, width=640, height=480, osd_num=1, to_ide=True)
        display_ready = True
        MediaManager.init()
        media_ready = True
        sensor.run()
        print("Saving JPEG photos to", folder)
        while True:
            os.exitpoint()
            frame = sensor.snapshot(chn=CAM_CHN_ID_0)
            started_ms = time.ticks_ms()
            if last_capture_ms is None or time.ticks_diff(started_ms, last_capture_ms) >= INTERVAL_MS:
                filename = "%s/%06d.jpg" % (folder, count + 1)
                frame.save(filename, quality=JPEG_QUALITY)
                last_capture_ms = started_ms
                count += 1
                if count == 1 or count % 20 == 0:
                    print("Saved", filename)

            if uart.any():
                revision = menu.menu_revision
                menu.feed(uart.read(512) or b"")
                if menu.menu_revision != revision:
                    last_status_ms = time.ticks_ms()
            connected = (last_status_ms is not None
                         and time.ticks_diff(time.ticks_ms(), last_status_ms) < STATUS_TIMEOUT_MS)
            lines = menu.menu_lines if connected else ("F407: NO STATUS",)
            frame.draw_rectangle(0, 0, 320, 24 * (len(lines) + 1), color=(0, 0, 0), fill=True)
            frame.draw_string_advanced(8, 2, 20, "Photos: %d" % count, color=(255, 255, 255))
            for index, line in enumerate(lines, 1):
                frame.draw_string_advanced(8, index * 24 + 2, 20, line, color=(255, 255, 255))
            Display.show_image(frame, layer=Display.LAYER_OSD0)
    except KeyboardInterrupt:
        print("Stopped. Photos saved:", count)
    finally:
        for resource, method in (
            (sensor, "stop"), (Display if display_ready else None, "deinit"),
            (MediaManager if media_ready else None, "deinit"), (uart, "deinit"),
        ):
            if resource is not None:
                try:
                    getattr(resource, method)()
                except Exception as exc:
                    print("Cleanup failed:", method, exc)


if __name__ == "__main__":
    run()
