"""Collect training photos to the Yahboom CanMV K230 data partition."""

import os
import time

from machine import FPIOA, UART
from media.display import Display
from media.media import MediaManager
from media.sensor import CAM_CHN_ID_0, Sensor
from balance_uart import Commands
from ybUtils.YbRGB import YbRGB


DATA_DIR = "/data"
PHOTO_DIR = DATA_DIR + "/steel_ball_photos"
INTERVAL_MS = 1000
JPEG_QUALITY = 95
STATUS_TIMEOUT_MS = 3000
FLASH_EVERY = 10
FLASH_MS = 180
LED_OFF = (0, 0, 0)


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
    # The board's direct ST7701 preview was verified with packed RGB888.
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
    count = 0
    last_capture_ms = None
    last_status_ms = None
    menu = Commands()
    uart = None
    led = None
    flash_started_ms = None
    display_ready = False
    media_ready = False

    os.exitpoint(os.EXITPOINT_ENABLE)
    try:
        led = YbRGB()
        led.show_rgb(LED_OFF)
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
                frame.to_rgb565().save(filename, quality=JPEG_QUALITY)
                last_capture_ms = started_ms
                count += 1
                if count == 1 or count % 20 == 0:
                    print("Saved", filename)
                if count % FLASH_EVERY == 0:
                    led.show_rgb((0, 255, 0))
                    flash_started_ms = time.ticks_ms()
            if flash_started_ms is not None and time.ticks_diff(time.ticks_ms(), flash_started_ms) >= FLASH_MS:
                led.show_rgb(LED_OFF)
                flash_started_ms = None

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
        if led is not None:
            try:
                led.show_rgb(LED_OFF)
            except Exception as exc:
                print("LED cleanup failed:", exc)
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
