"""Run on a Yahboom CanMV K230 to collect training photos to its SD card."""

import os
import time

from media.media import MediaManager
from media.sensor import CAM_CHN_ID_0, Sensor


PHOTO_DIR = "/sdcard/steel_ball_photos"
INTERVAL_MS = 500
JPEG_QUALITY = 95


def create_batch():
    if "steel_ball_photos" not in os.listdir("/sdcard"):
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

    os.exitpoint(os.EXITPOINT_ENABLE)
    MediaManager.init()
    try:
        sensor.run()
        print("Saving JPEG photos to", folder)
        while True:
            os.exitpoint()
            started_ms = time.ticks_ms()
            filename = "%s/%06d.jpg" % (folder, count + 1)
            sensor.snapshot(chn=CAM_CHN_ID_0).save(filename, quality=JPEG_QUALITY)
            count += 1
            if count == 1 or count % 20 == 0:
                print("Saved", filename)

            # shortcut: a slow SD write extends the interval; use buffering for strict timing.
            remaining_ms = INTERVAL_MS - time.ticks_diff(time.ticks_ms(), started_ms)
            if remaining_ms > 0:
                time.sleep_ms(remaining_ms)
    except KeyboardInterrupt:
        print("Stopped. Photos saved:", count)
    finally:
        try:
            sensor.stop()
        finally:
            MediaManager.deinit()


if __name__ == "__main__":
    run()
