"""Measured tube marks shared by the preview and pixel-to-mm conversion."""

import math

PREVIEW_WIDTH = 448
CAMERA_WIDTH = 640
# Measured from two 640x480 AI frames at the current pipe pose on 2026-11-19.
# +100 mm on screen left, -100 mm on screen right. Recalibrate after a pose change.
PIPE_TICK_X = (54.607, 71.071, 88.949, 104.951, 123.193, 138.243, 155.057, 171.773, 188.692, 205.1,
               222.782, 239.757, 257.138, 273.049, 289.884, 306.292, 322.7, 339.521, 355.586, 373.471, 389.627)
PIPE_TICK_MM = 10
# Current-pose scale enabled at user request; physical mm accuracy remains to check.
CALIBRATION_CONFIRMED = True


def pixel_to_mm(x_px):
    if not math.isfinite(x_px) or not 0 <= x_px < CAMERA_WIDTH:
        raise ValueError("Invalid camera X")
    if len(PIPE_TICK_X) != 21 or not all(
            0 <= a < b < PREVIEW_WIDTH for a, b in zip(PIPE_TICK_X, PIPE_TICK_X[1:])):
        raise ValueError("Expected 21 increasing tube marks")
    x = x_px * PREVIEW_WIDTH / CAMERA_WIDTH
    segment = len(PIPE_TICK_X) - 2
    for index in range(len(PIPE_TICK_X) - 1):
        if x <= PIPE_TICK_X[index + 1]:
            segment = index
            break
    a, b = PIPE_TICK_X[segment:segment + 2]
    # shortcut: extrapolate end segments so crossing +/-100 mm stays observable.
    # Recheck this mapping after any change of camera or tube position.
    return (10 - segment - (x - a) / (b - a)) * PIPE_TICK_MM
