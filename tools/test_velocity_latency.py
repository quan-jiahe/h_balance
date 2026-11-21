"""Velocity accuracy, variable frame times, noise and reversal latency."""
import math
import random
import statistics
import unittest

from test_pipe_calibration import app


def measured(tracker, x, dt):
    return tracker.update((320 + x, 240, 90), dt, 320, 1)


class VelocityLatencyTests(unittest.TestCase):
    def test_variable_frame_interval_and_raw_position(self):
        tracker = app.BallTracker()
        elapsed = 0
        for dt in (33, 20, 45, 30, 90, 150, 25, 33):
            elapsed += dt
            x, v, _, status = measured(tracker, elapsed * 0.04, dt)
            self.assertAlmostEqual(x, elapsed * 0.04)
            if len(tracker.samples) > 1:
                self.assertAlmostEqual(v, 40)
            self.assertEqual(status, app.RAW_VALID)
            self.assertLessEqual(len(tracker.samples), app.VELOCITY_SAMPLES)
            if len(tracker.samples) > 2:
                self.assertLessEqual(tracker.samples[-1][0] - tracker.samples[0][0], 120)

    def test_reversal_reacts_before_old_double_filter(self):
        tracker = app.BallTracker()
        x = filtered = old_v = 0.0
        measured(tracker, x, 33)
        for _ in range(30):
            x += 40 * 0.033
            measured(tracker, x, 33)
            previous = filtered
            filtered += 0.45 * (x - filtered)
            old_v += 0.25 * ((filtered - previous) / 0.033 - old_v)
        new_cross = old_cross = None
        for index in range(1, 16):
            x -= 40 * 0.033
            v = measured(tracker, x, 33)[1]
            previous = filtered
            filtered += 0.45 * (x - filtered)
            old_v += 0.25 * ((filtered - previous) / 0.033 - old_v)
            if v < 0 and new_cross is None:
                new_cross = index
            if old_v < 0 and old_cross is None:
                old_cross = index
        self.assertLessEqual(new_cross, 2)
        self.assertGreater(old_cross, new_cross)

    def test_stationary_noise_does_not_look_like_fast_motion(self):
        tracker = app.BallTracker()
        rng = random.Random(407)
        velocities = [measured(tracker, rng.uniform(-0.3, 0.3), 33)[1] for _ in range(1000)]
        self.assertLess(statistics.pstdev(velocities[10:]), 3)
        self.assertLess(max(abs(v) for v in velocities[10:]), 8)

    def test_long_uptime_and_lost_samples_reset_velocity(self):
        tracker = app.BallTracker()
        measured(tracker, 0, 33)
        tracker.sample_ms = 1 << 40
        tracker.samples = [(tracker.sample_ms, 0)]
        self.assertAlmostEqual(measured(tracker, 2, 50)[1], 40)
        self.assertEqual(measured(tracker, 3, 501), app.LOST_MEASUREMENT)
        self.assertEqual(measured(tracker, 3, 33)[1], 0)
        self.assertEqual(tracker.update(None, 33), app.LOST_MEASUREMENT)
        self.assertEqual(measured(tracker, 3, 33)[1], 0)

    def test_sinusoid_phase_delay_is_lower_than_old_filter(self):
        tracker = app.BallTracker()
        dt, frequency, amplitude = 0.033, 0.5, 40
        filtered, old_v = 0.0, 0.0
        new, old, actual = [], [], []
        for index in range(700):
            t = index * dt
            x = amplitude * math.sin(2 * math.pi * frequency * t)
            v = measured(tracker, x, 33)[1]
            previous = filtered
            filtered += 0.45 * (x - filtered)
            old_v += 0.25 * ((filtered - previous) / dt - old_v)
            new.append(v); old.append(old_v)
            actual.append(2 * math.pi * frequency * amplitude * math.cos(2 * math.pi * frequency * t))
        def lag(values):
            return max(range(10), key=lambda k: sum(actual[i] * values[i + k] for i in range(100, 650)))
        self.assertLess(lag(new), lag(old))
        self.assertLessEqual(lag(new), 2)


if __name__ == '__main__':
    unittest.main()
