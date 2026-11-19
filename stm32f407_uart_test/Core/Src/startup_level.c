#include "startup_level.h"
#include "board.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

enum {
    MECHANICAL_MAX_STEPS = 650,
    UPPER_MARGIN_STEPS = 10,
    SOFT_MAX_STEPS = MECHANICAL_MAX_STEPS - UPPER_MARGIN_STEPS,
    FAST_PULSE_INTERVAL_MS = 4,
    SLOW_PULSE_INTERVAL_MS = 8,
    SCAN_STEPS = 24,
    BOOT_SETTLE_MS = 220,
    SETTLE_MS = 60,
    LEVEL_SETTLE_MS = 100,
    SAMPLE_MS = 12,
    BOOT_SAMPLES = 40,
    SCAN_SAMPLES = 8,
    LEVEL_SAMPLES = 16
};
#define LEVEL_TOLERANCE_DEG 0.10f
/* shortcut: assumes Z is parallel to the pipe; measure this offset with a level if needed. */
#define LEVEL_ZERO_OFFSET_DEG 0.0f

typedef enum { CHECK_LOW, SWEEP_UP, RETURN_LEVEL, READY, MANUAL, FAULT } Mode;

static Mode mode;
static const char *fault;
static uint32_t last_poll_ms, last_sample_ms, last_move_ms, last_stable_ms, started_ms;
static int32_t move_target, level_step, previous_step, progress_step;
static unsigned samples, stable_count, corrections;
static float longitudinal_sign, angle, previous_angle, progress_angle;
static float sum[3], minimum[3], maximum[3];
static bool angle_valid, crossed_level, fine_return;

static void fail(const char *message)
{
    mode = FAULT;
    fault = message;
}

void startup_level_init(void)
{
    mode = CHECK_LOW;
    move_target = 0;
    crossed_level = false;
    fine_return = false;
    angle_valid = false;
    samples = stable_count = corrections = 0;
    last_poll_ms = last_sample_ms = last_move_ms = last_stable_ms = started_ms = board_millis();
    if (!board_imu_init()) { fail("IMU SPI"); }
}

static bool sample_still(uint32_t now, bool stopped, float *tilt)
{
    if ((uint32_t)(now - last_poll_ms) < SAMPLE_MS) { return false; }
    last_poll_ms = now;
    int16_t raw[3];
    const int status = board_imu_read(raw);
    if (status < 0 || (status == 0 && (uint32_t)(now - last_sample_ms) > 300u)) {
        fail("IMU DATA");
    }
    if (status <= 0) { return false; }
    last_sample_ms = now;
    const bool coarse = mode == SWEEP_UP || (mode == RETURN_LEVEL && !fine_return);
    const uint32_t settle_ms = mode == CHECK_LOW ? BOOT_SETTLE_MS
                             : (coarse ? SETTLE_MS : LEVEL_SETTLE_MS);
    if (!stopped || (uint32_t)(now - last_move_ms) < settle_ms) { return false; }

    float reading[3];
    float magnitude2 = 0.0f, span2 = 0.0f;
    for (unsigned i = 0; i < 3; ++i) {
        reading[i] = raw[i] / 16384.0f;
        magnitude2 += reading[i] * reading[i];
        const float span = fmaxf(maximum[i], reading[i]) - fminf(minimum[i], reading[i]);
        span2 += span * span;
    }
    if (magnitude2 < 0.90f*0.90f || magnitude2 > 1.10f*1.10f) {
        samples = stable_count = 0;
        return false;
    }
    if (samples && span2 > 0.010f*0.010f) {
        samples = stable_count = 0;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!samples) {
            minimum[i] = maximum[i] = reading[i];
            sum[i] = 0.0f;
        }
        minimum[i] = fminf(minimum[i], reading[i]);
        maximum[i] = fmaxf(maximum[i], reading[i]);
        sum[i] += reading[i];
    }
    const unsigned needed = mode == CHECK_LOW ? BOOT_SAMPLES : (coarse ? SCAN_SAMPLES : LEVEL_SAMPLES);
    if (++samples < needed) { return false; }

    /* Remove the extremes of the stationary gravity samples on each axis. */
    for (unsigned i = 0; i < 3; ++i) {
        sum[i] -= minimum[i] + maximum[i];
    }
    *tilt = atan2f(sum[2], sqrtf(sum[0]*sum[0] + sum[1]*sum[1])) * 57.2957795f
            - LEVEL_ZERO_OFFSET_DEG;
    samples = 0;
    return true;
}

static void target(int32_t step)
{
    if (step < 0 || step > SOFT_MAX_STEPS) {
        fail("TRAVEL LIMIT");
    } else {
        move_target = step;
    }
}

static bool check_progress(int32_t step, bool forward)
{
    const float change = forward ? angle - progress_angle : progress_angle - angle;
    if (change >= 0.08f) {
        progress_angle = angle;
        progress_step = step;
    } else if (change < -0.20f || labs((long)step - progress_step) >= 48) {
        fail("NO PROGRESS");
        return false;
    }
    return true;
}

static void sweep(int32_t step)
{
    if (!check_progress(step, true)) { return; }
    if (!crossed_level && angle < 0.0f) {
        previous_step = step;
        previous_angle = angle;
    } else if (!crossed_level) {
        const float fraction = -previous_angle / (angle - previous_angle);
        level_step = previous_step + (int32_t)lroundf(fraction * (step - previous_step));
        crossed_level = true;
    }
    if (step == SOFT_MAX_STEPS) {
        if (!crossed_level || angle <= 0.0f) {
            fail("NO LEVEL IN RANGE");
            return;
        }
        progress_step = step;
        progress_angle = angle;
        mode = RETURN_LEVEL;
        target(step - level_step > SCAN_STEPS ? step - SCAN_STEPS : level_step);
    } else {
        const int32_t next = step + (step == 0 ? 4 : SCAN_STEPS);
        target(next < SOFT_MAX_STEPS ? next : SOFT_MAX_STEPS);
    }
}

static void return_to_level(int32_t step)
{
    if (!fine_return && step > level_step) {
        if (!check_progress(step, false)) { return; }
        if (angle > LEVEL_TOLERANCE_DEG) {
            target(step - level_step > SCAN_STEPS ? step - SCAN_STEPS : level_step);
            return;
        }
    }

    if (!fine_return) {
        fine_return = true;
        return;  /* Verify the position with a longer stationary sample window. */
    }
    if (fabsf(angle) <= LEVEL_TOLERANCE_DEG) {
        if (++stable_count >= 3u) { mode = READY; }
        return;
    }
    stable_count = 0;
    if (++corrections > 64) {
        fail("LEVEL NOT STABLE");
    } else {
        target(step + (angle < 0.0f ? 1 : -1));
    }
}

void startup_level_tick(LinkTest *link, uint32_t now, uint8_t key)
{
    if (mode == FAULT) { return; }
    if (mode < READY && key == 0x43u) { fail("STOPPED"); return; }
    if (mode < READY && (uint32_t)(now - started_ms) > 60000u) {
        fail("LEVEL TIMEOUT");
        return;
    }

    float tilt;
    if (sample_still(now, link->steps == move_target, &tilt)) {
        if (mode == CHECK_LOW) {
            if (fabsf(tilt) <= LEVEL_TOLERANCE_DEG) {
                fail("PLACE AT LOW");
                return;
            }
            longitudinal_sign = tilt < 0.0f ? 1.0f : -1.0f;
            progress_step = link->steps;
            progress_angle = longitudinal_sign * tilt;
            mode = SWEEP_UP;
        }
        angle = longitudinal_sign * tilt;
        angle_valid = true;
        last_stable_ms = now;
        if (mode == SWEEP_UP) {
            sweep(link->steps);
        } else if (mode == RETURN_LEVEL) {
            return_to_level(link->steps);
        } else if (mode == READY && fabsf(angle) > 0.60f) {
            fail("LEVEL DRIFT");
        }
    }
    if (mode == FAULT) { return; }
    if ((uint32_t)(now - last_stable_ms) > 1800u) {
        fail("PIPE MOVING");
        return;
    }
    if (mode >= READY && link->steps == move_target
            && (key == LINK_JOG_FORWARD || key == LINK_JOG_BACK)) {
        const bool forward = key == LINK_JOG_FORWARD;
        if (forward ? link->steps < SOFT_MAX_STEPS : link->steps > 0) {
            target(link->steps + (forward ? 1 : -1));
            mode = MANUAL;
        }
    }
    const uint32_t pulse_interval = mode == SWEEP_UP || (mode == RETURN_LEVEL && !fine_return)
                                  ? FAST_PULSE_INTERVAL_MS : SLOW_PULSE_INTERVAL_MS;
    if (link->steps != move_target && (uint32_t)(now - last_move_ms) >= pulse_interval) {
        link_test_jog(link, link->steps < move_target);
        if (link->step_fault) { fail("STEP PULSE"); }
        last_move_ms = now;
        samples = 0;
    }
}

void startup_level_menu(LinkTest *link)
{
    static const char *const status[] = {
        "HOME: CHECK LOW", "HOME: SWEEP UP", "HOME: RETURN LEVEL",
        "HOME: READY", "HOME: MANUAL", "HOME: FAULT"
    };
    snprintf(link->startup_rows[0], 22, "%s", status[mode]);
    if (mode == FAULT) {
        snprintf(link->startup_rows[1], 22, "%s", fault);
    } else if (angle_valid) {
        const long cents = lroundf(fabsf(angle) * 100.0f);
        snprintf(link->startup_rows[1], 22, "ANGLE:%c%ld.%02ld deg",
                 angle < 0.0f ? '-' : '+', cents / 100, cents % 100);
    } else {
        snprintf(link->startup_rows[1], 22, "WAIT STILL");
    }
    snprintf(link->startup_rows[2], 22, "STEP:%ld/%d%s", (long)link->steps, SOFT_MAX_STEPS,
             mode == READY || mode == MANUAL ? "" : " ERR");
}
