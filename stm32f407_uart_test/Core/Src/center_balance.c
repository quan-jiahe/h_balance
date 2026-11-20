#include "center_balance.h"
#include "balance_tuning.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"

enum {
    AXIS_MAX_STEPS = 650, STARTUP_LEVEL_STEPS = 310,
    COARSE_JOG_STEPS = 25, MAX_COMMAND_CHANGE = 8,
    PULSE_INTERVAL_MS = 8, VISION_TIMEOUT_MS = 180,
    STARTUP_TIMEOUT_MS = 20000, FIXED_MOTOR_SIGN = -1
};

_Static_assert(sizeof(BalanceTuning) == 52, "SWD tuning layout changed");

volatile BalanceTuning g_balance_tuning = {
    0x42414C31u, 1u,
    0.25f, 0.70f, 0.875f, 1.50f, 1.60f, 72.0f, 88.0f, 120.0f,
    0.0f, 0.0f, 150.0f
};

typedef struct {
    float reference, integral, previous_error, demand;
    uint32_t sample_ms;
    bool initialized, settled, braking;
} BallController;

static BallController ball;
static uint32_t seen_observations, last_pulse_ms, started_ms;
static int32_t target_steps, level_steps, anchor_steps;
static bool startup_motion, armed, active, faulted;

static float clamp(float value, float min, float max)
{
    return fminf(fmaxf(value, min), max);
}

static bool vision_ready(const LinkTest *link, uint32_t now)
{
    return link->center_valid && now - link->center_rx_ms < VISION_TIMEOUT_MS;
}

static int32_t bounded_target(int32_t step)
{
    return step < 0 ? 0 : (step > AXIS_MAX_STEPS ? AXIS_MAX_STEPS : step);
}

static void hold_here(const LinkTest *link)
{
    target_steps = anchor_steps = link->steps;
    ball = (BallController){0};
    active = false;
}

/* The K230 already estimates mm/s; do not differentiate/filter it a second time. */
static void update_target(const LinkTest *link)
{
    const float position = link->center_error_mm;
    const float velocity = link->center_velocity_mm_s;
    const float error = -position;
    const float distance = fabsf(error), speed = fabsf(velocity);
    float dt = 0.033f;
    if (ball.initialized) {
        dt = clamp((link->center_sample_ms - ball.sample_ms) * 0.001f, 0.010f, 0.500f);
    }
    ball.sample_ms = link->center_sample_ms;
    ball.initialized = true;
    ball.settled = ball.settled ? distance < 4.0f && speed < 10.0f
                               : distance <= 2.0f && speed <= 4.0f;
    if (ball.settled) {
        /* Latch the actual pulse position; 310 is never a balance return target. */
        target_steps = anchor_steps = link->steps;
        ball.reference = ball.integral = ball.demand = 0.0f;
        ball.previous_error = error;
        ball.braking = false;
        return;
    }
    const float direction = error > 0.0f ? 1.0f : (error < 0.0f ? -1.0f : 0.0f);
    const float acceleration = g_balance_tuning.accel;
    const float allowed = fminf(g_balance_tuning.max_speed,
        sqrtf(2.0f * acceleration * fmaxf(0.0f, distance - 2.0f)));
    if (ball.reference * direction < 0.0f) {
        ball.reference = 0.0f;
    } else {
        ball.reference = direction * fminf(allowed,
            fabsf(ball.reference) + g_balance_tuning.ref_accel * dt);
    }
    const float toward = velocity * direction;
    const float stopping_distance = toward * toward / (2.0f * acceleration);
    ball.braking = toward > 0.0f && stopping_distance >= distance * (ball.braking ? 0.55f : 0.85f);
    if (error * ball.previous_error < 0.0f) { ball.integral *= 0.25f; }
    if (distance < 10.0f && speed < 14.0f && !ball.braking) {
        ball.integral = clamp(ball.integral + g_balance_tuning.ki * error * dt, -6.0f, 6.0f);
    } else {
        ball.integral *= 0.92f;
    }
    const float kv = g_balance_tuning.kv * (ball.braking ? g_balance_tuning.brake : 1.0f);
    float demand = g_balance_tuning.gain * (g_balance_tuning.kp * error
        + kv * (ball.reference - velocity) + ball.integral)
        + g_balance_tuning.kff * velocity + g_balance_tuning.kff2 * velocity * speed;
    if (distance >= 90.0f) { demand = direction * g_balance_tuning.max_offset; }
    demand = clamp(demand, -g_balance_tuning.max_offset, g_balance_tuning.max_offset);
    const float physical = clamp(anchor_steps + FIXED_MOTOR_SIGN * demand, 0.0f, AXIS_MAX_STEPS);
    demand = FIXED_MOTOR_SIGN * (physical - anchor_steps);
    ball.demand += clamp(demand - ball.demand, -MAX_COMMAND_CHANGE, MAX_COMMAND_CHANGE);
    target_steps = bounded_target(anchor_steps + FIXED_MOTOR_SIGN * (int32_t)lroundf(ball.demand));
    ball.previous_error = error;
}

int32_t center_balance_level_steps(void) { return level_steps; }

void center_balance_init(void)
{
    ball = (BallController){0};
    active = faulted = false;
    armed = startup_motion = true;
    target_steps = AXIS_MAX_STEPS;
    level_steps = -1;
    seen_observations = last_pulse_ms = 0;
    started_ms = board_millis();
}

void center_balance_key(const LinkTest *link, uint8_t key, uint32_t now)
{
    (void)now;
    if (key == LINK_KEY_BACK || (key == LINK_KEY_OK && armed && !startup_motion)) {
        armed = startup_motion = false;
        hold_here(link);
    } else if (startup_motion || faulted || link->step_fault) {
        return;
    } else if (level_steps < 0) {
        switch (key) {
        case LINK_KEY_UP: target_steps = bounded_target(target_steps + COARSE_JOG_STEPS); break;
        case LINK_KEY_DOWN: target_steps = bounded_target(target_steps - COARSE_JOG_STEPS); break;
        case LINK_JOG_FORWARD: target_steps = bounded_target(target_steps + 1); break;
        case LINK_JOG_BACK: target_steps = bounded_target(target_steps - 1); break;
        case LINK_KEY_OK:
            if (link->steps == target_steps && link->steps > 0 && link->steps < AXIS_MAX_STEPS) {
                level_steps = link->steps;
                hold_here(link);
                armed = true;
                seen_observations = link->center_observations;
            }
            break;
        default: break;
        }
    } else if (key == LINK_KEY_OK) {
        hold_here(link);
        armed = true;
        seen_observations = link->center_observations;
    }
}

void center_balance_tick(LinkTest *link, uint32_t now)
{
    if (faulted || link->step_fault) { return; }
    if (startup_motion && now - started_ms >= STARTUP_TIMEOUT_MS) {
        faulted = true;
        hold_here(link);
        return;
    }
    if (!startup_motion && level_steps >= 0) {
        if (!armed) { return; }
        if (!vision_ready(link, now)) {
            if (active) { hold_here(link); }
            return;
        }
        if (seen_observations != link->center_observations) {
            seen_observations = link->center_observations;
            if (abs(link->center_error_mm) > 100) {
                faulted = true;
                hold_here(link);
                return;
            }
            if (!active || (ball.initialized &&
                    link->center_sample_ms - ball.sample_ms >= VISION_TIMEOUT_MS)) {
                hold_here(link);
            }
            active = true;
            update_target(link);
        }
    }
    if (link->steps != target_steps && now - last_pulse_ms >= PULSE_INTERVAL_MS) {
        const bool forward = link->steps < target_steps;
        if (forward ? link->steps < AXIS_MAX_STEPS : link->steps > 0) {
            link_test_jog(link, forward);
        }
        last_pulse_ms = now;
        if (startup_motion && link->steps == target_steps) {
            if (target_steps == AXIS_MAX_STEPS) {
                target_steps = STARTUP_LEVEL_STEPS;
            } else {
                level_steps = STARTUP_LEVEL_STEPS; /* Display/IMU reference only. */
                startup_motion = false;
                hold_here(link);
                seen_observations = link->center_observations;
                link->center_valid = false;
            }
        }
    }
}

void center_balance_menu(LinkTest *link, uint32_t now)
{
    const char *status = link->step_fault || faulted ? "BAL: S5 FAULT"
        : startup_motion ? (target_steps == AXIS_MAX_STEPS ? "BAL: STARTUP UP" : "BAL: RETURN 310")
        : level_steps < 0 ? "BAL: SET LEVEL"
        : !armed ? "BAL: S4 STOPPED"
        : active && vision_ready(link, now) ? (ball.settled ? "BAL: S3 HOLD ZERO" : "BAL: S2 ZERO")
        : "BAL: S1 WAIT BALL";
    snprintf(link->startup_rows[0], 22, "%s", status);
    if (vision_ready(link, now) && !startup_motion) {
        snprintf(link->startup_rows[1], 22, "X:%+d V:%+d mm/s", link->center_error_mm, link->center_velocity_mm_s);
    } else {
        snprintf(link->startup_rows[1], 22, "BACK:STOP");
    }
    snprintf(link->startup_rows[2], 22, "STEP:%ld/%d", (long)link->steps, AXIS_MAX_STEPS);
}
