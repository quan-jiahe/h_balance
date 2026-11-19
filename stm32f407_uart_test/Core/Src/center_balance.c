#include "center_balance.h"
#include <math.h>
#include <stdio.h>

enum {
    AXIS_MAX_STEPS = 650,
    STARTUP_LEVEL_STEPS = 310,
    COARSE_JOG_STEPS = 25,
    MAX_COMMAND_CHANGE = 8,
    PULSE_INTERVAL_MS = 8,
    VISION_TIMEOUT_MS = 180
};

typedef struct {
    float position;
    float velocity;
    float reference;
    float integral;
    float previous_error;
    float demand;
    uint32_t sample_ms;
    bool initialized;
    bool settled;
    bool braking;
} BallController;

static BallController ball;
static uint32_t seen_observations, last_pulse_ms, last_valid_ms;
static int32_t target_steps, level_steps;
static int8_t motor_sign;
static bool startup_motion, active, holding;

static float clamp(float value, float min, float max)
{
    return fminf(fmaxf(value, min), max);
}

/* Position/speed balance, speed envelope and braking from F407bujin. */
static int32_t update_demand(int16_t pixel_error, uint32_t sample_ms)
{
    if (!ball.initialized) {
        ball.position = pixel_error;
        ball.previous_error = -ball.position;
        ball.sample_ms = sample_ms;
        ball.settled = fabsf(ball.position) <= 4.0f;
        ball.initialized = true;
        return (int32_t)ball.demand;
    }

    const float dt = clamp((sample_ms - ball.sample_ms) * 0.001f, 0.010f, 0.100f);
    ball.sample_ms = sample_ms;
    const float position_change = 0.45f * (pixel_error - ball.position);
    ball.position += position_change;
    const float measured_speed = clamp(position_change / dt, -500.0f, 500.0f);
    ball.velocity += 0.25f * (measured_speed - ball.velocity);

    const float error = -ball.position;
    const float distance = fabsf(error);
    const float speed = fabsf(ball.velocity);
    if (ball.settled) {
        ball.settled = distance < 8.0f && speed < 25.0f;
    } else {
        ball.settled = distance <= 4.0f && speed <= 10.0f;
    }

    const float direction = error > 0.0f ? 1.0f : (error < 0.0f ? -1.0f : 0.0f);
    const float acceleration = 220.0f;
    const float allowed = fminf(180.0f, sqrtf(2.0f * acceleration * fmaxf(0.0f, distance - 4.0f)));
    if (ball.reference * direction < 0.0f) {
        ball.reference = 0.0f;
    } else {
        ball.reference = direction * fminf(allowed, fabsf(ball.reference) + 300.0f * dt);
    }

    const float toward_target = ball.velocity * direction;
    const float stopping_distance = toward_target * toward_target / (2.0f * acceleration);
    ball.braking = toward_target > 0.0f && stopping_distance >= distance * (ball.braking ? 0.55f : 0.85f);
    if (error * ball.previous_error < 0.0f) { ball.integral *= 0.25f; }
    if (!ball.settled && distance < 25.0f && speed < 35.0f && !ball.braking) {
        ball.integral = clamp(ball.integral + 0.35f * error * dt, -6.0f, 6.0f);
    } else {
        ball.integral *= 0.92f;
    }

    const float velocity_gain = ball.braking ? 0.28f * 1.60f : 0.28f;
    float demand = 1.50f * (0.10f * error + velocity_gain * (ball.reference - ball.velocity) + ball.integral);
    if (ball.settled) {
        demand = ball.integral = ball.reference = 0.0f;
        ball.braking = false;
    }
    /* Clamp the physical target before slewing so the demand cannot build up past an end. */
    const float position_target = clamp(level_steps + motor_sign * demand, 0.0f, AXIS_MAX_STEPS);
    demand = motor_sign * (position_target - level_steps);
    ball.demand += clamp(demand - ball.demand, -MAX_COMMAND_CHANGE, MAX_COMMAND_CHANGE);
    ball.previous_error = error;
    return (int32_t)lroundf(ball.demand);
}

static bool vision_ready(const LinkTest *link, uint32_t now)
{
    return link->center_valid && now - link->center_rx_ms < VISION_TIMEOUT_MS;
}

int32_t center_balance_level_steps(void)
{
    return level_steps;
}

void center_balance_init(void)
{
    active = holding = false;
    startup_motion = true;
    motor_sign = 0;
    target_steps = AXIS_MAX_STEPS;
    level_steps = -1;
    last_pulse_ms = 0;
}

static int32_t bounded_target(int32_t position)
{
    if (position < 0) { return 0; }
    return position > AXIS_MAX_STEPS ? AXIS_MAX_STEPS : position;
}

void center_balance_key(const LinkTest *link, uint8_t key, uint32_t now)
{
    if (key == LINK_KEY_BACK || (key == LINK_KEY_OK && active)) {
        active = holding = false;
        startup_motion = false;
        target_steps = link->steps;
    } else if (startup_motion) {
        return;
    } else if (level_steps < 0 && !link->step_fault) {
        switch (key) {
        case LINK_KEY_UP: target_steps = bounded_target(target_steps + COARSE_JOG_STEPS); break;
        case LINK_KEY_DOWN: target_steps = bounded_target(target_steps - COARSE_JOG_STEPS); break;
        case LINK_JOG_FORWARD: target_steps = bounded_target(target_steps + 1); break;
        case LINK_JOG_BACK: target_steps = bounded_target(target_steps - 1); break;
        case LINK_KEY_OK:
            if (link->steps == target_steps && link->steps > 0 && link->steps < AXIS_MAX_STEPS) {
                level_steps = link->steps;
            }
            break;
        default: break;
        }
    } else if (!active && key == LINK_KEY_UP) {
        motor_sign = 1;
    } else if (!active && key == LINK_KEY_DOWN) {
        motor_sign = -1;
    } else if (key == LINK_KEY_OK && motor_sign && !link->step_fault && vision_ready(link, now)) {
        ball = (BallController){.demand = (float)((link->steps - level_steps) * motor_sign)};
        target_steps = link->steps;
        seen_observations = link->center_observations;
        last_pulse_ms = now;
        last_valid_ms = link->center_rx_ms;
        holding = false;
        active = true;
    }
}

void center_balance_tick(LinkTest *link, uint32_t now)
{
    if (link->step_fault || (!active && level_steps >= 0)) { return; }
    if (active) {
        const bool new_observation = seen_observations != link->center_observations && vision_ready(link, now);
        const bool timed_out = now - last_valid_ms >= VISION_TIMEOUT_MS || (ball.initialized &&
                               link->center_sample_ms - ball.sample_ms >= VISION_TIMEOUT_MS);
        if (!link->center_valid || timed_out) {
            if (!holding) {
                target_steps = link->steps;
                ball = (BallController){.demand = (float)((link->steps - level_steps) * motor_sign)};
                holding = true;
            }
        }
        if (new_observation) {
            seen_observations = link->center_observations;
            last_valid_ms = link->center_rx_ms;
            holding = false;
            const int32_t offset = motor_sign * update_demand(link->center_error_px, link->center_sample_ms);
            target_steps = bounded_target(level_steps + offset);
        }
        if (holding) { return; }
    }
    if (link->steps != target_steps && now - last_pulse_ms >= PULSE_INTERVAL_MS) {
        const bool forward = link->steps < target_steps;
        if (forward ? link->steps < AXIS_MAX_STEPS : link->steps > 0) {
            link_test_jog(link, forward);
        }
        last_pulse_ms = now;
        if (link->step_fault) {
            active = startup_motion = false;
        } else if (startup_motion && link->steps == target_steps) {
            if (target_steps == AXIS_MAX_STEPS) {
                target_steps = STARTUP_LEVEL_STEPS;
            } else {
                level_steps = STARTUP_LEVEL_STEPS;
                startup_motion = false;
                link->center_valid = false;
            }
        }
    }
}

void center_balance_menu(LinkTest *link, uint32_t now)
{
    const char *status = link->step_fault ? "BAL: STEP FAULT" : active ? (holding ? "BAL: HOLD" : "BAL: ACTIVE")
                         : startup_motion ? "BAL: STARTUP UP"
                         : level_steps < 0 ? "BAL: SET LEVEL"
                         : !motor_sign ? "BAL: SELECT DIR"
                         : vision_ready(link, now) ? "BAL: READY" : "BAL: WAIT BALL";
    if (startup_motion && target_steps == STARTUP_LEVEL_STEPS) {
        snprintf(link->startup_rows[0], 22, "BAL: RETURN %d", STARTUP_LEVEL_STEPS);
    } else {
        snprintf(link->startup_rows[0], 22, "%s", status);
    }
    if (startup_motion) {
        snprintf(link->startup_rows[1], 22, "BACK:STOP");
    } else if (level_steps < 0) {
        snprintf(link->startup_rows[1], 22, "UP/DN:25 JOG:1");
    } else if (active && holding && now - last_valid_ms >= VISION_TIMEOUT_MS) {
        snprintf(link->startup_rows[1], 22, "WAIT BALL AUTO");
    } else if (vision_ready(link, now)) {
        snprintf(link->startup_rows[1], 22, "DX:%+dpx DIR:%+d", link->center_error_px, motor_sign);
    } else {
        snprintf(link->startup_rows[1], 22, "NO BALL DIR:%+d", motor_sign);
    }
    snprintf(link->startup_rows[2], 22, "STEP:%ld/%d", (long)link->steps, AXIS_MAX_STEPS);
}
