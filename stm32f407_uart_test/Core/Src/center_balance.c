#include "center_balance.h"
#include "balance_tuning.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"

enum {
    AXIS_MAX_STEPS = 650, STARTUP_LEVEL_STEPS = 310,
    COARSE_JOG_STEPS = 25, MAX_COMMAND_CHANGE = 12,
    PULSE_INTERVAL_MS = 4, VISION_TIMEOUT_MS = 180,
    STARTUP_TIMEOUT_MS = 20000, FIXED_MOTOR_SIGN = -1,
    /* DIR=-1 was measured with right-positive pixels; mm is left-positive. */
    MM_AXIS_SIGN = -1, MOTOR_MM_SIGN = FIXED_MOTOR_SIGN * MM_AXIS_SIGN
};

enum {
    M3_PUSH_MIN_MM = 25, M3_PUSH_MAX_MM = 35, M3_PIN_JITTER_MM = 1,
    M3_STILL_MM_S = 4, M3_PIN_MS = 800, M3_RECOVER_MM = 12,
    M3_HOLD_MM = 8, M3_RECOVER_MS = 6000, M3_END_MS = 8000,
    M3_EDGE_MM = 90, M3_EDGE_CLEAR_MM = 85, M3_BOUNDARY_MM = 100,
    M3_PREDICT_FROM_MM = 45, M3_RESPONSE_MS = 150
};

_Static_assert(sizeof(BalanceTuning) == 52, "SWD tuning layout changed");

/* 在 VSCode 修改这些默认值，再编译并烧录即可永久生效。 */
volatile BalanceTuning g_balance_tuning = {
    .magic = 0x42414C31u,
    .version = 1u,
    .kp = 0.5f,          // 位置增益：增大时回中更积极
    .kv = 0.70f,          // 速度阻尼：增大时抑制运动和过冲
    .ki = 0.875f,         // 姿态学习速率；设0暂停学习，保留已有补偿
    .gain = 1.50f,        // 动态纠偏及学习速率倍率，不缩放已学补偿
    .brake = 1.60f,       // 提前制动时的速度增益倍率
    .max_speed = 30.0f,   // 目标球速上限，mm/s
    .accel = 40.0f,       // 制动距离规划加速度，mm/s²
    .ref_accel = 120.0f,  // 目标速度上升斜率，mm/s²
    .kff = 0.0f,          // 线性速度补偿
    .kff2 = 0.0f,         // 平方速度补偿
    .max_offset = 150.0f, // 相对锁存姿态的最大纠偏步数
};

/* Integral is stored in output steps, so live gain changes preserve the learned pose. */
typedef struct {
    float reference, integral, demand, stalled_seconds, stable_seconds;
    uint32_t sample_ms;
    bool initialized, settled, braking;
} BallController;

static BallController ball;
static uint32_t seen_observations, last_pulse_ms, started_ms;
static int32_t target_steps, level_steps, anchor_steps;
static bool startup_motion, armed, active;
static BalanceMode selected_mode, confirmed_mode;
enum { MODE2_START_TOL_MM = 5, MODE2_HOLD_TOL_MM = 8,
       MODE2_LEG_LIMIT_MS = 7000, MODE2_HOLD_MS = 1500, MODE2_TOTAL_LIMIT_MS = 30000 };
typedef enum {
    THREE_WAIT_START, THREE_MOVE, THREE_HOLD, THREE_DONE, THREE_TIMEOUT, THREE_STOP
} ThreePointPhase;
static struct {
    ThreePointPhase phase;
    unsigned leg;
    uint32_t pressed_ms, leg_started_ms, hold_started_ms;
} sequence;
static const int16_t three_targets_mm[] = {-50, 50, 0};
static struct {
    enum { M3_CENTER, M3_READY, M3_HELD, M3_RECOVER, M3_PASS, M3_TIME, M3_DRIFT, M3_LOST, M3_OUT } phase;
    int16_t pin_position;
    uint32_t pin_since, last_pinned_ms;
    bool entered, edge, edge_output;
} mode3;
typedef enum { NO_FAULT, BOOT_TIMEOUT, STEP_PULSE } FaultReason;
static FaultReason fault_reason;

static float clamp(float value, float min, float max)
{
    return fminf(fmaxf(value, min), max);
}

static bool vision_ready(const LinkTest *link, uint32_t now)
{
    return link->center_valid && now - link->center_rx_ms < VISION_TIMEOUT_MS;
}

static bool three_points_running(void)
{
    return sequence.phase == THREE_MOVE || sequence.phase == THREE_HOLD;
}

static bool three_points_start_ready(const LinkTest *link, uint32_t now)
{
    return vision_ready(link, now) && abs(link->center_error_mm) <= MODE2_START_TOL_MM;
}

const char *center_balance_three_point_status(const LinkTest *link, uint32_t now)
{
    static const char *const moving[] = {"P1 MOVE", "P2 MOVE", "O MOVE"};
    static const char *const holding[] = {"P1 HOLD", "P2 HOLD", "O HOLD"};
    static const char *const lost[] = {"P1 LOST", "P2 LOST", "O LOST"};
    static const char *const timed_out[] = {"P1 TIMEOUT", "P2 TIMEOUT", "O TIMEOUT"};
    switch (sequence.phase) {
    case THREE_MOVE: return vision_ready(link, now) ? moving[sequence.leg] : lost[sequence.leg];
    case THREE_HOLD: return holding[sequence.leg];
    case THREE_DONE: return "O DONE";
    case THREE_TIMEOUT: return timed_out[sequence.leg];
    case THREE_STOP: return "STOP";
    default: return three_points_start_ready(link, now) ? "O PRESS START" : "PLACE AT O";
    }
}

static int32_t bounded_target(int32_t step)
{
    return step < 0 ? 0 : (step > AXIS_MAX_STEPS ? AXIS_MAX_STEPS : step);
}

static void hold_here(const LinkTest *link)
{
    target_steps = link->steps;
    ball.reference = ball.stalled_seconds = ball.stable_seconds = 0.0f;
    ball.demand = MOTOR_MM_SIGN * (link->steps - anchor_steps);
    ball.initialized = ball.settled = false;
    active = false;
}

static void start_here(const LinkTest *link)
{
    anchor_steps = link->steps; /* Starting pose, not a measured horizontal pose. */
    ball = (BallController){0};
    hold_here(link);
}

static void fail(const LinkTest *link, FaultReason reason)
{
    if (fault_reason != NO_FAULT) { return; }
    fault_reason = reason;
    hold_here(link);
}

/* Returns true when the target changed and the controller must wait for a new sample. */
static bool three_points_observe(const LinkTest *link, uint32_t now)
{
    const int error = three_targets_mm[sequence.leg] - link->center_error_mm;
    if (abs(error) > MODE2_HOLD_TOL_MM) {
        sequence.phase = THREE_MOVE;
    } else if (sequence.phase == THREE_MOVE) {
        sequence.phase = THREE_HOLD;
        sequence.hold_started_ms = now;
    } else if (sequence.phase == THREE_HOLD && now - sequence.hold_started_ms >= MODE2_HOLD_MS) {
        if (sequence.leg + 1 < sizeof(three_targets_mm) / sizeof(three_targets_mm[0])) {
            ++sequence.leg;
            sequence.leg_started_ms = now;
            sequence.phase = THREE_MOVE;
            hold_here(link);
            return true;
        }
        sequence.phase = THREE_DONE;
    }
    return false;
}

static bool three_points_within_time(const LinkTest *link, uint32_t now)
{
    if (now - sequence.pressed_ms <= MODE2_TOTAL_LIMIT_MS
        && (sequence.phase == THREE_HOLD || now - sequence.leg_started_ms <= MODE2_LEG_LIMIT_MS)) {
        return true;
    }
    sequence.phase = THREE_TIMEOUT;
    armed = false;
    hold_here(link);
    return false;
}

static bool starts_on_confirmation(BalanceMode mode)
{
    return mode == BALANCE_MODE_CENTER || mode == BALANCE_MODE_DISTURBANCE;
}

static void mode3_vision_gap(void)
{
    if (confirmed_mode != BALANCE_MODE_DISTURBANCE) { return; }
    if (mode3.phase == M3_HELD || mode3.phase == M3_RECOVER) { mode3.phase = M3_LOST; }
    mode3.pin_position = 0;
    mode3.edge = mode3.edge_output = false;
}

static void mode3_observe(const LinkTest *link)
{
    const float x = link->center_error_mm;
    const float distance = fabsf(x), speed = fabsf(link->center_velocity_mm_s);
    const float outward = x < 0.0f ? -link->center_velocity_mm_s : link->center_velocity_mm_s;
    const float projected = distance + outward * (M3_RESPONSE_MS * 0.001f)
        + outward * outward / (2.0f * g_balance_tuning.accel);
    const bool approaching = distance >= M3_PREDICT_FROM_MM && outward > 0.0f
        && (projected >= M3_BOUNDARY_MM || mode3.edge);
    mode3.edge = distance >= M3_EDGE_MM || approaching
        || (mode3.edge && distance >= M3_EDGE_CLEAR_MM);

    if (mode3.phase >= M3_PASS) { return; }
    if (distance > M3_BOUNDARY_MM) {
        mode3.phase = M3_OUT;
        return;
    }
    const uint32_t sample = link->center_sample_ms;
    const bool near_pin = mode3.pin_position != 0 && fabsf(x - mode3.pin_position) <= M3_PIN_JITTER_MM;
    if (mode3.phase == M3_READY) {
        if (!near_pin && (distance < M3_PUSH_MIN_MM || distance > M3_PUSH_MAX_MM)) {
            mode3.pin_position = 0;
        } else {
            if (!near_pin) {
                mode3.pin_position = link->center_error_mm;
                mode3.pin_since = sample;
            }
            if (speed <= M3_STILL_MM_S && sample - mode3.pin_since >= M3_PIN_MS) {
                mode3.last_pinned_ms = sample;
                mode3.phase = M3_HELD;
            }
        }
    } else if (mode3.phase == M3_HELD) {
        /* Estimate release from motion after a stationary plateau. The physical release is not sensed. */
        const float motion = x - mode3.pin_position;
        if (near_pin && speed <= M3_STILL_MM_S) {
            mode3.last_pinned_ms = sample;
        } else if (fabsf(motion) > M3_PIN_JITTER_MM && motion * link->center_velocity_mm_s > 0.0f) {
            mode3.phase = M3_RECOVER;
        }
    }
    if (mode3.phase == M3_RECOVER) {
        const uint32_t elapsed = sample - mode3.last_pinned_ms;
        if (elapsed <= M3_RECOVER_MS && distance <= M3_RECOVER_MM) { mode3.entered = true; }
        if (elapsed >= M3_RECOVER_MS) {
            if (!mode3.entered) { mode3.phase = M3_TIME; }
            else if (distance > M3_HOLD_MM) { mode3.phase = M3_DRIFT; }
            else if (elapsed >= M3_END_MS) { mode3.phase = M3_PASS; }
        }
    }
}

/* The K230 already estimates mm/s; do not differentiate/filter it a second time. */
static void update_target(const LinkTest *link, int16_t target_mm)
{
    const float position = link->center_error_mm;
    const float velocity = link->center_velocity_mm_s;
    const float error = target_mm - position;
    const float distance = fabsf(error), speed = fabsf(velocity);
    float dt = 0.033f;
    if (ball.initialized) {
        dt = clamp((link->center_sample_ms - ball.sample_ms) * 0.001f, 0.010f, 0.500f);
    }
    ball.sample_ms = link->center_sample_ms;
    ball.initialized = true;
    const float end0 = MOTOR_MM_SIGN * -anchor_steps;
    const float end1 = MOTOR_MM_SIGN * (AXIS_MAX_STEPS - anchor_steps);
    const float low = fmaxf(-g_balance_tuning.max_offset, fminf(end0, end1));
    const float high = fminf(g_balance_tuning.max_offset, fmaxf(end0, end1));
    const bool disturbance = confirmed_mode == BALANCE_MODE_DISTURBANCE;
    const float outward = position < 0.0f ? -velocity : velocity;
    if (disturbance && mode3.edge && outward >= -M3_STILL_MM_S) {
        /* Remove the per-frame target slew during braking. STEP timing and travel limits still apply. */
        ball.reference = ball.stalled_seconds = ball.stable_seconds = 0.0f;
        ball.settled = false;
        ball.braking = true;
        ball.demand = position > 0.0f ? low : high;
        mode3.edge_output = true;
        target_steps = bounded_target(anchor_steps + MOTOR_MM_SIGN * (int32_t)lroundf(ball.demand));
        return;
    }
    if (disturbance && mode3.edge_output) {
        /* Resume the normal slew from the pulses actually sent, discarding the pending brake target. */
        ball.demand = MOTOR_MM_SIGN * (link->steps - anchor_steps);
        mode3.edge_output = false;
    }
    const bool near_zero = ball.settled ? distance < 4.0f && speed < 10.0f
                                       : distance <= 2.0f && speed <= 4.0f;
    ball.stable_seconds = near_zero ? fminf(0.3f, ball.stable_seconds + dt) : 0.0f;
    if (!near_zero) { ball.settled = false; }
    if (!ball.settled && ball.stable_seconds >= 0.3f && link->steps == target_steps) {
        /* Transfer the complete held output into the origin without a target jump. */
        anchor_steps = target_steps;
        ball.reference = ball.integral = ball.demand = 0.0f;
        ball.stalled_seconds = 0.0f;
        ball.braking = false;
        ball.settled = true;
    }
    if (ball.settled) {
        if (disturbance && mode3.phase == M3_CENTER) { mode3.phase = M3_READY; }
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
    ball.stalled_seconds = distance >= 10.0f && toward <= 4.0f && !ball.braking
        ? fminf(0.3f, ball.stalled_seconds + dt) : 0.0f;
    const float gain = g_balance_tuning.gain;
    const float integral_limit = g_balance_tuning.max_offset;
    ball.integral = clamp(ball.integral, -integral_limit, integral_limit);
    const float previous_integral = ball.integral;
    /* Keep the learned level while the judge is moving or holding the ball. */
    const bool allow_learning = !disturbance || (!mode3.edge && mode3.phase != M3_READY && mode3.phase != M3_HELD);
    if (allow_learning && !ball.braking && ((distance < 10.0f && speed < 14.0f) || ball.stalled_seconds >= 0.3f)) {
        /* Bound the ramp rate independently of how far the ball is from zero. */
        ball.integral = clamp(ball.integral + gain * g_balance_tuning.ki * clamp(error, -10.0f, 10.0f) * dt,
            -integral_limit, integral_limit);
    }
    const float kv = g_balance_tuning.kv * (ball.braking ? g_balance_tuning.brake : 1.0f);
    const float base = gain * (g_balance_tuning.kp * error + kv * (ball.reference - velocity))
        + g_balance_tuning.kff * velocity + g_balance_tuning.kff2 * velocity * speed;
    float demand = base + ball.integral;
    const float integral_change = ball.integral - previous_integral;
    const float applied = MOTOR_MM_SIGN * (link->steps - anchor_steps);
    /* Do not accumulate further into either the command or mechanical limit. */
    if ((demand > high && integral_change > 0.0f)
            || (demand < low && integral_change < 0.0f)
            || (fabsf(ball.demand - applied) > MAX_COMMAND_CHANGE && (demand - applied) * integral_change > 0.0f)) {
        ball.integral = previous_integral;
        demand = base + ball.integral;
    }
    demand = clamp(demand, low, high);
    ball.demand += clamp(demand - ball.demand, -MAX_COMMAND_CHANGE, MAX_COMMAND_CHANGE);
    target_steps = bounded_target(anchor_steps + MOTOR_MM_SIGN * (int32_t)lroundf(ball.demand));
}

int32_t center_balance_level_steps(void) { return level_steps; }
int32_t center_balance_target_steps(void) { return target_steps; }

uint8_t center_balance_state(const LinkTest *link, uint32_t now)
{
    if (link->step_fault || fault_reason != NO_FAULT) { return 5; }
    if (startup_motion || level_steps < 0) { return 0; }
    if (!armed) { return 4; }
    if (active && vision_ready(link, now)) { return ball.settled ? 3 : 2; }
    return 1;
}

BalanceMode center_balance_selected_mode(void) { return selected_mode; }
bool center_balance_mode_confirmed(void) { return selected_mode == confirmed_mode; }

const char *center_balance_mode3_status(const LinkTest *link, uint32_t now)
{
    if (!armed) { return "STOP"; }
    if (mode3.phase == M3_OUT) { return "OUT"; }
    if (mode3.edge && vision_ready(link, now)) { return "EDGE BRAKE"; }
    switch (mode3.phase) {
    case M3_PASS: return "PASS";
    case M3_TIME: return "TIMEOUT";
    case M3_DRIFT: return "HOLD FAIL";
    case M3_LOST: return "VISION LOST";
    default: break;
    }
    if (!active || !vision_ready(link, now)) { return "WAIT BALL"; }
    if (mode3.phase == M3_READY) { return "PUSH BALL"; }
    if (mode3.phase == M3_HELD) { return "WAIT RELEASE"; }
    return mode3.phase == M3_RECOVER ? "RECOVER" : "CENTER";
}

void center_balance_init(void)
{
    ball = (BallController){0};
    anchor_steps = 0;
    active = false;
    fault_reason = NO_FAULT;
    selected_mode = BALANCE_MODE_CENTER;
    confirmed_mode = BALANCE_MODE_NONE;
    sequence.phase = THREE_WAIT_START;
    sequence.leg = 0;
    armed = false;
    startup_motion = true;
    target_steps = AXIS_MAX_STEPS;
    level_steps = -1;
    seen_observations = last_pulse_ms = 0;
    started_ms = board_millis();
}

void center_balance_key(const LinkTest *link, uint8_t key, uint32_t now)
{
    (void)now;
    if (key == LINK_KEY_BACK || (key == LINK_KEY_OK && armed && !startup_motion
                                && starts_on_confirmation(confirmed_mode))) {
        if (confirmed_mode == BALANCE_MODE_THREE_POINTS && armed) { sequence.phase = THREE_STOP; }
        armed = startup_motion = false;
        hold_here(link);
    } else if (startup_motion || fault_reason != NO_FAULT || link->step_fault) {
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
                start_here(link);
                seen_observations = link->center_observations;
            }
            break;
        default: break;
        }
    } else if (key == LINK_KEY_OK && confirmed_mode == BALANCE_MODE_CENTER) {
        hold_here(link);
        armed = true;
        seen_observations = link->center_observations;
    }
}

void center_balance_button(const LinkTest *link, BoardButton button, uint32_t pressed_ms)
{
    if (three_points_running()) { return; }
    if (button == BOARD_SW1) {
        selected_mode = selected_mode == BALANCE_MODE_RESERVED ? BALANCE_MODE_CENTER : selected_mode + 1;
    } else if (button == BOARD_SW2) {
        selected_mode = selected_mode == BALANCE_MODE_CENTER ? BALANCE_MODE_RESERVED : selected_mode - 1;
    } else if (button == BOARD_SW3 && !startup_motion && level_steps >= 0
               && fault_reason == NO_FAULT && !link->step_fault) {
        if (armed && selected_mode == confirmed_mode) { return; }
        hold_here(link);
        confirmed_mode = selected_mode;
        sequence.phase = THREE_WAIT_START;
        armed = starts_on_confirmation(confirmed_mode);
        mode3.phase = M3_CENTER;
        mode3.pin_position = 0;
        mode3.entered = mode3.edge = mode3.edge_output = false;
        seen_observations = link->center_observations;
    } else if (button == BOARD_SW4 && confirmed_mode == BALANCE_MODE_THREE_POINTS
               && selected_mode == confirmed_mode && !startup_motion && level_steps >= 0
               && fault_reason == NO_FAULT && !link->step_fault
               && three_points_start_ready(link, board_millis())) {
        hold_here(link);
        sequence.leg = 0;
        sequence.pressed_ms = sequence.leg_started_ms = pressed_ms;
        sequence.phase = THREE_MOVE;
        armed = true;
        seen_observations = link->center_observations;
    }
}

void center_balance_tick(LinkTest *link, uint32_t now)
{
    if (link->step_fault) { fail(link, STEP_PULSE); }
    if (fault_reason != NO_FAULT) { return; }
    if (startup_motion && now - started_ms >= STARTUP_TIMEOUT_MS) {
        fail(link, BOOT_TIMEOUT);
        return;
    }
    if (!startup_motion && level_steps >= 0) {
        if (!armed) { return; }
        const bool running = three_points_running();
        const bool ready = vision_ready(link, now);
        const bool sample_gap = ball.initialized
            && link->center_sample_ms - ball.sample_ms >= VISION_TIMEOUT_MS;
        if (running && (!ready || sample_gap)) { sequence.phase = THREE_MOVE; }
        if (running && !three_points_within_time(link, now)) { return; }
        if (!ready) {
            if (active) {
                mode3_vision_gap();
                hold_here(link);
            }
            return;
        }
        if (seen_observations != link->center_observations) {
            seen_observations = link->center_observations;
            if (!active || sample_gap) {
                mode3_vision_gap();
                hold_here(link);
            }
            if (running) {
                if (three_points_observe(link, now)) { return; }
                if (three_points_running() && !three_points_within_time(link, now)) { return; }
            }
            active = true;
            if (confirmed_mode == BALANCE_MODE_DISTURBANCE) { mode3_observe(link); }
            update_target(link, running ? three_targets_mm[sequence.leg] : 0);
        }
    }
    if (link->steps != target_steps && now - last_pulse_ms >= PULSE_INTERVAL_MS) {
        const bool forward = link->steps < target_steps;
        if (forward ? link->steps < AXIS_MAX_STEPS : link->steps > 0) {
            link_test_jog(link, forward);
        }
        last_pulse_ms = now;
        if (link->step_fault) { fail(link, STEP_PULSE); return; }
        if (startup_motion && link->steps == target_steps) {
            if (target_steps == AXIS_MAX_STEPS) {
                target_steps = STARTUP_LEVEL_STEPS;
            } else {
                level_steps = STARTUP_LEVEL_STEPS; /* Display/IMU reference only. */
                startup_motion = false;
                start_here(link);
                seen_observations = link->center_observations;
                link->center_valid = false;
            }
        }
    }
}

void center_balance_menu(LinkTest *link, uint32_t now)
{
    const uint8_t state = center_balance_state(link, now);
    const char *status = state == 5 ? "BAL: S5 FAULT"
        : startup_motion ? (target_steps == AXIS_MAX_STEPS ? "BAL: STARTUP UP" : "BAL: RETURN 310")
        : level_steps < 0 ? "BAL: SET LEVEL"
        : !confirmed_mode ? "BAL: CONFIRM MODE"
        : confirmed_mode == BALANCE_MODE_RESERVED ? "BAL: MODE RESERVED"
        : state == 4 ? "BAL: S4 STOPPED"
        : state == 2 || state == 3 ? (state == 3 ? "BAL: S3 HOLD ZERO" : "BAL: S2 ZERO")
        : "BAL: S1 WAIT BALL";
    if (confirmed_mode == BALANCE_MODE_DISTURBANCE && state != 0 && state != 5) {
        snprintf(link->startup_rows[0], 22, "BAL: M3 %s", center_balance_mode3_status(link, now));
    } else if (confirmed_mode == BALANCE_MODE_THREE_POINTS && state != 0 && state != 5) {
        snprintf(link->startup_rows[0], 22, "BAL: %s", center_balance_three_point_status(link, now));
    } else {
        snprintf(link->startup_rows[0], 22, "%s", status);
    }
    if (fault_reason == STEP_PULSE || link->step_fault) {
        snprintf(link->startup_rows[1], 22, "STEP PULSE FAIL");
    } else if (fault_reason == BOOT_TIMEOUT) {
        snprintf(link->startup_rows[1], 22, "STARTUP TIMEOUT");
    } else if (vision_ready(link, now) && !startup_motion) {
        snprintf(link->startup_rows[1], 22, "X:%+d V:%+d mm/s", link->center_error_mm, link->center_velocity_mm_s);
    } else {
        snprintf(link->startup_rows[1], 22, "BACK:STOP");
    }
    snprintf(link->startup_rows[2], 22, "STEP:%ld/%d", (long)link->steps, AXIS_MAX_STEPS);
}
