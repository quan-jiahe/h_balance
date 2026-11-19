#include "pulse_cycle.h"
#include <stdio.h>

enum {
    CYCLE_MAX_STEPS = 650,
    SLOW_INTERVAL_MS = 12,
    MIN_CRUISE_INTERVAL_MS = 4,
    CRUISE_INTERVAL_COUNT = 5,
    STEPS_PER_SPEED_CHANGE = 16
};

static bool forward;
static bool stopped;
static uint32_t last_pulse_ms;
static uint32_t speed_state;
static int32_t cruise_interval_ms;

void pulse_cycle_init(uint32_t now)
{
    forward = true;
    stopped = false;
    last_pulse_ms = now;
    speed_state = 0x7E21B4A5u;
    cruise_interval_ms = MIN_CRUISE_INTERVAL_MS;
}

void pulse_cycle_tick(LinkTest *link, uint32_t now, uint8_t key)
{
    if (key == LINK_KEY_BACK) { stopped = true; }
    if (stopped || link->step_fault) { return; }

    const uint32_t elapsed = now - last_pulse_ms;
    const int32_t to_upper = CYCLE_MAX_STEPS - link->steps;
    const int32_t edge_distance = link->steps < to_upper ? link->steps : to_upper;
    const int32_t interval = SLOW_INTERVAL_MS - edge_distance / STEPS_PER_SPEED_CHANGE;
    const uint32_t pulse_interval = (uint32_t)(interval > cruise_interval_ms
                                              ? interval : cruise_interval_ms);
    if (elapsed < pulse_interval) { return; }

    link_test_jog(link, forward);
    last_pulse_ms = now;
    if (link->steps == (forward ? CYCLE_MAX_STEPS : 0)) {
        forward = !forward;
        /* Select a different cruise speed for each leg while keeping the edge ramp. */
        speed_state ^= speed_state << 13;
        speed_state ^= speed_state >> 17;
        speed_state ^= speed_state << 5;
        const uint32_t slot = (uint32_t)(cruise_interval_ms - MIN_CRUISE_INTERVAL_MS);
        const uint32_t change = 1u + speed_state % (CRUISE_INTERVAL_COUNT - 1u);
        cruise_interval_ms = MIN_CRUISE_INTERVAL_MS
                             + (int32_t)((slot + change) % CRUISE_INTERVAL_COUNT);
    }
}

void pulse_cycle_menu(LinkTest *link)
{
    const char *state = link->step_fault ? "MOTOR:ERR" : stopped ? "MOTOR:STOP"
                        : forward ? "MOTOR:FWD" : "MOTOR:REV";
    snprintf(link->startup_rows[0], 22, "%s", state);
    snprintf(link->startup_rows[1], 22, "%s", link->step_fault ? "STEP PULSE" : "RANGE:0-650");
    snprintf(link->startup_rows[2], 22, "STEP:%+ld", (long)link->steps);
}
