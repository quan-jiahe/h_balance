#include "vofa_tuning.h"
#include "vofa_uart.h"
#include "balance_tuning.h"
#include "center_balance.h"
#include "center_oled.h"
#include <ctype.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

typedef struct { const char *name; size_t offset; } Parameter;
#define PARAM(name) {#name, offsetof(BalanceTuning, name)}
static const Parameter parameters[] = {
    PARAM(kp), PARAM(kv), PARAM(ki), PARAM(gain), PARAM(brake), PARAM(max_speed),
    PARAM(accel), PARAM(ref_accel), PARAM(kff), PARAM(kff2), PARAM(max_offset), PARAM(brake_speed_gain)
};
enum { PARAM_COUNT = sizeof(parameters) / sizeof(parameters[0]),
       TRACE_START = 7 + PARAM_COUNT, PERIOD_MS = 50, MIN_TRACE_PERIOD_MS = 25, LINE_SIZE = 64 };
_Static_assert(VOFA_PACKET_SIZE <= 255, "VOFA frame must fit in the transmit ring");
static char line[LINE_SIZE];
static unsigned used;
static bool discard_line;
static int command_status;
static uint32_t last_byte_ms, last_frame_ms, seen_errors;
static uint32_t last_observations, dropped_frames;

static volatile float *value_at(const Parameter *p)
{
    return (volatile float *)((volatile uint8_t *)&g_balance_tuning + p->offset);
}

static bool apply_command(void)
{
    char *name = line;
    while (isspace((unsigned char)*name)) { ++name; }
    char *equals = strchr(name, '=');
    if (!equals) { return false; }
    char *end_name = equals;
    while (end_name > name && isspace((unsigned char)end_name[-1])) { --end_name; }
    *end_name = 0;
    for (char *c = name; *c; ++c) { *c = (char)tolower((unsigned char)*c); }
    char *end;
    const char *number = equals + 1;
    const double parsed = strtod(number, &end);
    if (end == number || !isfinite(parsed) || fabs(parsed) > FLT_MAX) { return false; }
    const float value = (float)parsed;
    while (isspace((unsigned char)*end)) { ++end; }
    if (*end) { return false; }
    for (unsigned i = 0; i < PARAM_COUNT; ++i) {
        const Parameter *p = &parameters[i];
        if (strcmp(name, p->name) != 0) { continue; }
        /* Braking divides by accel. Speed, ramp, and offset are magnitudes. */
        if ((value <= 0.0f && p->offset == offsetof(BalanceTuning, accel)) ||
            (value < 0.0f && (p->offset == offsetof(BalanceTuning, max_speed) ||
                              p->offset == offsetof(BalanceTuning, ref_accel) ||
                              p->offset == offsetof(BalanceTuning, max_offset) ||
                              p->offset == offsetof(BalanceTuning, brake_speed_gain)))) { return false; }
        *value_at(p) = value;
        return true;
    }
    return false;
}

void vofa_tuning_init(void)
{
    used = 0;
    discard_line = false;
    command_status = 0;
    last_byte_ms = last_frame_ms = seen_errors = last_observations = dropped_frames = 0;
    vofa_uart_init();
}

__attribute__((minsize)) void vofa_tuning_tick(const LinkTest *link, uint32_t now)
{
    const uint32_t errors = vofa_uart_errors();
    if (errors != seen_errors) {
        seen_errors = errors;
        used = 0;
        discard_line = true;
        last_byte_ms = now;
        command_status = -2;
    }
    if (used && now - last_byte_ms > 250u) {
        used = 0;
        discard_line = true;
        command_status = -1;
    }
    uint8_t byte;
    for (unsigned budget = 0; budget < 32 && vofa_uart_read(&byte); ++budget) {
        last_byte_ms = now;
        if (byte == '\n' || byte == '\r') {
            if (!discard_line && used) {
                line[used] = 0;
                command_status = apply_command() ? 1 : -1;
            }
            used = 0;
            discard_line = false;
        } else if (!discard_line) {
            if (byte < 32 || byte > 126 || used == LINE_SIZE - 1) {
                used = 0;
                discard_line = true;
                command_status = -1;
            } else {
                line[used++] = (char)byte;
            }
        }
    }
    const uint32_t elapsed = now - last_frame_ms;
    const bool new_observation = link->center_observations != last_observations;
    if (elapsed < PERIOD_MS && (!new_observation || elapsed < MIN_TRACE_PERIOD_MS)) { return; }
    last_frame_ms = now;
    last_observations = link->center_observations;

    const BalanceTrace *trace = center_balance_trace();
    float angle = 0.0f;
    const bool angle_valid = center_oled_relative_angle(now, &angle);
    float packet[] = {
        link->center_error_mm, link->center_velocity_mm_s, link->steps,
        center_balance_target_steps(), center_balance_state(link, now),
        link->center_valid && now - link->center_rx_ms < 180u, command_status,
        [TRACE_START] = now & 0xFFFFFFu,
        trace->mode, trace->target_mm, trace->target_mm - link->center_error_mm,
        trace->reference_speed, trace->reference_accel, trace->approach_speed,
        trace->braking_distance, trace->braking_threshold, trace->still_ms, trace->integral_multiplier,
        trace->position_steps, trace->speed_steps, trace->brake_steps, trace->integral_steps, trace->feedforward_steps,
        trace->demand_steps, trace->limited_steps, trace->hold_filter_steps, trace->anchor_steps, trace->flags,
        link->center_observations & 0xFFFFFFu, link->center_sample_ms & 0xFFFFFFu,
        link->center_observations ? (float)(now - link->center_rx_ms) : -1.0f,
        angle, angle_valid, link->step_pulses & 0xFFFFFFu, link->step_reversals & 0xFFFFFFu,
        link->last_step_direction, dropped_frames & 0xFFFFFFu, trace->phase,
        link->center_confidence, link->last_status, link->crc_errors & 0xFFFFFFu,
        0.0f
    };
    _Static_assert(sizeof(packet) == VOFA_PACKET_SIZE, "JustFloat channel layout");
    for (unsigned i = 0; i < PARAM_COUNT; ++i) { packet[7 + i] = *value_at(&parameters[i]); }
    for (unsigned i = TRACE_START; i < VOFA_CHANNELS; ++i) {
        // Infinity is the JustFloat delimiter, so sanitize using its wire representation.
        uint32_t bits;
        memcpy(&bits, &packet[i], sizeof(bits));
        if ((bits & 0x7F800000u) == 0x7F800000u) {
            packet[i] = (bits >> 31) ? -1000000.0f : 1000000.0f;
        }
    }
    static const uint8_t tail[] = {0, 0, 0x80, 0x7F};
    memcpy(packet + VOFA_CHANNELS, tail, sizeof(tail));
    if (!vofa_uart_write((const uint8_t *)packet, sizeof(packet))) { ++dropped_frames; }
}
