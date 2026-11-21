#include "vofa_tuning.h"
#include "vofa_uart.h"
#include "balance_tuning.h"
#include "center_balance.h"
#include <ctype.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

typedef struct { const char *name; size_t offset; float low, high; } Parameter;
#define PARAM(name, low, high) {#name, offsetof(BalanceTuning, name), low, high}
static const Parameter parameters[] = {
    PARAM(kp, 0, 3), PARAM(kv, 0, 4), PARAM(ki, 0, 3), PARAM(gain, 0, 3),
    PARAM(brake, 1, 4), PARAM(max_speed, 1, 150), PARAM(accel, 1, 300),
    PARAM(ref_accel, 1, 500), PARAM(kff, -3, 3), PARAM(kff2, -0.05f, 0.05f),
    PARAM(max_offset, 1, 300)
};
enum { PARAM_COUNT = sizeof(parameters) / sizeof(parameters[0]),
       CHANNELS = 7 + PARAM_COUNT, PERIOD_MS = 50, LINE_SIZE = 64 };
static char line[LINE_SIZE];
static unsigned used;
static bool discard_line;
static int command_status;
static uint32_t last_byte_ms, last_frame_ms, seen_errors;

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
    const float value = strtof(number, &end);
    if (end == number || !isfinite(value)) { return false; }
    while (isspace((unsigned char)*end)) { ++end; }
    if (*end) { return false; }
    for (unsigned i = 0; i < PARAM_COUNT; ++i) {
        const Parameter *p = &parameters[i];
        if (strcmp(name, p->name) == 0 && value >= p->low && value <= p->high) {
            *value_at(p) = value;
            return true;
        }
    }
    return false;
}

void vofa_tuning_init(void)
{
    used = 0;
    discard_line = false;
    command_status = 0;
    last_byte_ms = last_frame_ms = seen_errors = 0;
    vofa_uart_init();
}

void vofa_tuning_tick(const LinkTest *link, uint32_t now)
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
    if (now - last_frame_ms < PERIOD_MS) { return; }
    _Static_assert(sizeof(float) == 4, "JustFloat requires 32-bit floats");
    float channels[CHANNELS] = {
        link->center_error_mm, link->center_velocity_mm_s, link->steps,
        center_balance_target_steps(), center_balance_state(link, now),
        link->center_valid && now - link->center_rx_ms < 180u, command_status
    };
    for (unsigned i = 0; i < PARAM_COUNT; ++i) { channels[7 + i] = *value_at(&parameters[i]); }
    uint8_t packet[sizeof(channels) + 4];
    static const uint8_t tail[] = {0, 0, 0x80, 0x7F};
    memcpy(packet, channels, sizeof(channels));
    memcpy(packet + sizeof(channels), tail, sizeof(tail));
    if (vofa_uart_write(packet, sizeof(packet))) { last_frame_ms = now; }
}
