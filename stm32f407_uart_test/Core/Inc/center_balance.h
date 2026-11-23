#ifndef CENTER_BALANCE_H
#define CENTER_BALANCE_H

#include "link_test.h"
#include "board.h"

typedef enum {
    BALANCE_MODE_NONE,
    BALANCE_MODE_CENTER,
    BALANCE_MODE_THREE_POINTS,
    BALANCE_MODE_DISTURBANCE,
    BALANCE_MODE_RESERVED,
    BALANCE_MODE_SINE
} BalanceMode;

enum {
    CONTROL_TRACE_BRAKING = 1u, CONTROL_TRACE_INTEGRAL_BLOCKED = 2u,
    CONTROL_TRACE_AT_LIMIT = 4u, CONTROL_TRACE_SLEW_LIMIT = 8u,
    CONTROL_TRACE_HOLD_FILTER = 16u, CONTROL_TRACE_EDGE_BRAKE = 32u,
    CONTROL_TRACE_EVALUATED = 64u
};

typedef struct {
    float target_mm, reference_speed, reference_accel;
    float approach_speed, braking_distance, braking_threshold;
    float still_ms, integral_multiplier;
    float position_steps, speed_steps, brake_steps, integral_steps, feedforward_steps;
    float demand_steps, limited_steps, hold_filter_steps;
    int32_t anchor_steps;
    BalanceMode mode;
    unsigned phase, flags;
} BalanceTrace;

const BalanceTrace *center_balance_trace(void);
void center_balance_init(void);
void center_balance_key(const LinkTest *link, uint8_t key, uint32_t now);
void center_balance_button(const LinkTest *link, BoardButton button);
void center_balance_start_sine(const LinkTest *link, uint32_t pressed_ms, uint32_t now);
BalanceMode center_balance_selected_mode(void);
bool center_balance_mode_confirmed(void);
const char *center_balance_three_point_status(const LinkTest *link, uint32_t now);
const char *center_balance_mode3_status(const LinkTest *link, uint32_t now);
const char *center_balance_sine_status(const LinkTest *link, uint32_t now);
float center_balance_sine_reference_mm(uint32_t now);
float center_balance_sine_peak_error_mm(void);
float center_balance_sine_rms_error_mm(void);
void center_balance_tick(LinkTest *link, uint32_t now);
void center_balance_menu(LinkTest *link, uint32_t now);
int32_t center_balance_level_steps(void);
int32_t center_balance_target_steps(void);
uint8_t center_balance_state(const LinkTest *link, uint32_t now);

#endif
