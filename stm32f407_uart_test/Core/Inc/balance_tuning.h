#ifndef BALANCE_TUNING_H
#define BALANCE_TUNING_H
#include <stdint.h>

/* Order is shared with tools/tune_balance.py. ki=0 freezes, not erases, learned bias. */
typedef struct {
    uint32_t magic, version;
    float kp, kv, ki, gain, brake, max_speed, accel, ref_accel;
    float kff, kff2, max_offset;
} BalanceTuning;

extern volatile BalanceTuning g_balance_tuning;
#endif
