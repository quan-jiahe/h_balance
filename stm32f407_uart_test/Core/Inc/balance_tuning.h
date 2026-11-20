#ifndef BALANCE_TUNING_H
#define BALANCE_TUNING_H
#include <stdint.h>

/* Order is shared with tools/tune_balance.py. Defaults are in mm and seconds. */
typedef struct {
    uint32_t magic, version;
    float kp, kv, ki, gain, brake, max_speed, accel, ref_accel;
    float kff, kff2, max_offset;
} BalanceTuning;

extern volatile BalanceTuning g_balance_tuning;
#endif
