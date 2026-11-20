#ifndef VOFA_TUNING_H
#define VOFA_TUNING_H
#include "link_test.h"

/* JustFloat telemetry, ASCII name=value\n commands. No remote motor-start command. */
void vofa_tuning_init(void);
void vofa_tuning_tick(const LinkTest *link, uint32_t now);
#endif
