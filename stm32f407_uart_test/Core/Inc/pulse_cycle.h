#ifndef PULSE_CYCLE_H
#define PULSE_CYCLE_H

#include "link_test.h"

void pulse_cycle_init(uint32_t now);
void pulse_cycle_tick(LinkTest *link, uint32_t now, uint8_t key);
void pulse_cycle_menu(LinkTest *link);

#endif
