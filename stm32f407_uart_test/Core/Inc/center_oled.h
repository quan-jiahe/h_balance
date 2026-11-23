#ifndef CENTER_OLED_H
#define CENTER_OLED_H

#include "link_test.h"

void center_oled_init(void);
void center_oled_tick(const LinkTest *link, uint32_t now);
bool center_oled_relative_angle(uint32_t now, float *degrees);

#endif
