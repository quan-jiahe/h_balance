#ifndef STARTUP_LEVEL_H
#define STARTUP_LEVEL_H

#include "link_test.h"

/* Call once after board_init(), then once per loop before publishing the menu. */
void startup_level_init(void);
void startup_level_tick(LinkTest *link, uint32_t now_ms, uint8_t key);
void startup_level_menu(LinkTest *link);

#endif
