#ifndef CENTER_BALANCE_H
#define CENTER_BALANCE_H

#include "link_test.h"

void center_balance_init(void);
void center_balance_key(const LinkTest *link, uint8_t key, uint32_t now);
void center_balance_tick(LinkTest *link, uint32_t now);
void center_balance_menu(LinkTest *link, uint32_t now);
int32_t center_balance_level_steps(void);

#endif
