#ifndef CENTER_BALANCE_H
#define CENTER_BALANCE_H

#include "link_test.h"
#include "board.h"

typedef enum {
    BALANCE_MODE_NONE,
    BALANCE_MODE_CENTER,
    BALANCE_MODE_THREE_POINTS,
    BALANCE_MODE_DISTURBANCE,
    BALANCE_MODE_RESERVED
} BalanceMode;

void center_balance_init(void);
void center_balance_key(const LinkTest *link, uint8_t key, uint32_t now);
void center_balance_button(const LinkTest *link, BoardButton button);
BalanceMode center_balance_selected_mode(void);
bool center_balance_mode_confirmed(void);
void center_balance_tick(LinkTest *link, uint32_t now);
void center_balance_menu(LinkTest *link, uint32_t now);
int32_t center_balance_level_steps(void);
int32_t center_balance_target_steps(void);
uint8_t center_balance_state(const LinkTest *link, uint32_t now);

#endif
