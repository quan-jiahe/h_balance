#include "board.h"
#include "stm32f4xx.h"

int main(void)
{
    board_init();
    for (;;) {
        board_buttons_poll(board_millis());

        const bool sw4 = board_button_is_pressed(BOARD_SW4);
        const bool sw5 = board_button_is_pressed(BOARD_SW5);
        board_led_set(BOARD_LED3, board_button_is_pressed(BOARD_SW1) || sw4);
        board_led_set(BOARD_LED4, board_button_is_pressed(BOARD_SW2) || sw4 || sw5);
        board_led_set(BOARD_LED5, board_button_is_pressed(BOARD_SW3) || sw5);
        __WFI();
    }
}
