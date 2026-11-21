#include "board.h"
#include "link_test.h"
#if defined(F407_CENTER_BALANCE) || defined(F407_OLED_TEST)
#include "center_oled.h"
#endif
#ifdef F407_CENTER_BALANCE
#include "center_balance.h"
#elif !defined(F407_OLED_TEST)
#include "pulse_cycle.h"
#endif
#include "stm32f4xx.h"

int main(void)
{
#ifdef F407_OLED_TEST
    board_init();
    center_oled_init();
    for (;;) {
        center_oled_tick(NULL, board_millis());
        __WFI();
    }
#else
    LinkTest link;
    uint8_t menu[LINK_MENU_SIZE];
    uint32_t last_menu_ms = 0;
    link_test_init(&link);
    board_init();
#ifdef F407_CENTER_BALANCE
    center_oled_init();
    center_balance_init();
#else
    pulse_cycle_init(board_millis());
#endif

    for (;;) {
        uint8_t byte;
        while (board_uart_read(&byte)) {
            link.key_event = 0;
#ifdef F407_CENTER_BALANCE
            link_test_feed(&link, byte, board_millis());
            if (link.key_event) {
                center_balance_key(&link, link.key_event, board_millis());
            }
#else
            const uint8_t command = link_test_feed(&link, byte, board_millis());
            if (link.key_event) {
                pulse_cycle_tick(&link, board_millis(), link.key_event);
            }
            if (command == LINK_START || command == LINK_STOP) {
                uint8_t packet[7];
                link_test_command(command, packet);
                board_uart_write(packet, sizeof(packet));
            }
#endif
        }
        const uint32_t now_ms = board_millis();
#ifdef F407_CENTER_BALANCE
        center_balance_tick(&link, now_ms);
#else
        pulse_cycle_tick(&link, now_ms, 0);
#endif
        if (link.menu_due || (uint32_t)(now_ms - last_menu_ms) >= 250u) {
#ifdef F407_CENTER_BALANCE
            center_balance_menu(&link, now_ms);
#else
            pulse_cycle_menu(&link);
#endif
            link_test_menu(&link, now_ms, board_uart_errors(), menu);
            board_uart_write(menu, sizeof(menu));
            link.menu_due = 0;
            last_menu_ms = now_ms;
        }
#ifdef F407_CENTER_BALANCE
        center_oled_tick(&link, board_millis());
#endif
        __WFI();
    }
#endif
}
