#include "board.h"
#include "link_test.h"
#include "startup_level.h"
#include "stm32f4xx.h"

int main(void)
{
    LinkTest link;
    uint8_t menu[LINK_MENU_SIZE];
    uint32_t last_menu_ms = 0;
    link_test_init(&link);
    board_init();
    startup_level_init();

    for (;;) {
        uint8_t byte;
        while (board_uart_read(&byte)) {
            link.key_event = 0;
            const uint8_t command = link_test_feed(&link, byte, board_millis());
            if (link.key_event) {
                startup_level_tick(&link, board_millis(), link.key_event);
            }
            if (command == LINK_START || command == LINK_STOP) {
                uint8_t packet[7];
                link_test_command(command, packet);
                board_uart_write(packet, sizeof(packet));
            }
        }
        const uint32_t now_ms = board_millis();
        startup_level_tick(&link, now_ms, 0);
        if (link.menu_due || (uint32_t)(now_ms - last_menu_ms) >= 250u) {
            startup_level_menu(&link);
            link_test_menu(&link, now_ms, board_uart_errors(), menu);
            board_uart_write(menu, sizeof(menu));
            link.menu_due = 0;
            last_menu_ms = now_ms;
        }
        __WFI();
    }
}
