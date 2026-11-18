#include "board.h"
#include "link_test.h"
#include "stm32f4xx.h"

int main(void)
{
    LinkTest link;
    uint8_t menu[LINK_MENU_SIZE];
    uint32_t last_menu_ms = 0;
    link_test_init(&link);
    board_init();

    for (;;) {
        uint8_t byte;
        while (board_uart_read(&byte)) {
            const uint8_t command = link_test_feed(&link, byte, board_millis());
            if (command == LINK_JOG_FORWARD || command == LINK_JOG_BACK) {
                link_test_jog(&link, command == LINK_JOG_FORWARD);
            } else if (command != 0u) {
                uint8_t packet[7];
                link_test_command(command, packet);
                board_uart_write(packet, sizeof(packet));
            }
        }
        const uint32_t now_ms = board_millis();
        if (link.menu_due || (uint32_t)(now_ms - last_menu_ms) >= 250u) {
            link_test_menu(&link, now_ms, board_uart_errors(), menu);
            board_uart_write(menu, sizeof(menu));
            link.menu_due = 0;
            last_menu_ms = now_ms;
        }
        __WFI();
    }
}
