#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void board_init(void);
uint32_t board_millis(void);
bool board_uart_read(uint8_t *byte);
void board_uart_write(const uint8_t *data, size_t length);
uint32_t board_uart_errors(void);

#endif
