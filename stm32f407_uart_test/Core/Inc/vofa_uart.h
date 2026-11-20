#ifndef VOFA_UART_H
#define VOFA_UART_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* USART1 PA9 TX / PA10 RX, 115200 8N1. Separate from K230 USART3. */
void vofa_uart_init(void);
bool vofa_uart_read(uint8_t *byte);
bool vofa_uart_write(const uint8_t *data, size_t size);
uint32_t vofa_uart_errors(void);
#endif
