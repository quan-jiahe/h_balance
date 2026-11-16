/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


/**
 * @file    uart.h
 * @brief   K230 UART 通信接口
 */

#ifndef UART_H
#define UART_H

#include "ti_msp_dl_config.h"

void UART_send_string(UART_Regs *uart, const char *str);
void UART_send_char(UART_Regs *uart, const uint8_t chr);

#endif /* UART_H */
