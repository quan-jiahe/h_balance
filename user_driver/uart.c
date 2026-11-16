/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


/**
 * @file    uart.c
 * @brief   K230 UART 中断服务 — 观测帧接收与触控事件入队
 */

#include "uart.h"
#include "mspm0_k230_comm.h"
#include "key.h"

/* ================================================================
 * 全局实例 (main.c)
 * ================================================================ */
extern K230_FrameParser g_k230_parser;

/* ================================================================
 * 全局数据 (balance_pid.c)
 * ================================================================ */
extern volatile float   g_ball_error_mm;
extern volatile float   g_ball_velocity_mm_s;
extern volatile uint8_t g_ball_data_valid;
extern volatile uint8_t g_ball_confidence;
extern volatile uint8_t g_ball_status;

/* ================================================================
 * 阻塞式发送 (调试用)
 * ================================================================ */
void UART_send_char(UART_Regs *uart, const uint8_t chr)
{
    DL_UART_transmitDataBlocking(uart, chr);
}

void UART_send_string(UART_Regs *uart, const char *str)
{
    while (*str) {
        UART_send_char(uart, (uint8_t)*str);
        str++;
    }
}

/* ================================================================
 * K230 UART3 接收中断
 * ================================================================ */
void K230_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(K230_UART_INST)) {
    case DL_UART_MAIN_IIDX_RX: {
        uint8_t byte_val = DL_UART_Main_receiveData(K230_UART_INST);
        g_rx_byte_count++;
        K230_BallFrame frame;

        if (K230_FrameParser_PushByte(&g_k230_parser, byte_val, &frame)) {
            /* CRC 通过 — 检查数据是否有效 (非 LOST + 有 RAW_VALID 或 TRACK_VALID) */
            if (K230_DATA_VALID(frame.status)) {
                g_ball_error_mm     = (float)frame.x_mm;
                g_ball_velocity_mm_s = (float)frame.vx_mm_s;
                g_ball_confidence   = frame.confidence;
                g_ball_status       = frame.status;
                g_ball_data_valid   = 1;
            }
        }
        if (g_k230_parser.command != 0u) {
            Menu_QueueKey(g_k230_parser.command);
            g_k230_parser.command = 0u;
        }
        break;
    }
    default:
        break;
    }
}
