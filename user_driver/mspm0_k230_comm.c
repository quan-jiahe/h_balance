/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


/**
 * @file    mspm0_k230_comm.c
 * @brief   K230 通信协议实现: CRC16-CCITT, 帧解析, 发送
 *
 * CRC16-CCITT (与 K230 端 k230_ball_distance_uart_m0.py 完全一致):
 *   多项式:  0x1021
 *   初值:    0xFFFF
 *   输入:    非反射 (MSB first)
 *   输出:    非反射
 *   覆盖:    字节 2~15 (version + length + 12 payload = 14 bytes)
 */

#include "ti_msp_dl_config.h"
#include "mspm0_k230_comm.h"

#ifndef K230_UART_INST
#define K230_UART_INST UART3
#endif

/* ================================================================
 * 调试计数器
 * ================================================================ */
volatile uint32_t g_rx_byte_count      = 0;
volatile uint32_t g_header_count       = 0;
volatile uint32_t g_crc_fail_count     = 0;
volatile uint32_t g_valid_frame_count  = 0;

/* ================================================================
 * CRC16-CCITT
 *   Polynomial: 0x1021
 *   Initial:    0xFFFF
 *   Algorithm:  MSB-first, non-reflected
 *   Coverage:   bytes 2 ~ 15 (14 bytes), 与 K230 一致
 * ================================================================ */

uint16_t K230_CRC16(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint8_t i = 0u; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8u);
        for (uint8_t j = 0u; j < 8u; j++) {
            if (crc & 0x8000u) {
                crc = (crc << 1u) ^ 0x1021u;
            } else {
                crc <<= 1u;
            }
        }
    }
    return crc;
}

/* ================================================================
 * 帧解析器 (18 字节观测帧 / 7 字节触控帧)
 * ================================================================ */

void K230_FrameParser_Init(K230_FrameParser *parser)
{
    if (parser == NULL) return;
    parser->state = K230_RX_WAIT_HEAD_0;
    parser->index = 0u;
    parser->command = 0u;
}

bool K230_FrameParser_PushByte(K230_FrameParser *parser,
                               uint8_t byte,
                               K230_BallFrame *frame)
{
    if ((parser == NULL) || (frame == NULL)) return false;

    switch (parser->state) {

    case K230_RX_WAIT_HEAD_0:
        if (byte == K230_BALL_HEAD_0) {
            parser->buf[0] = byte;
            parser->index  = 1u;
            parser->state  = K230_RX_WAIT_HEAD_1;
        }
        break;

    case K230_RX_WAIT_HEAD_1:
        if (byte == K230_BALL_HEAD_1) {
            parser->buf[1] = byte;
            parser->index  = 2u;
            parser->state  = K230_RX_READ_BODY;
            g_header_count++;
        } else if (byte == K230_BALL_HEAD_0) {
            parser->buf[0] = byte;
            parser->index  = 1u;
        } else {
            K230_FrameParser_Init(parser);
        }
        break;

    case K230_RX_READ_BODY:
        parser->buf[parser->index] = byte;
        parser->index++;

        if (parser->index == 4u &&
            (parser->buf[2] != K230_BALL_PROTO_VER ||
             (parser->buf[3] != K230_BALL_PAYLOAD_LEN && parser->buf[3] != 1u))) {
            K230_FrameParser_Init(parser);
            return false;
        }

        if (parser->index >= 4u && parser->index == 6u + parser->buf[3]) {
            uint8_t crc_index = (uint8_t)(4u + parser->buf[3]);
            parser->state = K230_RX_WAIT_HEAD_0;
            parser->index = 0u;

            uint16_t calc_crc = K230_CRC16(&parser->buf[2], (uint8_t)(crc_index - 2u));
            uint16_t recv_crc = (uint16_t)parser->buf[crc_index]
                              | ((uint16_t)parser->buf[crc_index + 1u] << 8u);

            if (calc_crc != recv_crc) {
                g_crc_fail_count++;
                return false;
            }

            if (parser->buf[3] == 1u) {
                uint8_t command = parser->buf[4];
                if (command >= K230_KEY_UP && command <= K230_MENU_SYNC)
                    parser->command = command;
                return false;
            }

            /* ---- 解析 Payload ---- */
            frame->sequence = (uint16_t)parser->buf[4]
                            | ((uint16_t)parser->buf[5] << 8u);
            frame->timestamp_ms = (uint32_t)parser->buf[6]
                                | ((uint32_t)parser->buf[7] << 8u)
                                | ((uint32_t)parser->buf[8] << 16u)
                                | ((uint32_t)parser->buf[9] << 24u);
            frame->x_mm = (int16_t)(
                (uint16_t)parser->buf[10] |
                ((uint16_t)parser->buf[11] << 8u));
            frame->vx_mm_s = (int16_t)(
                (uint16_t)parser->buf[12] |
                ((uint16_t)parser->buf[13] << 8u));
            frame->confidence = parser->buf[14];
            frame->status     = parser->buf[15];

            g_valid_frame_count++;
            return true;
        }
        break;

    default:
        K230_FrameParser_Init(parser);
        break;
    }

    return false;
}

/* ================================================================
 * 发送函数 (同样使用 CRC16-CCITT, 覆盖字节 2~4)
 *   帧: 0xA5 0x5A VER LEN CMD CRC16_LE = 7 bytes
 * ================================================================ */

#define K230_CMD_START_BALANCE  0x30u
#define K230_CMD_STOP_BALANCE   0x31u

void K230_Comm_SendBytes(const uint8_t *bytes, uint16_t length)
{
    if (bytes == NULL) return;
    for (uint16_t i = 0u; i < length; i++) {
        DL_UART_Main_transmitDataBlocking(K230_UART_INST, bytes[i]);
    }
}

static void k230_send_cmd(uint8_t cmd)
{
    uint8_t frame[7];
    frame[0] = K230_BALL_HEAD_0;
    frame[1] = K230_BALL_HEAD_1;
    frame[2] = K230_BALL_PROTO_VER;
    frame[3] = 1u;                    // payload length = 1
    frame[4] = cmd;
    /* CRC 覆盖字节 2~4 (version + length + cmd) */
    uint16_t crc = K230_CRC16(&frame[2], 3u);
    frame[5] = (uint8_t)(crc & 0xFFu);
    frame[6] = (uint8_t)((crc >> 8u) & 0xFFu);
    K230_Comm_SendBytes(frame, 7u);
}

void K230_Comm_SendStartBalance(void)
{
    k230_send_cmd(K230_CMD_START_BALANCE);
}

void K230_Comm_SendStopBalance(void)
{
    k230_send_cmd(K230_CMD_STOP_BALANCE);
}

void K230_Comm_SendMenu(const char lines[K230_MENU_ROWS][K230_MENU_COLS])
{
    uint8_t frame[7u + K230_MENU_ROWS * K230_MENU_COLS];
    frame[0] = K230_BALL_HEAD_0;
    frame[1] = K230_BALL_HEAD_1;
    frame[2] = K230_BALL_PROTO_VER;
    frame[3] = 1u + K230_MENU_ROWS * K230_MENU_COLS;
    frame[4] = K230_MENU_MESSAGE;
    for (uint8_t row = 0u; row < K230_MENU_ROWS; row++) {
        for (uint8_t col = 0u; col < K230_MENU_COLS; col++) {
            frame[5u + row * K230_MENU_COLS + col] = (uint8_t)lines[row][col];
        }
    }
    uint16_t crc = K230_CRC16(&frame[2], (uint8_t)(sizeof(frame) - 4u));
    frame[sizeof(frame) - 2u] = (uint8_t)crc;
    frame[sizeof(frame) - 1u] = (uint8_t)(crc >> 8u);
    K230_Comm_SendBytes(frame, sizeof(frame));
}
