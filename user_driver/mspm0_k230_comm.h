/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


/**
 * @file    mspm0_k230_comm.h
 * @brief   MSPM0 ↔ K230 串口通信协议
 *
 * K230 → MSPM0 帧格式 (18 字节固定长):
 *   ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 *   │  0   │  1   │  2   │  3   │ 4-5  │ 6-9  │10-11 │12-13 │  14  │
 *   ├──────┼──────┼──────┼──────┼──────┼──────┼──────┼──────┼──────┤
 *   │ 0xA5 │ 0x5A │ 0x01 │  12  │ seq  │ts_ms │ x_mm │vx_mm │ conf │
 *   └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 *   ┌──────┬──────────┐
 *   │  15  │  16-17   │
 *   ├──────┼──────────┤
 *   │status│ CRC16 LE │
 *   └──────┴──────────┘
 *
 *   所有多字节字段均为小端序 (Little-Endian).
 *   CRC16-CCITT: 多项式 0x1021, 初值 0xFFFF, 覆盖字节 2~15 (共 14 字节).
 */

#ifndef MSPM0_K230_COMM_H
#define MSPM0_K230_COMM_H

#include <stdbool.h>
#include <stdint.h>

/* ================================================================
 * 帧格式常量
 * ================================================================ */

#define K230_BALL_FRAME_SIZE     18u
#define K230_BALL_HEAD_0         0xA5u
#define K230_BALL_HEAD_1         0x5Au
#define K230_BALL_PROTO_VER      0x01u
#define K230_BALL_PAYLOAD_LEN    12u

/* K230 触控导航事件，7 字节命令帧。 */
#define K230_KEY_UP              0x40u
#define K230_KEY_DOWN            0x41u
#define K230_KEY_OK              0x42u
#define K230_KEY_BACK            0x43u
#define K230_MENU_SYNC           0x44u
#define K230_MENU_MESSAGE        0x50u
#define K230_MENU_ROWS           4u
#define K230_MENU_COLS           22u

/* 状态标志位 — 与 K230 端完全一致 */
#define K230_STATUS_RAW_VALID    0x01u   // 原始检测有效
#define K230_STATUS_TRACK_VALID  0x02u   // 跟踪有效
#define K230_STATUS_LOCKED       0x04u   // 已锁定
#define K230_STATUS_HELD         0x08u   // 保持中
#define K230_STATUS_PREDICTED    0x10u   // 预测值
#define K230_STATUS_LOST         0x20u   // 小球丢失

/* 复合状态: 数据可用 = 原始有效或跟踪有效 */
#define K230_DATA_VALID(status) \
    (((status) & (K230_STATUS_RAW_VALID | K230_STATUS_TRACK_VALID)) != 0u \
     && ((status) & K230_STATUS_LOST) == 0u)

/* ================================================================
 * 数据结构
 * ================================================================ */

/** 小球数据帧 (K230 → MSPM0) */
typedef struct {
    uint16_t sequence;
    uint32_t timestamp_ms;
    int16_t  x_mm;
    int16_t  vx_mm_s;
    uint8_t  confidence;
    uint8_t  status;
} K230_BallFrame;

typedef enum {
    K230_RX_WAIT_HEAD_0 = 0,
    K230_RX_WAIT_HEAD_1,
    K230_RX_READ_BODY,
} K230_RxState;

typedef struct {
    K230_RxState state;
    uint8_t buf[K230_BALL_FRAME_SIZE];
    uint8_t index;
    uint8_t command;  /* 非零表示已校验的触控事件，由 uart.c 取走。 */
} K230_FrameParser;

/* ================================================================
 * 调试计数器 (在 mspm0_k230_comm.c 中定义, ISR 中递增)
 * ================================================================ */
extern volatile uint32_t g_rx_byte_count;
extern volatile uint32_t g_header_count;
extern volatile uint32_t g_crc_fail_count;
extern volatile uint32_t g_valid_frame_count;

/* ================================================================
 * API
 * ================================================================ */

void K230_FrameParser_Init(K230_FrameParser *parser);

/**
 * @brief  接收 18 字节观测帧或 7 字节触控帧。
 * @return true=观测帧就绪；触控事件写入 parser->command，返回 false。
 */
bool K230_FrameParser_PushByte(K230_FrameParser *parser,
                               uint8_t byte,
                               K230_BallFrame *frame);

/** CRC16-CCITT (poly=0x1021, init=0xFFFF, 非反射) */
uint16_t K230_CRC16(const uint8_t *data, uint8_t len);

void K230_Comm_SendStartBalance(void);
void K230_Comm_SendStopBalance(void);
void K230_Comm_SendBytes(const uint8_t *bytes, uint16_t length);
void K230_Comm_SendMenu(const char lines[K230_MENU_ROWS][K230_MENU_COLS]);

#endif /* MSPM0_K230_COMM_H */
