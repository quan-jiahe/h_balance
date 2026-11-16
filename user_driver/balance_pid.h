/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


/**
 * @file    balance_pid.h
 * @brief   小球平衡控制系统 — 位置环 PID
 *
 * 架构:
 *   K230 串口 18B帧 → x_mm + vx_mm_s
 *       ↓
 *   位置式 PID (balance_pid)
 *       ↓
 *   目标角度 → 步进电机闭环
 *
 * 调参核心参数 (在 balance_pid.c 中修改):
 *   BALANCE_KP, BALANCE_KI, BALANCE_KD     — PID 系数
 *   BALANCE_DIRECTION_SIGN                  — ±1, 输出方向
 *   BALANCE_NEUTRAL_ANGLE_DEG               — 基准角度
 *   BALANCE_MAX_ANGLE_DEG                   — 最大倾斜角度
 */

#ifndef BALANCE_PID_H
#define BALANCE_PID_H

#include <stdint.h>

/* 控制周期 (ms), 与 K230 数据发送间隔 (≤20ms) 匹配 */
#define CL_CTRL_PERIOD_MS  20U  /* 匹配 K230 50fps 帧间隔 */

/* ================================================================
 * 全局变量 (由 uart.c ISR 写入)
 * ================================================================ */
extern volatile float   g_ball_error_mm;       // 位置偏差 (mm)
extern volatile float   g_ball_velocity_mm_s;  // 钢球速度 (mm/s)
extern volatile uint8_t g_ball_data_valid;     // 1=新数据就绪
extern volatile uint8_t g_ball_confidence;     // 置信度 0~100
extern volatile uint8_t g_ball_status;         // 状态标志位

/* ================================================================
 * 公开 API
 * ================================================================ */

/** 初始化 */
void Balance_Init(void);

/** 执行一次控制循环 (每 20ms 由 g_cl_trigger 驱动) */
void Balance_Control(void);

/** 启动/停止 */
void Balance_Start(void);
void Balance_StartAutoSeq(void);    // 启动自动序列: +50mm(5s) → -50mm
void Balance_Stop(void);
int  Balance_IsActive(void);

/** 位置环目标值 (mm, 菜单 Set Target 调整) */
extern float g_ball_target_mm;
void  Balance_SetTarget(float mm);
float Balance_GetBallTarget(void);

/** 平衡 PID 参数 (菜单可调) */
extern float g_balance_kp;
extern float g_balance_ki;
extern float g_balance_kd;
extern float g_balance_kff;
extern float g_balance_kff2;

/** 自动序列参数 (菜单可调) */
extern uint32_t g_auto_seq_phase0_ramp_ms;  // 阶段0 斜坡时长
extern uint32_t g_auto_seq_phase1_ramp_ms;  // 阶段1 斜坡时长

/** 查询当前控制阶段 (供 OLED 显示) */
int         Balance_IsPidRunning(void);   // PID 闭环运行中
int         Balance_IsAutoSeq(void);      // 自动序列运行中
const char *Balance_GetStateStr(void);
const char *Balance_GetAutoSeqPhaseStr(void);  // 自动序列阶段描述

/** 请求回到 0° (退出平衡/上电时调用) */
void Balance_RequestGoHome(void);

/** 执行一次归零角度闭环 (主循环控制周期调用), 返回 1=进行中 0=已完成 */
int  Balance_GoHome(void);

/** 查询是否正在归零 */
int  Balance_IsGoingHome(void);

/** 获取当前状态 (供 OLED 显示) */
float Balance_GetError(void);           // 最近位置偏差 (mm)
float Balance_GetVelocity(void);        // 最近速度 (mm/s)
float Balance_GetTargetAngle(void);     // PID 目标角度 (deg)
float Balance_GetCurrentAngle(void);    // 编码器当前角度 (deg)
uint8_t Balance_GetConfidence(void);    // 置信度

#endif /* BALANCE_PID_H */
