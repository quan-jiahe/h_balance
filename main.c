/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "init.h"
#include "key.h"
#include "motor.h"
#include "encoder.h"
#include "mspm0_k230_comm.h"
#include "balance_pid.h"

// ========== K230 帧解析器全局实例 ==========
K230_FrameParser g_k230_parser;

// 系统时间戳
volatile uint32_t sys_tick_ms = 0;
static uint32_t encoder_tick_acc  = 0;
static uint32_t control_tick_acc  = 0;
volatile uint8_t g_cl_trigger = 0;

void SysTick_Handler(void)
{
    sys_tick_ms++;
    encoder_tick_acc++;
    control_tick_acc++;
    if (encoder_tick_acc >= 5) {
        encoder_tick_acc = 0;
        Encoder_Tick(5);
    }
    if (control_tick_acc >= CL_CTRL_PERIOD_MS) {
        control_tick_acc = 0;
        g_cl_trigger = 1;
    }
}

int main(void)
{
    System_Init();

    K230_FrameParser_Init(&g_k230_parser);
    Balance_Init();
    Encoder_SetZero(ENCODER_AXIS_X);
    Menu_Init();

    DL_UART_Main_enableInterrupt(K230_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(K230_UART_INST_INT_IRQN);

    while (1)
    {
        Menu_Run();

        // ---- 20ms 控制周期 ----
        if (g_cl_trigger) {
            g_cl_trigger = 0;
            if (Balance_IsActive()) {
                Balance_Control();
            } else {
                Balance_GoHome();   // 归零中或空闲(no-op)
            }
        }

        // 完全空闲时确保电机停止
        if (!Balance_IsActive() && !Balance_IsGoingHome()
            && Motor_IsBusy(MOTOR_AXIS_X)) {
            Motor_Stop(MOTOR_AXIS_X);
        }
    }
}
