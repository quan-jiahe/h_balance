/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


/*
 * init.c — 系统初始化模块
 */

#include "init.h"
#include "oled.h"
#include "beep.h"
#include "motor.h"
#include "encoder.h"

void System_Init(void)
{
    /* 1. SysConfig 外设初始化 (含 K230 UART 时钟+引脚) */
    SYSCFG_DL_init();

    /* 2. OLED 初始化 */
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    /* 3. 步进电机初始化 (STEP/DIR/EN) */
    Motor_Init();

    /* 4. 编码器初始化 (QEI + PWM 角度) */
    Encoder_Init();

    /* 5. 蜂鸣器默认关闭 */
    Beep_Off();
}
