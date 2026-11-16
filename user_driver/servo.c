/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


/**
 * ==================== 单路舵机驱动 ====================
 *
 * 50Hz PWM, 0.5~2.5ms 对应 0~180°
 * 使用 PWM_SERVO (TIMG7, CH1=PA27)
 *
 * 时钟: 80MHz / DIV8 / prescale=10 = 1MHz
 * 周期: 20000 ticks / 1MHz = 20ms (50Hz)
 * ==============================================================
 */

#include "servo.h"
#include "ti_msp_dl_config.h"

/* 1MHz 定时器时钟, 周期 = 1000000/50 = 20000 ticks
 * 0°=0.5ms=500ticks, 180°=2.5ms=2500ticks
 * CC = period - (500 + angle * 2000 / 180)                 */
#define SERVO_PERIOD_TICKS       (20000U)
#define SERVO_MIN_PULSE_TICKS    (500U)
#define SERVO_RANGE_PULSE_TICKS  (2000U)

#define SERVO_CC_IDX             GPIO_PWM_SERVO_C1_IDX

static uint8_t ClampAngle(uint8_t a)
{
    if (a > 180) a = 180;
    return a;
}

static uint16_t AngleToPulseTicks(uint8_t angle)
{
    return (uint16_t)(SERVO_MIN_PULSE_TICKS +
                      ((uint32_t)angle * SERVO_RANGE_PULSE_TICKS) / 180UL);
}

static uint16_t AngleToCC(uint8_t angle)
{
    return (uint16_t)(SERVO_PERIOD_TICKS - AngleToPulseTicks(angle));
}

void Servo_Init(void)
{
    DL_TimerG_stopCounter(PWM_SERVO_INST);
    DL_TimerG_setCaptureCompareValue(PWM_SERVO_INST, AngleToCC(90),
                                     SERVO_CC_IDX);
    DL_TimerG_startCounter(PWM_SERVO_INST);
}

void Servo_SetAngle(uint8_t angle)
{
    angle = ClampAngle(angle);
    DL_TimerG_setCaptureCompareValue(PWM_SERVO_INST, AngleToCC(angle),
                                     SERVO_CC_IDX);
}
