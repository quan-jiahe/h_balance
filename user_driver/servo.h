/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


/**
 * ==================== 单路舵机驱动 ====================
 *
 * 基于 PWM_SERVO (TIMA1), 50Hz PWM, 0.5~2.5ms 对应 0~180°
 *
 * 使用前请在 SysConfig 中:
 *   1. 配置 PWM_SERVO → TIMA1 → PWM 模式, 预分频 64, 周期 10000
 *   2. 将目标通道引脚设为 PA27 (或其他需要的引脚)
 *   3. 确保勾选 "Start Counter"
 * ==============================================================
 */

#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

void Servo_Init(void);
void Servo_SetAngle(uint8_t angle);   /* 0~180°, 超范围自动限幅 */

#endif
