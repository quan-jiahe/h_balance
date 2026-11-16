/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


#include "beep.h"

void Beep_On(void)
{
    DL_GPIO_clearPins(BEEP_PORT, BEEP_PIN1_PIN);   // 低电平触发
}

void Beep_Off(void)
{
    DL_GPIO_setPins(BEEP_PORT, BEEP_PIN1_PIN);     // 高电平关闭
}
