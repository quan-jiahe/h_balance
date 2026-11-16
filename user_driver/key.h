/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


#ifndef KEY_H
#define KEY_H

#include <stdint.h>

void Menu_Init(void);
void Menu_Run(void);                  // 主循环处理触控事件并刷新菜单。
void Menu_QueueKey(uint8_t command);  // UART ISR 入队，不在中断里执行控制。

#endif
