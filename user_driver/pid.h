/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


/**
 * @file    pid.h
 * @brief   位置式 PID 控制器（可复用）
 *
 * 公式: u(k) = Kp*e(k) + Ki*Σe(i) + Kd*(e(k)-e(k-1))
 * 特性: 积分分离 + 积分限幅 + 输出限幅
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <math.h>

typedef struct {
    float Kp;                  // 比例系数
    float Ki;                  // 积分系数
    float Kd;                  // 微分系数
    float setpoint;            // 期望值（通常为 0）

    float integral;            // 积分累积和
    float last_error;          // 上一次误差

    float output_max;          // 输出上限
    float output_min;          // 输出下限
    float integral_limit;      // 积分项限幅
    float integral_sep;        // 积分分离阈值（|误差|大于此值时不累积积分）
} PID_TypeDef;

void  PID_Init(PID_TypeDef *pid);
float PID_Update(PID_TypeDef *pid, float measured);
void  PID_Reset(PID_TypeDef *pid);
void  PID_Tune(PID_TypeDef *pid, float kp, float ki, float kd);

#endif
