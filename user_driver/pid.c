/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


/**
 * @file    pid.c
 * @brief   位置式 PID 控制器实现
 */

#include "pid.h"

/**
 * @brief  初始化 PID 结构体（默认参数为占位值，需根据实际系统调参）
 * @param  pid  PID 结构体指针
 */
void PID_Init(PID_TypeDef *pid)
{
    pid->Kp = 0.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    pid->setpoint = 0.0f;

    pid->integral   = 0.0f;
    pid->last_error = 0.0f;

    pid->output_max     =  100.0f;
    pid->output_min     = -100.0f;
    pid->integral_limit =   50.0f;
    pid->integral_sep   =   10.0f;
}

/**
 * @brief  执行一次 PID 计算
 * @param  pid       PID 结构体指针
 * @param  measured  当前测量值（对于平衡系统即小球位置误差 x_mm）
 * @return PID 输出值（对于平衡系统即步进电机目标角度，单位：度）
 */
float PID_Update(PID_TypeDef *pid, float measured)
{
    float error = pid->setpoint - measured;   // 期望(0) - 实际误差

    // ---- P 项 ----
    float P = pid->Kp * error;

    // ---- I 项（积分分离：大偏差时只使用 PD，防止积分饱和）----
    if (fabsf(error) < pid->integral_sep) {
        pid->integral += error;
        // 积分限幅
        if (pid->integral > pid->integral_limit)
            pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit)
            pid->integral = -pid->integral_limit;
    }
    float I = pid->Ki * pid->integral;

    // ---- D 项（误差变化率）----
    float D = pid->Kd * (error - pid->last_error);
    pid->last_error = error;

    // ---- 合成输出并限幅 ----
    float output = P + I + D;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

/**
 * @brief  重置 PID 积分和上一次误差（模式切换时调用）
 */
void PID_Reset(PID_TypeDef *pid)
{
    pid->integral   = 0.0f;
    pid->last_error = 0.0f;
}

/**
 * @brief  在线调参
 */
void PID_Tune(PID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}
