/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


/**
 * @file    balance_pid.c
 * @brief   小球平衡位置环 PID 控制实现
 *
 * 控制链: K230 (x_mm, vx_mm_s) → 位置式PID → 目标角度 → 步进电机闭环
 *
 * ================================================================
 * 调参指南 (TUNING GUIDE)
 * ================================================================
 *
 * 步骤 1 — 方向确认:
 *   设 g_balance_kp=0.5, KI=0, KD=0 → 启动平衡 → 观察小球:
 *   - 小球加速远离中心 → BALANCE_DIRECTION_SIGN 取反
 *   - 小球向中心靠拢但震荡 → 步骤 2
 *
 * 步骤 2 — 比例调节:
 *   逐步增大 g_balance_kp (+0.2/次):
 *   - 过冲严重 → Kp 偏大, 回退
 *   - 回中缓慢 → Kp 偏小, 继续
 *   - 目标: 小球快速回中, 轻微过冲
 *
 * 步骤 3 — 微分阻尼 (位置微分 + 速度反馈):
 *   逐步增大 g_balance_kd (+0.5/次):
 *   - 震荡衰减快 → 正确, 继续
 *   - 电机高频抖动 → KD 过大
 *   - 目标: 1-2 个来回稳定
 *
 * 步骤 4 — 积分消静差:
 *   逐步增大 g_balance_ki (+0.01/次):
 *   - 停在偏离位置 → 积分不足
 *   - 来回摆动不收敛 → 积分过大
 *   - 目标: 精确停在中心 ±1mm
 *
 * 步骤 5 — 动态速度前馈 (可选, 进阶):
 *   启用 g_balance_kff (线性项) + g_balance_kff2 (平方项):
 *   输出 = KFF*v + KFF2*v*|v|, 速度越快阻尼越大
 *   - 先调 KFF 至低速稳定, 再逐步加 KFF2 抑制高速过冲
 *   - 小球向中心高速运动时提前减速
 *   - 改善大偏差瞬态响应
 */

#include "balance_pid.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "mspm0_k230_comm.h"
#include <math.h>

/* ================================================================
 * 调参常量
 * ================================================================ */

/* PID 系数 (全局变量, 菜单可调) */
float g_balance_kp   = 0.40f;
float g_balance_ki   = 0.018f;
float g_balance_kd   = 5.0f;
float g_balance_kff  = -0.325f;
float g_balance_kff2 = -0.002f;

/* 输出方向 (+1 或 -1) */
#define BALANCE_DIRECTION_SIGN   (1.0f)

/* 机械参数 */
#define BALANCE_NEUTRAL_ANGLE_DEG  0.0f
#define BALANCE_MAX_ANGLE_DEG      30.0f

/* 保护 */
#define BALANCE_ERROR_TIMEOUT_MS   500U
#define BALANCE_DEADZONE_MM        1.0f
#define CL_TOLERANCE_DEG           0.5f
#define CL_START_THRESHOLD_DEG     1.0f   // 滞回: 电机停止时需超过此值才启动
#define CL_STOP_THRESHOLD_DEG      0.3f   // 滞回: 电机运行中需低于此值才停止

/* ================================================================
 * 全局变量 (由 uart.c ISR 更新, balance_pid / key 读取)
 * ================================================================ */

volatile float   g_ball_error_mm     = 0.0f;
volatile float   g_ball_velocity_mm_s = 0.0f;
volatile uint8_t g_ball_data_valid   = 0;
volatile uint8_t g_ball_confidence   = 0;
volatile uint8_t g_ball_status       = 0;

/* 系统时间戳 (main.c) */
extern volatile uint32_t sys_tick_ms;

/* ================================================================
 * 静态变量
 * ================================================================ */

/* 平衡控制状态机 */
typedef enum {
    BAL_STATE_IDLE,         // 空闲
    BAL_STATE_PID,          // PID 闭环控制
    BAL_STATE_AUTO_SEQ,     // 自动序列: +50mm(5s) → -50mm(稳定)
} BalanceState;

/* 自动序列阶段 */
typedef enum {
    AUTO_SEQ_PHASE_0,       // 阶段0: 目标+50mm, 到达+45mm后切换, 不超过+60mm
    AUTO_SEQ_PHASE_1,       // 阶段1: 目标-50mm, 稳定在[-60,-40]
} AutoSeqPhase;

#define AUTO_SEQ_PHASE0_TARGET_MM     50.0f   // 阶段0 目标
#define AUTO_SEQ_PHASE0_MAX_MM        60.0f   // 阶段0 上限 (超过强制回拉)
#define AUTO_SEQ_PHASE0_SWITCH_MM     48.0f   // 球到达此位置后切换到阶段1
#define AUTO_SEQ_PHASE0_BOOST_MM      10.0f   // 斜坡终点过冲量, 补偿稳态误差
#define AUTO_SEQ_PHASE1_TARGET_MM    -50.0f   // 阶段1 目标
#define AUTO_SEQ_PHASE1_BOOST_MM     -5.0f    // 斜坡终点过冲量

/* 位置目标 */
float    g_ball_target_mm        = 0.0f;   // PID 位置目标 (mm), 菜单可调

static PID_TypeDef  balance_pid;
static BalanceState s_bal_state       = BAL_STATE_IDLE;
static int          s_go_home         = 0;
static float        s_target_angle    = 0.0f;
static float        s_last_error      = 0.0f;
static float        s_last_velocity   = 0.0f;
static uint32_t     s_timeout_count    = 0;
static int          s_last_motor_dir   = -1; // 上次电机方向, -1=未知
static uint32_t     s_last_motor_freq  = 0;  // 上次电机频率, 用于防卡顿守卫

/* 运行时可切换的速度前馈系数 */
static float s_kff  = 0.0f;
static float s_kff2 = 0.0f;

/* 自动序列变量 */
static AutoSeqPhase s_auto_seq_phase       = AUTO_SEQ_PHASE_0;
static uint32_t     s_auto_seq_start_ms    = 0;
static float        s_ramp_start_pos       = 0.0f;  // 阶段切换时的球位置
static uint32_t     s_ramp_start_ms        = 0;     // 阶段切换时刻

uint32_t g_auto_seq_phase0_ramp_ms = 800U;   // 阶段0 斜坡时长 (ms), 菜单可调
uint32_t g_auto_seq_phase1_ramp_ms = 1500U;  // 阶段1 斜坡时长 (ms), 菜单可调

/* ================================================================
 * 公开实现
 * ================================================================ */

void Balance_Init(void)
{
    PID_Init(&balance_pid);
    balance_pid.Kp = g_balance_kp;
    balance_pid.Ki = g_balance_ki;
    balance_pid.Kd = g_balance_kd;
    balance_pid.setpoint = 0.0f;

    balance_pid.output_max =  BALANCE_MAX_ANGLE_DEG;
    balance_pid.output_min = -BALANCE_MAX_ANGLE_DEG;
    balance_pid.integral_limit = 300.0f;
    balance_pid.integral_sep   = 10.0f;

    s_kff  = g_balance_kff;
    s_kff2 = g_balance_kff2;

    g_ball_error_mm     = 0.0f;
    g_ball_velocity_mm_s = 0.0f;
    g_ball_data_valid   = 0;
    g_ball_confidence   = 0;
    g_ball_status       = 0;

    s_bal_state      = BAL_STATE_IDLE;
    s_go_home        = 1;   // 上电后自动归零
    s_target_angle   = BALANCE_NEUTRAL_ANGLE_DEG;
    s_last_error     = 0.0f;
    s_last_velocity  = 0.0f;
    s_timeout_count  = 0;

    Encoder_SetZero(ENCODER_AXIS_X);
}


void Balance_Control(void)
{
    /* ---- 状态机分流 ---- */
    switch (s_bal_state) {

    case BAL_STATE_PID:
        break;

    /* 自动序列: 两阶段均用斜坡过渡, 防过冲 */
    case BAL_STATE_AUTO_SEQ: {
        if (s_auto_seq_phase == AUTO_SEQ_PHASE_0) {
            /* 斜坡: 0 → +60mm (目标+boost), 线性过渡 */
            uint32_t elapsed = sys_tick_ms - s_auto_seq_start_ms;
            float    t = (float)elapsed / (float)g_auto_seq_phase0_ramp_ms;
            if (t > 1.0f) t = 1.0f;
            balance_pid.setpoint = t * (AUTO_SEQ_PHASE0_TARGET_MM + AUTO_SEQ_PHASE0_BOOST_MM);

            /* 球到达 +38mm 以上: 进入阶段1 */
            if (s_last_error >= AUTO_SEQ_PHASE0_SWITCH_MM) {
                s_auto_seq_phase = AUTO_SEQ_PHASE_1;
                s_ramp_start_pos = s_last_error;      // 记录当前球位置
                s_ramp_start_ms  = sys_tick_ms;
                PID_Reset(&balance_pid);
            }
        } else {
            /* 斜坡: 从切换位置过渡到 -55mm (目标+boost) */
            uint32_t ramp_elapsed = sys_tick_ms - s_ramp_start_ms;
            float    t = (float)ramp_elapsed / (float)g_auto_seq_phase1_ramp_ms;
            if (t > 1.0f) t = 1.0f;
            balance_pid.setpoint = s_ramp_start_pos
                                 + t * (AUTO_SEQ_PHASE1_TARGET_MM + AUTO_SEQ_PHASE1_BOOST_MM - s_ramp_start_pos);
        }
        break;
    }

    default:
        return;
    }

    /* ---- PID 闭环控制 (BAL_STATE_PID / BAL_STATE_AUTO_SEQ 到达此处) ---- */
    float error_mm    = s_last_error;
    float velocity_mm_s = s_last_velocity;
    uint8_t has_new_data = 0;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (g_ball_data_valid) {
        error_mm        = g_ball_error_mm;
        velocity_mm_s   = g_ball_velocity_mm_s;
        g_ball_data_valid = 0;
        has_new_data    = 1;
    }
    if (primask == 0U) __enable_irq();

    if (has_new_data) {
        s_timeout_count   = 0;
        s_last_error      = error_mm;
        s_last_velocity   = velocity_mm_s;
    } else {
        s_timeout_count++;
    }

    if (s_timeout_count > (BALANCE_ERROR_TIMEOUT_MS / CL_CTRL_PERIOD_MS)) {
        Motor_Stop(MOTOR_AXIS_X);
        return;
    }

    if (error_mm >  120.0f) error_mm =  120.0f;
    if (error_mm < -120.0f) error_mm = -120.0f;
    if (fabsf(error_mm) < BALANCE_DEADZONE_MM) error_mm = 0.0f;

    float pid_out = PID_Update(&balance_pid, error_mm);
    pid_out *= BALANCE_DIRECTION_SIGN;
    float abs_v = fabsf(velocity_mm_s);
    pid_out += s_kff * velocity_mm_s
             + s_kff2 * velocity_mm_s * abs_v;

    /* 机械不对称补偿: 逆时针方向(重力辅助)降低有效增益, 防止过冲震荡 */
    if (pid_out < 0.0f) {
        pid_out *= 0.75f;
    }

    s_target_angle = BALANCE_NEUTRAL_ANGLE_DEG + pid_out;
    if (s_target_angle >  BALANCE_MAX_ANGLE_DEG) s_target_angle =  BALANCE_MAX_ANGLE_DEG;
    if (s_target_angle < -BALANCE_MAX_ANGLE_DEG) s_target_angle = -BALANCE_MAX_ANGLE_DEG;

    /* 自动序列阶段0: 若小球超过+60mm, 强制最大回拉角度 */
    if (s_bal_state == BAL_STATE_AUTO_SEQ && s_auto_seq_phase == AUTO_SEQ_PHASE_0) {
        if (error_mm > AUTO_SEQ_PHASE0_MAX_MM)
            s_target_angle = -BALANCE_MAX_ANGLE_DEG;
    }

    float cur_angle = Encoder_GetAngle(ENCODER_AXIS_X);
    float angle_err = s_target_angle - cur_angle;
    while (angle_err >  180.0f) angle_err -= 360.0f;
    while (angle_err < -180.0f) angle_err += 360.0f;

    /* 滞回: 停止时用宽阈值防误触发, 运行中用窄阈值坚持到底 */
    if (s_last_motor_dir == -1) {
        if (fabsf(angle_err) <= CL_START_THRESHOLD_DEG) return;
    } else {
        if (fabsf(angle_err) <= CL_STOP_THRESHOLD_DEG) {
            Motor_Stop(MOTOR_AXIS_X);
            s_last_motor_dir  = -1;
            s_last_motor_freq = 0;
            return;
        }
    }

    uint8_t dir = (angle_err > 0.0f) ? 1 : 0;
    if ((int)dir != s_last_motor_dir) {
        Motor_Stop(MOTOR_AXIS_X);
        Motor_SetDirection(MOTOR_AXIS_X, dir);
        s_last_motor_dir = dir;
    }

    float abs_err = fabsf(angle_err);
    uint32_t freq;
    if      (abs_err > 30.0f) freq = 2200;
    else if (abs_err > 5.0f)  freq = 800;
    else                      freq = 200;
    /* 守卫: 仅未运行/方向变/频率变时重启, 防止每周期停-启卡顿 */
    if (!Motor_IsBusy(MOTOR_AXIS_X) || (int)dir != s_last_motor_dir || freq != s_last_motor_freq) {
        Motor_StartContinuous(freq, dir);
        s_last_motor_dir  = (int)dir;
        s_last_motor_freq = freq;
    }
}


void Balance_Start(void)
{
    PID_Reset(&balance_pid);
    s_bal_state        = BAL_STATE_PID;
    balance_pid.setpoint = g_ball_target_mm;  // 使用菜单设定的目标值
    s_target_angle     = BALANCE_NEUTRAL_ANGLE_DEG;
    s_timeout_count    = 0;
    balance_pid.Kp = g_balance_kp;
    balance_pid.Ki = g_balance_ki;
    s_kff  = g_balance_kff;
    s_kff2 = g_balance_kff2;
    K230_Comm_SendStartBalance();
}

void Balance_StartAutoSeq(void)
{
    PID_Reset(&balance_pid);
    s_bal_state         = BAL_STATE_AUTO_SEQ;
    s_auto_seq_phase    = AUTO_SEQ_PHASE_0;
    s_auto_seq_start_ms = sys_tick_ms;
    balance_pid.setpoint = AUTO_SEQ_PHASE0_TARGET_MM;
    balance_pid.Kp = 0.4f;
    balance_pid.Ki = 0.01f;
    s_kff  = -0.15f;
    s_kff2 = -0.001f;
    s_target_angle      = BALANCE_NEUTRAL_ANGLE_DEG;
    s_timeout_count     = 0;
    K230_Comm_SendStartBalance();
}

void Balance_SetTarget(float mm)
{
    if (mm >  120.0f) mm =  120.0f;
    if (mm < -120.0f) mm = -120.0f;
    g_ball_target_mm = mm;
}

float Balance_GetBallTarget(void) { return g_ball_target_mm; }


void Balance_Stop(void)
{
    if (s_bal_state == BAL_STATE_PID || s_bal_state == BAL_STATE_AUTO_SEQ) {
        K230_Comm_SendStopBalance();
    }
    s_bal_state       = BAL_STATE_IDLE;
    Motor_Stop(MOTOR_AXIS_X);
    s_last_motor_dir  = -1;
    s_last_motor_freq = 0;
    Balance_RequestGoHome();
}


int Balance_IsActive(void)  { return (s_bal_state != BAL_STATE_IDLE); }
int Balance_IsGoingHome(void) { return s_go_home; }

int  Balance_IsPidRunning(void)  { return (s_bal_state == BAL_STATE_PID); }
int  Balance_IsAutoSeq(void)     { return (s_bal_state == BAL_STATE_AUTO_SEQ); }

const char *Balance_GetStateStr(void)
{
    switch (s_bal_state) {
    case BAL_STATE_PID:         return "PID Control";
    case BAL_STATE_AUTO_SEQ:    return "Auto Seq";
    default:                    return "Idle";
    }
}

/** 获取自动序列当前阶段字符串 (供 OLED 显示) */
const char *Balance_GetAutoSeqPhaseStr(void)
{
    switch (s_auto_seq_phase) {
    case AUTO_SEQ_PHASE_0: return "Phase0: ->+50mm";
    case AUTO_SEQ_PHASE_1: return "Phase1: ->-50mm";
    default:               return "?";
    }
}


void Balance_RequestGoHome(void)
{
    PID_Reset(&balance_pid);
    s_go_home = 1;
}


/**
 * @brief  执行一次归零角度闭环 (每控制周期调用)
 * @return 1=仍在归零中, 0=已到 0°
 */
int Balance_GoHome(void)
{
    if (!s_go_home) return 0;

    float cur_angle = Encoder_GetAngle(ENCODER_AXIS_X);
    float angle_err = 0.0f - cur_angle;   // target = 0°
    while (angle_err >  180.0f) angle_err -= 360.0f;
    while (angle_err < -180.0f) angle_err += 360.0f;

    /* 滞回: 停止时用宽阈值, 运行中用窄阈值, 防止归零到位时的边界震荡 */
    if (s_last_motor_dir == -1) {
        if (fabsf(angle_err) <= CL_START_THRESHOLD_DEG) return 0;
    } else {
        if (fabsf(angle_err) <= CL_STOP_THRESHOLD_DEG) {
            Motor_Stop(MOTOR_AXIS_X);
            s_last_motor_dir  = -1;
            s_last_motor_freq = 0;
            s_go_home = 0;
            return 0;
        }
    }

    uint8_t dir = (angle_err > 0.0f) ? 1 : 0;
    if ((int)dir != s_last_motor_dir) {
        Motor_Stop(MOTOR_AXIS_X);
        Motor_SetDirection(MOTOR_AXIS_X, dir);
        s_last_motor_dir = dir;
    }

    float abs_err = fabsf(angle_err);
    uint32_t freq;
    if      (abs_err > 30.0f) freq = 2200;
    else if (abs_err > 5.0f)  freq = 800;
    else                      freq = 200;

    /* 守卫: 仅未运行/方向变/频率变时重启 */
    if (!Motor_IsBusy(MOTOR_AXIS_X) || (int)dir != s_last_motor_dir || freq != s_last_motor_freq) {
        Motor_StartContinuous(freq, dir);
        s_last_motor_dir  = (int)dir;
        s_last_motor_freq = freq;
    }

    return 1;
}

float   Balance_GetError(void)        { return s_last_error; }
float   Balance_GetVelocity(void)     { return s_last_velocity; }
float   Balance_GetTargetAngle(void)  { return s_target_angle; }
float   Balance_GetCurrentAngle(void) { return Encoder_GetAngle(ENCODER_AXIS_X); }
uint8_t Balance_GetConfidence(void)   { return g_ball_confidence; }
