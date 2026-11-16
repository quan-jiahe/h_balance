/**
 * @copyright (c) 2026 good1example (hzt), hya, cyx. All rights reserved.
 * @date      2026-08-01
 */


/**
 * @file    key.c
 * @brief   K230 触控导航 + OLED / K230 同步菜单
 *
 * UART 接收上、下、确认、返回事件；实体按键不再参与控制。
 *
 * 主菜单:
 *   > Show Data      — 实时显示 K230 数据 (x_mm, vx_mm_s)
 *   > Ball Balance   — 启动小球平衡 PID 控制
 */

#include "key.h"
#include "oled.h"
#include "balance_pid.h"
#include "mspm0_k230_comm.h"
#include <stdio.h>
#include <string.h>

/* ================================================================
 * 外部变量
 * ================================================================ */
extern volatile uint32_t sys_tick_ms;
extern volatile float   g_ball_error_mm;
extern volatile float   g_ball_velocity_mm_s;
extern volatile uint8_t g_ball_data_valid;
extern volatile uint8_t g_ball_confidence;

/* ================================================================
 * 静态变量
 * ================================================================ */
static int       menu_cursor   = 0;

/* 目标值设定模式 */
static int       s_target_adjust = 0;  // 1=设定目标值中
static int       s_target_sign   = 1;  // 1=加(+) / 0=减(-)

/* 数据显示模式 */
static int       s_show_data   = 0;   // 1=显示数据中
static uint32_t  s_data_draw_tick = 0;
#define DATA_DRAW_INTERVAL_MS  200U   // 200ms 刷新

/* 单生产者(ISR)、单消费者(主循环)队列，保持连续触控的顺序。 */
static volatile uint8_t s_keys[8];
static volatile uint8_t s_key_head = 0;
static volatile uint8_t s_key_tail = 0;
static uint8_t s_current_key = 0;
static char s_menu_lines[K230_MENU_ROWS][K230_MENU_COLS];

static uint32_t menu_start_tick = 0;
static uint8_t  draw_needed     = 1;

/* ================================================================
 * 调参系统
 * ================================================================ */
typedef enum { PT_FLOAT, PT_UINT32 } ParamValType;

typedef struct {
    const char   *name;
    ParamValType  type;
    void         *ptr;
    float         step;
} TuneParamDef;

static float _tune_get(const TuneParamDef *p) {
    if (p->type == PT_FLOAT) return *(float*)p->ptr;
    return (float)*(uint32_t*)p->ptr;
}
static void _tune_set(const TuneParamDef *p, float val) {
    if (p->type == PT_FLOAT) *(float*)p->ptr = val;
    else *(uint32_t*)p->ptr = (uint32_t)val;
}

/* 平衡 PID 参数 */
static const TuneParamDef s_pid_params[] = {
    {"Kp",   PT_FLOAT,  &g_balance_kp,   0.01f},
    {"Ki",   PT_FLOAT,  &g_balance_ki,   0.001f},
    {"Kd",   PT_FLOAT,  &g_balance_kd,   0.1f},
    {"KFF",  PT_FLOAT,  &g_balance_kff,  0.01f},
    {"KFF2", PT_FLOAT,  &g_balance_kff2, 0.0005f},
};
/* 自动序列参数 */
static const TuneParamDef s_seq_params[] = {
    {"Phase0 Ramp", PT_UINT32, &g_auto_seq_phase0_ramp_ms, 100.0f},
    {"Phase1 Ramp", PT_UINT32, &g_auto_seq_phase1_ramp_ms, 100.0f},
};
static const struct {
    const char         *name;
    const TuneParamDef *params;
    int                 count;
} s_tune_cats[] = {
    {"Balance PID", s_pid_params, 5},
    {"Auto Seq",    s_seq_params, 2},
};

static int s_tune_cat   = 0;   // 当前调参分类索引
static int s_tune_param = 0;   // 当前参数索引
static int s_tune_sign  = 1;   // +/- 符号
static int s_in_tuning  = 0;   // 0=主菜单, 1=分类选择, 2=参数选择, 3=调整值

/* ================================================================
 * 触控事件与菜单镜像
 * ================================================================ */
void Menu_QueueKey(uint8_t command)
{
    if (command < K230_KEY_UP || command > K230_MENU_SYNC) return;
    uint8_t next = (uint8_t)((s_key_head + 1u) % 8u);
    if (next == s_key_tail) return;
    s_keys[s_key_head] = command;
    s_key_head = next;
}

static void _menu_clear(void)
{
    memset(s_menu_lines, 0, sizeof(s_menu_lines));
    OLED_Clear();
}

static void _menu_show(u8 x, u8 y, u8 *text, u8 size)
{
    if (y / 16u < K230_MENU_ROWS) {
        char *line = s_menu_lines[y / 16u];
        strncpy(line, (const char *)text, K230_MENU_COLS - 1u);
        line[K230_MENU_COLS - 1u] = '\0';
    }
    OLED_ShowString(x, y, text, size);
}

static void _menu_refresh(void)
{
    OLED_Refresh();
    K230_Comm_SendMenu(s_menu_lines);
}

#define KEY_UP()    (s_current_key == K230_KEY_UP)
#define KEY_DOWN()  (s_current_key == K230_KEY_DOWN)
#define KEY_OK()    (s_current_key == K230_KEY_OK)
#define KEY_BACK()  (s_current_key == K230_KEY_BACK)

/* ================================================================
 * 显示辅助
 * ================================================================ */
static void _draw_line(int line, int is_cursor, const char *text)
{
    char buf[22];
    if (is_cursor)
        snprintf(buf, sizeof(buf), ">%s", text);
    else
        snprintf(buf, sizeof(buf), " %s", text);
    _menu_show(0, line * 16, (u8 *)buf, 16);
}

/* ================================================================
 * 菜单页绘制
 * ================================================================ */
static void _draw_main_menu(void)
{
    const char *items[] = { "Set Target", "Ball Balance", "Auto Seq", "Tuning", "Show Data" };
    int count = 5;

    /* 滚动: 光标在底部时偏移 */
    int scroll = (menu_cursor >= 3) ? (menu_cursor - 2) : 0;

    _menu_clear();
    _menu_show(0, 0, (u8 *)"-- Main Menu --", 16);
    for (int i = scroll; i < count && i < scroll + 3; i++) {
        _draw_line(i - scroll + 1, (i == menu_cursor), items[i]);
    }
    _menu_refresh();
}

/* ---- 目标值设定 ---- */
static void _draw_target_adjust(void)
{
    char val_line[22], buf_plus[22], buf_minus[22];

    snprintf(val_line, sizeof(val_line), "Target: %.0f mm",
             (double)g_ball_target_mm);
    snprintf(buf_plus,  sizeof(buf_plus),  "[+] +5 mm");
    snprintf(buf_minus, sizeof(buf_minus), "[-] -5 mm");

    _menu_clear();
    _menu_show(0, 0,  (u8 *)"Set Target", 16);
    _menu_show(0, 16, (u8 *)val_line, 16);
    _draw_line(2, (s_target_sign == 1), buf_plus);
    _draw_line(3, (s_target_sign == 0), buf_minus);
    _menu_refresh();
}

static void _handle_target_adjust(void)
{
    if (KEY_UP()) {
        s_target_sign = 1;
        draw_needed = 1;
    } else if (KEY_DOWN()) {
        s_target_sign = 0;
        draw_needed = 1;
    }
    if (KEY_OK()) {
        float delta = (s_target_sign == 1) ? 5.0f : -5.0f;
        Balance_SetTarget(g_ball_target_mm + delta);
        draw_needed = 1;
    }
    if (KEY_BACK()) {
        s_target_adjust = 0;
        menu_cursor = 0;
        draw_needed = 1;
    }
}

/* ---- 实时数据显示 ---- */
static void _draw_show_data(void)
{
    char line0[22], line1[22], line2[22];

    snprintf(line0, sizeof(line0), "Pos:%+.1f mm",
             (double)g_ball_error_mm);
    snprintf(line1, sizeof(line1), "Vel:%+.1f mm/s",
             (double)g_ball_velocity_mm_s);
    snprintf(line2, sizeof(line2), "Ang:%.1f C:%u%%",
             (double)Balance_GetCurrentAngle(),
             (unsigned int)g_ball_confidence);

    _menu_clear();
    _menu_show(0, 0,  (u8 *)"K230 Live Data", 16);
    _menu_show(0, 16, (u8 *)line0, 16);
    _menu_show(0, 32, (u8 *)line1, 16);
    _menu_show(0, 48, (u8 *)line2, 16);
    _menu_refresh();
    s_data_draw_tick = sys_tick_ms;
}

/* ---- 小球平衡模式显示 ---- */
static void _draw_balance_running(void)
{
    char line0[22], line1[22], line2[22];

    snprintf(line0, sizeof(line0), "%s",
             Balance_GetStateStr());

    if (Balance_IsAutoSeq()) {
        snprintf(line1, sizeof(line1), "%s",
                 Balance_GetAutoSeqPhaseStr());
        snprintf(line2, sizeof(line2), "Err:%+.1fmm OK:Stop",
                 (double)Balance_GetError());
    } else {
        snprintf(line1, sizeof(line1), "Err:%+.1fmm",
                 (double)Balance_GetError());
        snprintf(line2, sizeof(line2), "OK to Stop");
    }

    _menu_clear();
    _menu_show(0, 0,  (u8 *)"Ball Balance", 16);
    _menu_show(0, 16, (u8 *)line0, 16);
    _menu_show(0, 32, (u8 *)line1, 16);
    _menu_show(0, 48, (u8 *)line2, 16);
    _menu_refresh();
}

/* ---- 调参: 分类选择 ---- */
static void _draw_tune_cat(void)
{
    _menu_clear();
    _menu_show(0, 0, (u8 *)"-- Tuning --", 16);
    for (int i = 0; i < 2; i++) {
        _draw_line(i + 1, (i == s_tune_cat), s_tune_cats[i].name);
    }
    _menu_refresh();
}
static void _handle_tune_cat(void)
{
    if (KEY_UP())   { if (s_tune_cat > 0) { s_tune_cat--; draw_needed = 1; } }
    if (KEY_DOWN()) { if (s_tune_cat < 1) { s_tune_cat++; draw_needed = 1; } }
    if (KEY_OK())   { s_in_tuning = 2; s_tune_param = 0; draw_needed = 1; }
    if (KEY_BACK()) { s_in_tuning = 0; menu_cursor = 4; draw_needed = 1; }
}

/* ---- 调参: 参数选择 ---- */
static void _draw_tune_param(void)
{
    const TuneParamDef *p = s_tune_cats[s_tune_cat].params;
    int count = s_tune_cats[s_tune_cat].count;
    char buf[22];

    _menu_clear();
    snprintf(buf, sizeof(buf), "%s", s_tune_cats[s_tune_cat].name);
    _menu_show(0, 0, (u8 *)buf, 16);
    /* 只显示 3 行, 需要滚动 */
    int scroll = (s_tune_param >= 2) ? (s_tune_param - 1) : 0;
    for (int i = scroll; i < count && i < scroll + 2; i++) {
        snprintf(buf, sizeof(buf), "%s:%.3f", p[i].name, (double)_tune_get(&p[i]));
        _draw_line(i - scroll + 1, (i == s_tune_param), buf);
    }
    _menu_refresh();
}
static void _handle_tune_param(void)
{
    int count = s_tune_cats[s_tune_cat].count;
    if (KEY_UP())   { if (s_tune_param > 0) { s_tune_param--; draw_needed = 1; } }
    if (KEY_DOWN()) { if (s_tune_param < count - 1) { s_tune_param++; draw_needed = 1; } }
    if (KEY_OK())   { s_in_tuning = 3; s_tune_sign = 1; draw_needed = 1; }
    if (KEY_BACK()) { s_in_tuning = 1; draw_needed = 1; }
}

/* ---- 调参: 调整值 ---- */
static void _draw_tune_adjust(void)
{
    const TuneParamDef *p = &s_tune_cats[s_tune_cat].params[s_tune_param];
    char line0[22], line1[22], line2[22];

    snprintf(line0, sizeof(line0), "%s:%.3f", p->name, (double)_tune_get(p));
    snprintf(line1, sizeof(line1), "[+] +%.3f", (double)p->step);
    snprintf(line2, sizeof(line2), "[-] -%.3f", (double)p->step);

    _menu_clear();
    _menu_show(0, 0,  (u8 *)"Adjust", 16);
    _menu_show(0, 16, (u8 *)line0, 16);
    _draw_line(2, (s_tune_sign == 1), line1);
    _draw_line(3, (s_tune_sign == 0), line2);
    _menu_refresh();
}
static void _handle_tune_adjust(void)
{
    const TuneParamDef *p = &s_tune_cats[s_tune_cat].params[s_tune_param];
    if (KEY_UP())        { s_tune_sign = 1; draw_needed = 1; }
    else if (KEY_DOWN()) { s_tune_sign = 0; draw_needed = 1; }
    if (KEY_OK()) {
        float delta = (s_tune_sign == 1) ? p->step : -p->step;
        _tune_set(p, _tune_get(p) + delta);
        draw_needed = 1;
    }
    if (KEY_BACK()) { s_in_tuning = 2; draw_needed = 1; }
}

/* ================================================================
 * 按键处理
 * ================================================================ */
static void _handle_main_menu(void)
{
    if (KEY_UP())   { if (menu_cursor > 0) { menu_cursor--; draw_needed = 1; } }
    if (KEY_DOWN()) { if (menu_cursor < 4) { menu_cursor++; draw_needed = 1; } }
    if (KEY_OK()) {
        switch (menu_cursor) {
        case 0: // Set Target
            s_target_adjust = 1;
            draw_needed     = 1;
            break;
        case 1: // Ball Balance
            Balance_Start();
            draw_needed  = 1;
            break;
        case 2: // Auto Seq
            Balance_StartAutoSeq();
            draw_needed  = 1;
            break;
        case 3: // Tuning
            s_in_tuning  = 1;
            s_tune_cat   = 0;
            draw_needed  = 1;
            break;
        case 4: // Show Data
            K230_Comm_SendStartBalance();  // 仅开启视觉数据，不启动电机闭环。
            s_show_data  = 1;
            draw_needed  = 1;
            s_data_draw_tick = 0;
            break;
        }
    }
}

/* ================================================================
 * 公开 API
 * ================================================================ */
void Menu_Init(void)
{
    menu_cursor     = 0;
    s_target_adjust = 0;
    s_target_sign   = 1;
    s_show_data     = 0;
    s_in_tuning     = 0;
    s_tune_cat      = 0;
    s_tune_param    = 0;

    menu_start_tick = sys_tick_ms;
    draw_needed     = 1;

    s_key_head = s_key_tail = 0;
    s_current_key = 0;

    _draw_main_menu();
}

void Menu_Run(void)
{
    if (sys_tick_ms - menu_start_tick < 500) return;

    s_current_key = 0;
    if (s_key_tail != s_key_head) {
        s_current_key = s_keys[s_key_tail];
        s_key_tail = (uint8_t)((s_key_tail + 1u) % 8u);
        if (s_current_key == K230_MENU_SYNC) draw_needed = 1;
    }

    /* ---- 目标值设定模式 (先绘制后处理: BACK 设置 draw_needed 留给下一轮主菜单) ---- */
    if (s_target_adjust) {
        if (draw_needed) { draw_needed = 0; _draw_target_adjust(); }
        _handle_target_adjust();
        return;
    }

    /* ---- 数据显示模式 (先绘制后处理: BACK 设置 draw_needed 留给下一轮主菜单) ---- */
    if (s_show_data) {
        if (draw_needed) { draw_needed = 0; _draw_show_data(); }
        if (sys_tick_ms - s_data_draw_tick >= DATA_DRAW_INTERVAL_MS) _draw_show_data();
        if (KEY_BACK()) { s_show_data = 0; menu_cursor = 0; draw_needed = 1; }
        return;
    }

    /* ---- 小球平衡模式 (先绘制后处理: OK 设置 draw_needed 留给下一轮主菜单) ---- */
    if (Balance_IsActive()) {
        if (draw_needed) { draw_needed = 0; _draw_balance_running(); }
        if (KEY_OK() || KEY_BACK()) { Balance_Stop(); menu_cursor = 0; draw_needed = 1; }
        return;
    }

    /* ---- 调参子菜单 (s_in_tuning: 1=分类, 2=参数, 3=调整) ---- */
    if (s_in_tuning == 1) {
        if (draw_needed) { draw_needed = 0; _draw_tune_cat(); }
        _handle_tune_cat();
        return;
    }
    if (s_in_tuning == 2) {
        if (draw_needed) { draw_needed = 0; _draw_tune_param(); }
        _handle_tune_param();
        return;
    }
    if (s_in_tuning == 3) {
        if (draw_needed) { draw_needed = 0; _draw_tune_adjust(); }
        _handle_tune_adjust();
        return;
    }

    /* ---- 主菜单 (先绘制后处理: OK 设置 draw_needed 留给下一轮子页面) ---- */
    if (draw_needed) { draw_needed = 0; _draw_main_menu(); }
    _handle_main_menu();
}

