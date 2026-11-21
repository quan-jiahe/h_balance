#include "link_test.h"
#include "board.h"
#include "center_balance.h"
#include "fixtures.h"
#include "pulse_cycle.h"
#include "startup_level.h"
#include "balance_tuning.h"
#include "vofa_tuning.h"
#include "vofa_uart.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static volatile uint32_t *const result = (volatile uint32_t *)0x2001E000u;
static uint32_t now_ms;
static unsigned pulse_count, up_pulses;
static int furthest_position;
static bool pulse_direction, pulse_fails;
static bool imu_offline, imu_stale, motor_stalled;
static int imu_noise;
static int position, motor_sign = 1;
static float motion_scale = 1.0f, imu_sign = 1.0f;

static uint8_t tuning_rx[256], tuning_tx[80];
static unsigned tuning_head, tuning_tail, tuning_tx_size, tuning_frames;
static uint32_t tuning_errors;
static bool tuning_tx_full;
void vofa_uart_init(void)
{
    tuning_head = tuning_tail = tuning_tx_size = tuning_frames = 0;
    tuning_errors = 0;
    tuning_tx_full = false;
}
bool vofa_uart_read(uint8_t *byte)
{
    if (tuning_tail == tuning_head) { return false; }
    *byte = tuning_rx[tuning_tail++];
    return true;
}
bool vofa_uart_write(const uint8_t *data, size_t size)
{
    if (tuning_tx_full || size > sizeof(tuning_tx)) { return false; }
    memcpy(tuning_tx, data, size);
    tuning_tx_size = size;
    ++tuning_frames;
    return true;
}
uint32_t vofa_uart_errors(void) { return tuning_errors; }

void SystemInit(void) { }

bool board_step_pulse(bool forward)
{
    ++pulse_count;
    if (forward) { ++up_pulses; }
    pulse_direction = forward;
    if (!pulse_fails && !motor_stalled) { position += forward ? motor_sign : -motor_sign; }
    if (position > furthest_position) { furthest_position = position; }
    return !pulse_fails;
}

uint32_t board_millis(void) { return now_ms; }
bool board_imu_init(void) { return !imu_offline; }

int board_imu_read(int16_t xyz[3])
{
    if (imu_offline) { return -1; }
    if (imu_stale) { return 0; }
    const float radians = (motion_scale * position - 240.0f) * 0.0004f;
    xyz[0] = (int16_t)(16384.0f * 0.8f * cosf(radians));
    xyz[1] = (int16_t)(16384.0f * 0.6f * cosf(radians));
    xyz[2] = (int16_t)(16384.0f * imu_sign * sinf(radians));
    xyz[2] += now_ms % 24u < 12u ? imu_noise : -imu_noise;
    return 1;
}

static void check(int condition, unsigned bit)
{
    ++result[1];
    if (!condition) {
        result[0] |= 1u << bit;
        if (result[3] == 0) { result[3] = result[1]; }
    }
}

static uint8_t feed(LinkTest *link, const uint8_t *bytes, size_t length)
{
    uint8_t command = 0;
    for (size_t i = 0; i < length; ++i) {
        uint8_t reply = link_test_feed(link, bytes[i], ++now_ms);
        if (reply != 0u) { command = reply; }
    }
    return command;
}

static void run_ms(LinkTest *link, unsigned duration)
{
    for (unsigned i = 0; i < duration; ++i) {
        ++now_ms;
        startup_level_tick(link, now_ms, 0);
    }
    startup_level_menu(link);
}

static void restart(LinkTest *link, int at_position)
{
    position = at_position;
    pulse_count = up_pulses = 0;
    furthest_position = at_position;
    link_test_init(link);
    startup_level_init();
}

static void test_startup(void)
{
    LinkTest link;
    pulse_fails = false;
    restart(&link, 0);
    startup_level_tick(&link, now_ms, 0x40u);
    startup_level_tick(&link, now_ms, LINK_JOG_FORWARD);
    check(pulse_count == 0, 24);
    imu_noise = 320;
    run_ms(&link, 500);
    check(pulse_count == 0, 24);
    imu_noise = 0;
    run_ms(&link, 450);
    check(pulse_count == 0, 24);
    run_ms(&link, 4400);
    check(strcmp(link.startup_rows[0], "HOME: SWEEP UP") == 0
          && position > 240 && furthest_position < 640, 24);
    run_ms(&link, 8500);
    check(strcmp(link.startup_rows[0], "HOME: READY") == 0
          && position >= 236 && position <= 244 && position == link.steps, 26);
    check(furthest_position == 640 && up_pulses == 640, 26);
    const unsigned ready_pulses = pulse_count;
    run_ms(&link, 2400);
    check(pulse_count == ready_pulses, 26);

    startup_level_tick(&link, now_ms, LINK_JOG_FORWARD);
    run_ms(&link, 450);
    check(pulse_count == ready_pulses + 1
          && strcmp(link.startup_rows[0], "HOME: MANUAL") == 0, 27);

    imu_sign = -1.0f;
    imu_noise = 65;
    restart(&link, 0);
    run_ms(&link, 13000);
    check(strcmp(link.startup_rows[0], "HOME: READY") == 0
          && position >= 236 && position <= 244 && up_pulses == 640, 24);
    imu_sign = 1.0f;
    imu_noise = 0;

    restart(&link, 240);
    run_ms(&link, 1100);
    check(strcmp(link.startup_rows[1], "PLACE AT LOW") == 0 && pulse_count == 0, 25);

    motor_stalled = true;
    restart(&link, 0);
    run_ms(&link, 2800);
    check(strcmp(link.startup_rows[1], "NO PROGRESS") == 0
          && pulse_count > 0 && pulse_count <= 52, 28);
    motor_stalled = false;
    motor_sign = -1;
    restart(&link, 0);
    run_ms(&link, 2800);
    check(strcmp(link.startup_rows[1], "NO PROGRESS") == 0 && pulse_count < 52, 28);
    motor_sign = 1;

    restart(&link, 0);
    run_ms(&link, 1150);
    imu_offline = true;
    run_ms(&link, 36);
    check(strcmp(link.startup_rows[1], "IMU DATA") == 0 && pulse_count > 0, 29);
    imu_offline = false;
    imu_stale = true;
    restart(&link, 0);
    run_ms(&link, 1100);
    check(strcmp(link.startup_rows[1], "IMU DATA") == 0 && pulse_count == 0, 29);
    imu_stale = false;

    restart(&link, 0);
    run_ms(&link, 1150);
    const unsigned before_stop = pulse_count;
    startup_level_tick(&link, now_ms, 0x43);
    run_ms(&link, 3600);
    check(strcmp(link.startup_rows[1], "STOPPED") == 0 && pulse_count == before_stop, 30);

    motion_scale = 0.30f;
    restart(&link, 0);
    run_ms(&link, 12000);
    check(strcmp(link.startup_rows[1], "NO LEVEL IN RANGE") == 0
          && position == 640 && pulse_count == 640, 27);
    motion_scale = 1.0f;

    pulse_fails = true;
    restart(&link, 0);
    run_ms(&link, 1100);
    check(strcmp(link.startup_rows[1], "STEP PULSE") == 0
          && link.steps == 0 && pulse_count == 1, 31);
    pulse_fails = false;
}

static void test_pulse_cycle(void)
{
    LinkTest link;
    link_test_init(&link);
    pulse_count = up_pulses = 0;
    position = furthest_position = 0;
    imu_offline = true;
    now_ms = UINT32_MAX - 6u;
    pulse_cycle_init(now_ms);
    uint32_t leg_started = now_ms;
    uint32_t previous_return_duration = 0;
    for (unsigned elapsed = 1; elapsed <= 12; ++elapsed) {
        pulse_cycle_tick(&link, ++now_ms, 0);
        check(pulse_count == (elapsed == 12), 24);
    }

    for (unsigned cycle = 0; cycle < 2; ++cycle) {
        if (cycle != 0) {
            for (unsigned tick = 0; tick < 12; ++tick) { pulse_cycle_tick(&link, ++now_ms, 0); }
            check(link.steps == 1 && pulse_direction, 24);
        }
        for (unsigned tick = 0; tick < 8000 && link.steps < 650; ++tick) {
            pulse_cycle_tick(&link, ++now_ms, 0);
        }
        const uint32_t forward_duration = now_ms - leg_started;
        leg_started = now_ms;
        check(cycle == 0 || forward_duration != previous_return_duration, 24);
        check(link.steps == 650 && up_pulses == (cycle + 1u) * 650u, 24);
        pulse_cycle_menu(&link);
        check(strcmp(link.startup_rows[0], "MOTOR:REV") == 0, 24);
        const unsigned at_top = pulse_count;
        for (unsigned tick = 0; tick < 11; ++tick) { pulse_cycle_tick(&link, ++now_ms, 0); }
        check(pulse_count == at_top, 24);
        pulse_cycle_tick(&link, ++now_ms, 0);
        check(link.steps == 649 && pulse_count == at_top + 1 && !pulse_direction, 24);

        for (unsigned tick = 0; tick < 8000 && link.steps > 0; ++tick) {
            pulse_cycle_tick(&link, ++now_ms, 0);
        }
        check(link.steps == 0 && position == 0 && furthest_position == 650
              && pulse_count == (cycle + 1u) * 1300u, 24);
        previous_return_duration = now_ms - leg_started;
        leg_started = now_ms;
        check(previous_return_duration != forward_duration, 24);
        pulse_cycle_menu(&link);
        check(strcmp(link.startup_rows[0], "MOTOR:FWD") == 0, 24);
    }
    pulse_cycle_tick(&link, ++now_ms, LINK_KEY_BACK);
    for (unsigned tick = 0; tick < 500; ++tick) { pulse_cycle_tick(&link, ++now_ms, 0); }
    pulse_cycle_menu(&link);
    check(pulse_count == 2600 && strcmp(link.startup_rows[0], "MOTOR:STOP") == 0, 24);
    link_test_menu(&link, now_ms, 0, (uint8_t *)0x2001E300u);

    pulse_fails = true;
    link_test_init(&link);
    pulse_cycle_init(now_ms);
    pulse_cycle_tick(&link, now_ms + 12, 0);
    pulse_cycle_tick(&link, now_ms + 1000, 0);
    pulse_cycle_menu(&link);
    check(pulse_count == 2601 && link.steps == 0 && link.step_fault
          && strcmp(link.startup_rows[0], "MOTOR:ERR") == 0
          && strcmp(link.startup_rows[1], "STEP PULSE") == 0, 24);
    pulse_fails = imu_offline = false;
}

static void feed_center(LinkTest *link, const uint8_t *source, uint32_t sample_ms)
{
    uint8_t frame[sizeof(fixture_center_left)];
    memcpy(frame, source, sizeof(frame));
    for (unsigned i = 0; i < 4; ++i) {
        frame[6 + i] = (uint8_t)(sample_ms >> (8 * i));
    }
    const uint16_t crc = link_crc16(frame + 2, sizeof(frame) - 4);
    frame[16] = (uint8_t)crc;
    frame[17] = (uint8_t)(crc >> 8);
    feed(link, frame, sizeof(frame));
    center_balance_tick(link, now_ms);
}

static void park_center_startup(LinkTest *link)
{
    now_ms = 0;
    pulse_count = up_pulses = 0;
    pulse_fails = false;
    link_test_init(link);
    center_balance_init();
    for (unsigned i = 0; i < 9000 && center_balance_level_steps() < 0; ++i) {
        center_balance_tick(link, ++now_ms);
    }
}
static void finish_center_startup(LinkTest *link)
{
    park_center_startup(link);
    center_balance_button(link, BOARD_SW3, now_ms);
}

static void test_center_startup(void)
{
    LinkTest link;
    park_center_startup(&link);
    center_balance_menu(&link, now_ms);
    check(link.steps == 310 && pulse_count == 990 && up_pulses == 650, 24);
    check(strcmp(link.startup_rows[0], "BAL: CONFIRM MODE") == 0, 24);
    check(strstr(link.startup_rows[1], "DIR") == 0, 24);
    const unsigned done = pulse_count;
    feed_center(&link, fixture_center_right, 100);
    for (unsigned i = 0; i < 250; ++i) { center_balance_tick(&link, ++now_ms); }
    check(pulse_count == done && !center_balance_mode_confirmed(), 24);
    center_balance_button(&link, BOARD_SW2, now_ms);
    check(center_balance_selected_mode() == BALANCE_MODE_RESERVED, 24);
    center_balance_button(&link, BOARD_SW1, now_ms);
    check(center_balance_selected_mode() == BALANCE_MODE_CENTER, 24);
    center_balance_button(&link, BOARD_SW1, now_ms);
    center_balance_button(&link, BOARD_SW3, now_ms);
    check(center_balance_mode_confirmed() && center_balance_selected_mode() == BALANCE_MODE_THREE_POINTS, 24);
    feed_center(&link, fixture_center_right, 200);
    check(pulse_count == done && center_balance_target_steps() == 310, 24);
    center_balance_button(&link, BOARD_SW1, now_ms);
    center_balance_button(&link, BOARD_SW3, now_ms);
    feed_center(&link, fixture_center_left, 300);
    check(center_balance_mode_confirmed() && center_balance_selected_mode() == BALANCE_MODE_DISTURBANCE
          && pulse_count == done, 24);
    center_balance_button(&link, BOARD_SW2, now_ms);
    center_balance_button(&link, BOARD_SW2, now_ms);
    center_balance_button(&link, BOARD_SW4, now_ms);
    feed_center(&link, fixture_center_right, 400);
    check(!center_balance_mode_confirmed() && pulse_count == done, 24);
    center_balance_button(&link, BOARD_SW3, now_ms);
    center_balance_tick(&link, ++now_ms);
    check(center_balance_state(&link, now_ms) == 1 && pulse_count == done, 24);
    feed_center(&link, fixture_center_right, 500);
    check(center_balance_state(&link, now_ms) == 2 && center_balance_target_steps() < 310, 24);
    center_balance_button(&link, BOARD_SW1, now_ms);
    center_balance_button(&link, BOARD_SW3, now_ms);
    check(center_balance_target_steps() == link.steps && center_balance_state(&link, now_ms) == 4, 24);
    now_ms = 0;
    link_test_init(&link);
    center_balance_init();
    center_balance_tick(&link, 20000);
    center_balance_menu(&link, 20000);
    check(strcmp(link.startup_rows[0], "BAL: S5 FAULT") == 0, 24);
    check(strcmp(link.startup_rows[1], "STARTUP TIMEOUT") == 0, 24);
}
static void test_center_balance(void)
{
    LinkTest link;
    finish_center_startup(&link);
    pulse_count = 0;
    feed_center(&link, fixture_pixel_only, 100);
    center_balance_menu(&link, now_ms);
    check(!link.center_valid && pulse_count == 0 && strcmp(link.startup_rows[0], "BAL: S1 WAIT BALL") == 0, 24);
    feed_center(&link, fixture_center_right, 200);
    center_balance_menu(&link, now_ms);
    link_test_menu(&link, now_ms, 0, (uint8_t *)0x2001E380u);
    check(link.center_error_mm == 30 && link.center_velocity_mm_s == 0, 24);
    check(strcmp(link.startup_rows[0], "BAL: S2 ZERO") == 0, 24);
    for (uint32_t t = 233; t < 800; t += 33) {
        feed_center(&link, fixture_center_right, t);
        for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
    }
    check(link.steps < 310 && pulse_count > 0 && !pulse_direction, 24);
    feed_center(&link, fixture_center_near_left, 850);
    for (uint32_t t = 883; t < 2400; t += 33) {
        feed_center(&link, fixture_center_near_right, t);
        for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
    }
    center_balance_menu(&link, now_ms);
    check(strcmp(link.startup_rows[0], "BAL: S3 HOLD ZERO") == 0, 24);
    const int frozen = link.steps;
    const unsigned pulses = pulse_count;
    feed_center(&link, fixture_center_near_left, 2400);
    for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
    check(frozen != 310 && link.steps == frozen && pulse_count == pulses, 24);
    feed_center(&link, fixture_center_fast, 2450);
    for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
    check(link.steps < frozen && !pulse_direction && link.center_velocity_mm_s == 40, 24);
    feed_center(&link, fixture_center_lost, 2500);
    const unsigned lost_pulses = pulse_count;
    for (unsigned i = 0; i < 250; ++i) { center_balance_tick(&link, ++now_ms); }
    center_balance_menu(&link, now_ms);
    check(pulse_count == lost_pulses && strcmp(link.startup_rows[0], "BAL: S1 WAIT BALL") == 0, 24);
    feed_center(&link, fixture_center_left, 2600);
    for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
    check(pulse_direction && pulse_count > lost_pulses, 24);
    const uint32_t rx = link.center_rx_ms;
    for (unsigned i = 0; i < 15; ++i) { feed_center(&link, fixture_center_left, 2600); }
    check(link.center_rx_ms == rx, 24);
    center_balance_menu(&link, now_ms);
    check(strcmp(link.startup_rows[0], "BAL: S1 WAIT BALL") == 0, 24);
    center_balance_key(&link, LINK_KEY_BACK, now_ms);
    const unsigned stopped = pulse_count;
    feed_center(&link, fixture_center_right, 2800);
    center_balance_menu(&link, now_ms);
    check(pulse_count == stopped && strcmp(link.startup_rows[0], "BAL: S4 STOPPED") == 0, 24);
    center_balance_key(&link, LINK_KEY_OK, now_ms);
    feed_center(&link, fixture_center_right, 2900);
    center_balance_menu(&link, now_ms);
    check(strcmp(link.startup_rows[0], "BAL: S2 ZERO") == 0, 24);
    feed_center(&link, fixture_center_edge, 2950);
    center_balance_menu(&link, now_ms);
    check(strcmp(link.startup_rows[0], "BAL: S2 ZERO") == 0, 24);
    const unsigned edge = pulse_count;
    for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
    check(pulse_count > edge && !pulse_direction, 24);
    feed_center(&link, fixture_center_near_left, 3000);
    center_balance_menu(&link, now_ms);
    check(strcmp(link.startup_rows[0], "BAL: S2 ZERO") == 0, 24);
    feed_center(&link, fixture_center_edge_left, 3100);
    center_balance_menu(&link, now_ms);
    check(strcmp(link.startup_rows[0], "BAL: S2 ZERO") == 0, 24);
    const unsigned negative_edge = pulse_count;
    for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
    check(pulse_count > negative_edge && pulse_direction, 24);
    finish_center_startup(&link);
    pulse_fails = true;
    feed_center(&link, fixture_center_right, 3000);
    for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
    center_balance_menu(&link, now_ms);
    check(link.step_fault && strcmp(link.startup_rows[0], "BAL: S5 FAULT") == 0, 24);
    check(strcmp(link.startup_rows[1], "STEP PULSE FAIL") == 0, 24);
    pulse_fails = false;
    for (unsigned end = 0; end < 2; ++end) {
        now_ms = 0;
        link_test_init(&link);
        center_balance_init();
        link.steps = end ? 640 : 10;
        center_balance_key(&link, LINK_KEY_BACK, now_ms);
        center_balance_key(&link, LINK_KEY_OK, now_ms);
        center_balance_button(&link, BOARD_SW3, now_ms);
        pulse_count = 0;
        for (uint32_t t = 4000; t < 4800; t += 33) {
            feed_center(&link, end ? fixture_center_left : fixture_center_right, t);
            for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
            check(link.steps >= 0 && link.steps <= 650, 24);
        }
        check(link.steps == (end ? 650 : 0) && pulse_count == 10, 24);
    }
    check(g_balance_tuning.magic == 0x42414C31u && g_balance_tuning.version == 1, 24);
}


static float tuning_channel(unsigned index)
{
    float value;
    memcpy(&value, tuning_tx + index * 4, 4);
    return value;
}
static void tuning_enqueue(const char *text)
{
    tuning_head = (unsigned)strlen(text);
    tuning_tail = 0;
    memcpy(tuning_rx, text, tuning_head);
}
static void tuning_command(LinkTest *link, const char *text)
{
    tuning_enqueue(text);
    while (tuning_tail < tuning_head) { vofa_tuning_tick(link, ++now_ms); }
    now_ms += 50;
    vofa_tuning_tick(link, now_ms);
}
static void observe_ball(LinkTest *link, int16_t x, int16_t velocity)
{
    link->center_error_mm = x;
    link->center_velocity_mm_s = velocity;
    link->center_valid = true;
    link->center_rx_ms = now_ms;
    link->center_sample_ms += 33;
    ++link->center_observations;
    center_balance_tick(link, now_ms);
    for (unsigned i = 0; i < 33; ++i) { center_balance_tick(link, ++now_ms); }
}

static void test_three_point_mode(void)
{
    LinkTest link;
    park_center_startup(&link);
    center_balance_button(&link, BOARD_SW1, now_ms);
    center_balance_button(&link, BOARD_SW3, now_ms);
    observe_ball(&link, 6, 0);
    center_balance_button(&link, BOARD_SW4, now_ms);
    observe_ball(&link, 0, 0);
    check(center_balance_target_steps() == 310, 27);
    center_balance_button(&link, BOARD_SW4, now_ms);
    observe_ball(&link, 0, 0);
    check(center_balance_target_steps() < 310, 27);
    check(!strcmp(center_balance_three_point_status(&link, now_ms), "P1 MOVE"), 27);
    center_balance_button(&link, BOARD_SW1, now_ms);
    center_balance_button(&link, BOARD_SW4, now_ms);
    check(center_balance_selected_mode() == BALANCE_MODE_THREE_POINTS, 27);

    observe_ball(&link, -42, -12); /* Exactly 8 mm from P1, even while crossing. */
    check(!strcmp(center_balance_three_point_status(&link, now_ms), "P1 HOLD"), 27);
    for (unsigned i = 0; i < 20; ++i) { observe_ball(&link, -50, 0); }
    observe_ball(&link, -41, 0);
    check(!strcmp(center_balance_three_point_status(&link, now_ms), "P1 MOVE"), 27);
    observe_ball(&link, -50, 0);
    for (unsigned i = 0; i < 45; ++i) { observe_ball(&link, -50, 0); }
    check(!strcmp(center_balance_three_point_status(&link, now_ms), "P1 HOLD"), 27);
    observe_ball(&link, -50, 0);
    check(!strcmp(center_balance_three_point_status(&link, now_ms), "P2 MOVE"), 27);

    for (unsigned i = 0; i < 47; ++i) { observe_ball(&link, 50, 0); }
    check(!strcmp(center_balance_three_point_status(&link, now_ms), "O MOVE"), 27);
    for (unsigned i = 0; i < 47; ++i) { observe_ball(&link, 0, 0); }
    check(!strcmp(center_balance_three_point_status(&link, now_ms), "O DONE"), 27);

    /* A lost observation invalidates a hold and does not pause the deadline. */
    const uint32_t pressed_ms = now_ms - 20;
    center_balance_button(&link, BOARD_SW4, pressed_ms);
    observe_ball(&link, -50, 0);
    check(!strcmp(center_balance_three_point_status(&link, now_ms), "P1 HOLD"), 27);
    link.center_valid = false;
    center_balance_tick(&link, ++now_ms);
    check(!strcmp(center_balance_three_point_status(&link, now_ms), "P1 LOST"), 27);
    now_ms = pressed_ms + 7001;
    center_balance_tick(&link, now_ms);
    check(!strcmp(center_balance_three_point_status(&link, now_ms), "P1 TIMEOUT")
          && center_balance_target_steps() == link.steps, 27);
}

static void test_center_stall_integral(void)
{
    const BalanceTuning defaults = g_balance_tuning;
    LinkTest link;
    g_balance_tuning.kp = g_balance_tuning.kv = 0;
    g_balance_tuning.ki = g_balance_tuning.gain = 1;
    g_balance_tuning.max_offset = 100;
    for (unsigned side = 0; side < 2; ++side) {
        const int x = side ? -30 : 30;
        finish_center_startup(&link);
        for (unsigned i = 0; i < 8; ++i) { observe_ball(&link, x, 0); }
        check(center_balance_target_steps() == 310, 24);
        observe_ball(&link, x, side ? 10 : -10);
        for (unsigned i = 0; i < 8; ++i) { observe_ball(&link, x, 0); }
        check(center_balance_target_steps() == 310, 24);
        for (unsigned i = 0; i < 30; ++i) { observe_ball(&link, x, 0); }
        const int ramped = center_balance_target_steps();
        check(side ? ramped > 315 : ramped < 305, 24);
        for (unsigned i = 0; i < 30; ++i) { observe_ball(&link, x, side ? 10 : -10); }
        check(center_balance_target_steps() == ramped, 24);
        observe_ball(&link, 0, side ? 40 : -40);
        const int braking = center_balance_target_steps();
        for (unsigned i = 0; i < 8; ++i) { observe_ball(&link, 0, side ? 40 : -40); }
        check(center_balance_target_steps() == braking, 24);
        const int held = link.steps;
        for (unsigned i = 0; i < 12; ++i) { observe_ball(&link, 0, 0); }
        check(held != 310 && link.steps == held && center_balance_state(&link, now_ms) == 3, 24);
        link.center_valid = false;
        center_balance_tick(&link, ++now_ms);
        for (unsigned i = 0; i < 8; ++i) { observe_ball(&link, x, 0); }
        check(center_balance_target_steps() == held, 24);
    }
    int at_thirty = 0;
    for (unsigned far = 0; far < 2; ++far) {
        finish_center_startup(&link);
        for (unsigned i = 0; i < 40; ++i) { observe_ball(&link, far ? 120 : 30, 0); }
        if (!far) { at_thirty = center_balance_target_steps(); }
        check(center_balance_target_steps() == at_thirty && center_balance_state(&link, now_ms) == 2, 24);
    }
    const int learned = center_balance_target_steps();
    g_balance_tuning.ki = 0;
    for (unsigned i = 0; i < 40; ++i) { observe_ball(&link, 30, 0); }
    check(center_balance_target_steps() == learned, 24);
    g_balance_tuning.gain = 2;
    for (unsigned i = 0; i < 8; ++i) { observe_ball(&link, 30, 0); }
    check(center_balance_target_steps() == learned, 24);
    g_balance_tuning.ki = 1;
    g_balance_tuning.gain = 0;
    finish_center_startup(&link);
    for (unsigned i = 0; i < 40; ++i) { observe_ball(&link, 30, 0); }
    check(center_balance_target_steps() == 310, 24);
    g_balance_tuning.gain = 1;
    for (unsigned limit = 0; limit < 3; ++limit) {
        g_balance_tuning.kp = 2;
        g_balance_tuning.max_offset = limit ? 100 : 20;
        finish_center_startup(&link);
        const int anchor = limit == 1 ? 10 : (limit == 2 ? 640 : 310);
        /* Establish a new starting pose explicitly; STOP must not recalibrate it. */
        center_balance_init();
        link.steps = anchor;
        center_balance_key(&link, LINK_KEY_BACK, now_ms);
        center_balance_key(&link, LINK_KEY_OK, now_ms);
        center_balance_button(&link, BOARD_SW3, now_ms);
        for (unsigned i = 0; i < 60; ++i) { observe_ball(&link, limit == 2 ? -30 : 30, 0); }
        check(center_balance_target_steps() == (limit == 1 ? 0 : (limit == 2 ? 650 : 290)), 24);
        g_balance_tuning.kp = 0;
        /* Allow the existing per-observation target slew to finish. */
        for (unsigned i = 0; i < 2; ++i) { observe_ball(&link, limit == 2 ? -30 : 30, 0); }
        check(abs(center_balance_target_steps() - anchor) <= 1, 24);
    }
    g_balance_tuning = defaults;
}

static void test_center_bias_continuity(void)
{
    const BalanceTuning defaults = g_balance_tuning;
    LinkTest link;
    g_balance_tuning.kp = g_balance_tuning.kv = 0;
    g_balance_tuning.ki = g_balance_tuning.gain = 1;
    g_balance_tuning.max_offset = 100;
    finish_center_startup(&link);
    for (unsigned i = 0; i < 100; ++i) { observe_ball(&link, 30, 0); }
    const int learned = center_balance_target_steps();
    check(learned < 285, 25);
    observe_ball(&link, 0, -40);
    observe_ball(&link, -10, -40);
    check(center_balance_target_steps() == learned, 25);
    for (unsigned i = 0; i < 10; ++i) { observe_ball(&link, -30, 10); }
    check(center_balance_target_steps() == learned, 25);
    link.center_valid = false;
    center_balance_tick(&link, ++now_ms);
    check(center_balance_target_steps() == link.steps && center_balance_state(&link, now_ms) == 1, 25);
    observe_ball(&link, -30, 10);
    check(center_balance_target_steps() == learned, 25);
    for (unsigned i = 0; i < 200; ++i) { center_balance_tick(&link, ++now_ms); }
    check(center_balance_state(&link, now_ms) == 1, 25);
    observe_ball(&link, -30, 10);
    check(center_balance_target_steps() == learned, 25);
    center_balance_key(&link, LINK_KEY_BACK, now_ms);
    const unsigned stopped = pulse_count;
    for (unsigned i = 0; i < 12; ++i) { observe_ball(&link, -30, 10); }
    check(pulse_count == stopped && center_balance_state(&link, now_ms) == 4, 25);
    center_balance_key(&link, LINK_KEY_OK, now_ms);
    observe_ball(&link, -30, 10);
    check(center_balance_target_steps() == learned, 25);
    for (unsigned i = 0; i < 40; ++i) { observe_ball(&link, -30, 0); }
    check(center_balance_target_steps() > learned + 8, 25);
    const int before_hold = center_balance_target_steps();
    for (unsigned i = 0; i < 20; ++i) {
        observe_ball(&link, 0, 0);
        check(center_balance_target_steps() == before_hold, 25);
    }
    check(center_balance_state(&link, now_ms) == 3 && link.steps == before_hold, 25);
    g_balance_tuning.ki = 0;
    observe_ball(&link, -30, 10);
    check(center_balance_target_steps() == before_hold && center_balance_state(&link, now_ms) == 2, 25);
    g_balance_tuning = defaults;
}

static void test_center_saturation_recovery(void)
{
    const BalanceTuning defaults = g_balance_tuning;
    LinkTest link;
    g_balance_tuning.kp = g_balance_tuning.kv = 0;
    g_balance_tuning.gain = g_balance_tuning.ki = 1;
    for (unsigned limit = 0; limit < 3; ++limit) {
        const int anchor = limit == 1 ? 10 : (limit == 2 ? 640 : 310);
        /* Manual setup establishes only an arbitrary starting pose. */
        link_test_init(&link);
        center_balance_init();
        link.steps = anchor;
        center_balance_key(&link, LINK_KEY_BACK, now_ms);
        center_balance_key(&link, LINK_KEY_OK, now_ms);
        center_balance_button(&link, BOARD_SW3, now_ms);
        g_balance_tuning.max_offset = limit ? 100 : 20;
        const int x = limit == 2 ? -30 : 30;
        for (unsigned i = 0; i < 120; ++i) { observe_ball(&link, x, 0); }
        const int saturated = center_balance_target_steps();
        check(saturated == (limit == 1 ? 0 : (limit == 2 ? 650 : 290)), 25);
        for (unsigned i = 0; i < 80; ++i) { observe_ball(&link, x, 0); }
        check(center_balance_target_steps() == saturated, 25);
        for (unsigned i = 0; i < 12; ++i) {
            const int previous = center_balance_target_steps();
            observe_ball(&link, -x, 0);
            check(abs(center_balance_target_steps() - previous) <= 12, 25);
        }
        check(x > 0 ? center_balance_target_steps() > saturated + 2
                    : center_balance_target_steps() < saturated - 2, 25);
    }
    g_balance_tuning = defaults;
}

static void test_center_feedback_pause(void)
{
    const BalanceTuning defaults = g_balance_tuning;
    LinkTest link;
    g_balance_tuning.kp = g_balance_tuning.ki = g_balance_tuning.gain = 1;
    g_balance_tuning.kv = 0;
    g_balance_tuning.max_offset = 150;
    finish_center_startup(&link);
    for (unsigned i = 0; i < 100; ++i) { observe_ball(&link, 30, 0); }
    g_balance_tuning.ki = 0;
    const int compensated = center_balance_target_steps();
    check(compensated < 255, 25);
    link.center_valid = false;
    center_balance_tick(&link, ++now_ms);
    observe_ball(&link, 30, 0);
    check(center_balance_target_steps() == compensated, 25);
    center_balance_key(&link, LINK_KEY_BACK, now_ms);
    center_balance_key(&link, LINK_KEY_OK, now_ms);
    observe_ball(&link, 30, 0);
    check(center_balance_target_steps() == compensated, 25);
    for (unsigned i = 0; i < 200; ++i) { center_balance_tick(&link, ++now_ms); }
    observe_ball(&link, 30, 0);
    check(center_balance_target_steps() == compensated, 25);
    g_balance_tuning = defaults;
}

static void test_center_braking(void)
{
    const BalanceTuning defaults = g_balance_tuning;
    LinkTest link;
    int targets[2];
    g_balance_tuning.kp = g_balance_tuning.ki = 0;
    g_balance_tuning.kv = g_balance_tuning.gain = 1;
    for (unsigned i = 0; i < 2; ++i) {
        g_balance_tuning.brake = i ? 2 : 1;
        finish_center_startup(&link);
        for (unsigned t = 0; t < 20; ++t) { observe_ball(&link, 10, -50); }
        targets[i] = center_balance_target_steps();
    }
    check(targets[0] > 310 && targets[1] > targets[0], 25);
    g_balance_tuning = defaults;
}

typedef struct {
    float x, velocity, neutral, response, x_history[3], fit_x[5];
    uint32_t fit_ms[5];
    unsigned history, fit_count;
} BallPlant;

/* Same short-window regression as K230, in addition to the image delay. */
static float plant_velocity(BallPlant *plant, float x)
{
    if (plant->fit_count == 5) {
        for (unsigned i = 0; i < 4; ++i) {
            plant->fit_x[i] = plant->fit_x[i + 1];
            plant->fit_ms[i] = plant->fit_ms[i + 1];
        }
        --plant->fit_count;
    }
    const unsigned last = plant->fit_count++;
    plant->fit_x[last] = x;
    plant->fit_ms[last] = now_ms;
    while (plant->fit_count > 2 && now_ms - plant->fit_ms[0] > 120) {
        for (unsigned i = 0; i + 1 < plant->fit_count; ++i) {
            plant->fit_x[i] = plant->fit_x[i + 1];
            plant->fit_ms[i] = plant->fit_ms[i + 1];
        }
        --plant->fit_count;
    }
    float mean_t = 0, mean_x = 0, numerator = 0, denominator = 0;
    for (unsigned i = 0; i < plant->fit_count; ++i) {
        mean_t += (int32_t)(plant->fit_ms[i] - now_ms);
        mean_x += plant->fit_x[i];
    }
    mean_t /= plant->fit_count;
    mean_x /= plant->fit_count;
    for (unsigned i = 0; i < plant->fit_count; ++i) {
        const float t = (int32_t)(plant->fit_ms[i] - now_ms) - mean_t;
        numerator += t * (plant->fit_x[i] - mean_x);
        denominator += t * t;
    }
    return denominator > 0 ? 1000 * numerator / denominator : 0;
}

static void plant_observe(LinkTest *link, BallPlant *plant)
{
    const unsigned i = plant->history;
    plant->x_history[i] = plant->x;
    plant->history = (i + 1) % 3;
    link->center_error_mm = (int16_t)lroundf(plant->x_history[plant->history]);
    link->center_velocity_mm_s = (int16_t)lroundf(plant_velocity(plant, plant->x_history[plant->history]));
    link->center_valid = true;
    link->center_sample_ms = link->center_rx_ms = now_ms;
    ++link->center_observations;
}

static void plant_integrate(const LinkTest *link, BallPlant *plant)
{
    const float drive = plant->response * (link->steps - plant->neutral);
    float acceleration = drive - 0.6f * plant->velocity;
    if (fabsf(plant->velocity) < 0.05f && fabsf(drive) <= 1.0f) {
        plant->velocity = acceleration = 0;
    } else {
        const float friction_sign = fabsf(plant->velocity) < 0.05f ? drive : plant->velocity;
        acceleration -= friction_sign > 0 ? 1.0f : -1.0f;
    }
    plant->velocity += acceleration * 0.001f;
    plant->x += plant->velocity * 0.001f;
}

/* Ideal pulse tracking plus gravity, drag, dry friction and 66 ms image delay. */
static void run_plant(LinkTest *link, BallPlant *plant, unsigned duration, unsigned report)
{
    float peak = fabsf(plant->x), tail_square = 0, tail_speed = 0;
    unsigned tail_count = 0, holds = 0;
    bool valid_travel = true, valid_slew = true, saturated = false;
    for (unsigned ms = 0; ms < duration; ++ms) {
        ++now_ms;
        if (ms % 33 == 0) { plant_observe(link, plant); }
        const int before = center_balance_target_steps();
        center_balance_tick(link, now_ms);
        if (report == 12 && ms < 1000) { saturated |= center_balance_target_steps() <= 270; }
        valid_slew &= abs(center_balance_target_steps() - before) <= 12;
        valid_travel &= link->steps >= 0 && link->steps <= 650;
        plant_integrate(link, plant);
        peak = fmaxf(peak, fabsf(plant->x));
        if (ms >= duration - 5000) {
            tail_square += plant->x * plant->x;
            tail_speed += fabsf(plant->velocity);
            ++tail_count;
            holds += center_balance_state(link, now_ms) == 3;
        }
    }
    volatile float *out = (volatile float *)(0x2001E800u + report * 32u);
    out[0] = plant->x; out[1] = plant->velocity; out[2] = link->steps;
    out[3] = peak; out[4] = sqrtf(tail_square / tail_count);
    out[5] = tail_speed / tail_count; out[6] = (float)holds / tail_count;
    out[7] = plant->neutral;
    check(valid_travel && valid_slew, 26);
    check(out[4] <= 4.0f && out[5] <= 4.0f && out[6] > 0.5f, 26);
    check(peak < 100.0f && fabsf(link->steps - plant->neutral) < 12.0f, 26);
    if (report == 12) { check(saturated, 26); }
}

static void test_adaptive_plant(void)
{
    const BalanceTuning defaults = g_balance_tuning;
    for (unsigned scenario = 0; scenario < 6; ++scenario) {
        g_balance_tuning = defaults;
        if (scenario >= 2) {
            g_balance_tuning.ki = 0.3f;
            g_balance_tuning.max_speed = 30;
            g_balance_tuning.accel = 40;
            g_balance_tuning.ref_accel = 60;
        }
        LinkTest link;
        finish_center_startup(&link);
        BallPlant plant = {0};
        plant.x = scenario & 1 ? -40 : 40;
        plant.neutral = scenario & 1 ? 350 : 270;
        plant.response = scenario >= 2 ? 0.4f : 0.8f;
        if (scenario == 4) {
            plant.neutral = 340;
            plant.response = 0.6f;
            g_balance_tuning.max_offset = 40;
        }
        if (scenario == 5) { plant.x = 0; }
        for (unsigned i = 0; i < 3; ++i) { plant.x_history[i] = plant.x; }
        run_plant(&link, &plant, 60000, scenario * 3);
        plant.velocity += scenario & 1 ? -20 : 20;
        run_plant(&link, &plant, 30000, scenario * 3 + 1);
        plant.neutral += scenario & 1 ? 15 : -15;
        run_plant(&link, &plant, 60000, scenario * 3 + 2);
    }
    g_balance_tuning = defaults;
}

static void test_three_point_plant(void)
{
    static const float neutral[] = {310, 270, 350};
    static const float response[] = {0.6f, 0.8f, 0.4f};
    static const char *const hold_labels[] = {"P1 HOLD", "P2 HOLD", "O HOLD"};
    static const char *const arrival_labels[] = {"P2 MOVE", "O MOVE", "O DONE"};
    for (unsigned trial = 0; trial < 3; ++trial) {
        LinkTest link;
        park_center_startup(&link);
        center_balance_button(&link, BOARD_SW1, now_ms);
        center_balance_button(&link, BOARD_SW3, now_ms);
        BallPlant plant = {0};
        plant.neutral = neutral[trial];
        plant.response = response[trial];
        plant_observe(&link, &plant);
        center_balance_button(&link, BOARD_SW4, now_ms);
        unsigned next = 0, move_started = 0, hold_started = 0;
        float max_position = 0;
        volatile float *report = (volatile float *)(0x2001E700u + trial * 32u);
        for (unsigned ms = 1; ms <= 30000; ++ms) {
            ++now_ms;
            if (ms % 33 == 0) { plant_observe(&link, &plant); }
            center_balance_tick(&link, now_ms);
            plant_integrate(&link, &plant);
            max_position = fmaxf(max_position, fabsf(plant.x));
            const char *status = center_balance_three_point_status(&link, now_ms);
            if (strstr(status, "TIMEOUT")) { break; }
            if (!strcmp(status, hold_labels[next])) {
                if (!hold_started) { hold_started = ms; }
            } else if (!strcmp(status, arrival_labels[next]) && hold_started) {
                const unsigned move_ms = hold_started - move_started;
                check(move_ms <= 7000 && ms - hold_started >= 1500, 27);
                report[next++] = move_ms;
                move_started = ms;
                hold_started = 0;
                if (next == 3) { report[3] = ms; break; }
            } else {
                hold_started = 0;
            }
        }
        report[4] = next;
        report[5] = max_position;
        check(next == 3 && max_position < 90, 27);
    }
}

static void test_vofa_tuning(void)
{
    LinkTest link;
    BalanceTuning defaults = g_balance_tuning;
    finish_center_startup(&link);
    vofa_tuning_init();
    tuning_enqueue("k");
    vofa_tuning_tick(&link, ++now_ms);
    tuning_enqueue("p=0.");
    vofa_tuning_tick(&link, ++now_ms);
    check(g_balance_tuning.kp == defaults.kp, 24);
    tuning_command(&link, "6\r\n");
    check(g_balance_tuning.kp == defaults.kp && tuning_channel(6) == -3, 24);
    tuning_command(&link, " KV = 0.9\nki=0.1\n");
    check(g_balance_tuning.kv == defaults.kv && g_balance_tuning.ki == defaults.ki && tuning_channel(6) == -3, 24);
    const char *bad[] = {"kp=nan\n", "kv=inf\n", "kp=-1\n", "max_offset=651\n",
        "gain=1x\n", "magic=0\n", "dir=-1\n", "start\n", "kp=0.9;kv=1\n", "kp=\n"};
    const BalanceTuning valid = g_balance_tuning;
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        tuning_command(&link, bad[i]);
        const BalanceTuning actual = g_balance_tuning;
        check(memcmp(&valid, &actual, sizeof(valid)) == 0 && tuning_channel(6) == -1, 24);
    }
    char overflow[100];
    memset(overflow, 'x', 90);
    memcpy(overflow + 90, "kp=1\n", 6);
    tuning_command(&link, overflow);
    check(g_balance_tuning.kp == valid.kp && tuning_channel(6) == -1, 24);
    tuning_enqueue("kp=");
    vofa_tuning_tick(&link, ++now_ms);
    now_ms += 300;
    vofa_tuning_tick(&link, now_ms);
    tuning_command(&link, "0.8\n");
    check(g_balance_tuning.kp == valid.kp, 24);
    ++tuning_errors;
    tuning_command(&link, "kp=0.8\n");
    check(g_balance_tuning.kp == valid.kp && tuning_channel(6) == -2, 24);
    tuning_command(&link, "kp=0.6\n");
    check(tuning_channel(6) == -3, 24);

    const char *others[] = {"gain=1.2\n", "brake=1.8\n", "max_speed=30\n", "accel=50\n",
        "ref_accel=60\n", "kff=-0.2\n", "kff2=-0.002\n", "max_offset=100\n"};
    for (unsigned i = 0; i < sizeof(others) / sizeof(others[0]); ++i) {
        tuning_command(&link, others[i]);
        check(tuning_channel(6) == -3, 24);
    }
    check(g_balance_tuning.max_speed == 30 && g_balance_tuning.accel == 40 && g_balance_tuning.max_offset == 150, 24);
    check(g_balance_tuning.kff == 0 && g_balance_tuning.kff2 == 0, 24);
    link.center_error_mm = 12;
    link.center_velocity_mm_s = -7;
    link.center_valid = true;
    now_ms += 50;
    link.center_rx_ms = now_ms;
    vofa_tuning_tick(&link, now_ms);
    static const uint8_t tail[] = {0, 0, 0x80, 0x7F};
    check(tuning_tx_size == 76 && memcmp(tuning_tx + 72, tail, 4) == 0, 24);
    check(tuning_channel(0) == 12 && tuning_channel(1) == -7 && tuning_channel(2) == link.steps, 24);
    check(tuning_channel(3) == center_balance_target_steps() && tuning_channel(4) == 1 && tuning_channel(5) == 1, 24);
    memcpy((void *)0x2001E480u, tuning_tx, tuning_tx_size);
    const unsigned sent = tuning_frames;
    vofa_tuning_tick(&link, now_ms + 49);
    check(tuning_frames == sent, 24);
    tuning_tx_full = true;
    vofa_tuning_tick(&link, now_ms + 50);
    check(tuning_frames == sent, 24);
    tuning_tx_full = false;
    vofa_tuning_tick(&link, now_ms + 201);
    check(tuning_frames == sent + 1 && tuning_channel(5) == 0, 24);
    memset(overflow, 'x', 96);
    overflow[96] = 0;
    tuning_enqueue(overflow);
    vofa_tuning_tick(&link, now_ms + 202);
    check(tuning_tail == 32 && tuning_head == 96, 24);
    g_balance_tuning = defaults;
}

int main(void)
{
    LinkTest link;
    uint8_t corrupt[sizeof(fixture_jog_forward)];
    const uint8_t noise[] = {0, 0xA5, 0xA5, 0x5A, 2, 255};
    const uint8_t bad_length[] = {0xA5, 0x5A, 1, 255};
    uint8_t unknown[7];
    result[0] = result[1] = result[2] = result[3] = 0;
    check(link_crc16((const uint8_t *)"123456789", 9) == 0x29B1u, 0);
    link_test_init(&link);
    feed(&link, noise, sizeof(noise));
    feed(&link, fixture_up, sizeof(fixture_up));
    check(link.value == 1 && link.packets == 1, 1);
    link_test_command(0x77, unknown);
    feed(&link, unknown, sizeof(unknown));
    check(link.packets == 1 && link.value == 1, 2);
    memcpy(corrupt, fixture_up, sizeof(corrupt));
    corrupt[6] ^= 1;
    feed(&link, corrupt, sizeof(corrupt));
    feed(&link, fixture_down, sizeof(fixture_down));
    check(link.value == 0 && link.packets == 2 && link.crc_errors != 0, 3);
    feed(&link, fixture_valid, sizeof(fixture_valid));
    check(link.observations == 1 && link.last_status == 1, 4);
    check(link.center_error_mm == 123 && link.center_velocity_mm_s == -45, 4);
    feed(&link, fixture_up, sizeof(fixture_up));
    feed(&link, fixture_back, sizeof(fixture_back));
    check(link.value == 0 && link.last_key == 0x43, 5);
    check(feed(&link, fixture_ok, sizeof(fixture_ok)) == LINK_STOP
          && link.stream_enabled == 0, 6);
    link_test_command(LINK_STOP, (uint8_t *)0x2001E200u);
    check(feed(&link, fixture_ok, sizeof(fixture_ok)) == LINK_START
          && link.stream_enabled == 1, 7);
    link_test_command(LINK_START, (uint8_t *)0x2001E208u);
    link.value = 9999;
    feed(&link, fixture_up, sizeof(fixture_up));
    check(link.value == 9999, 8);
    link.value = -9999;
    feed(&link, fixture_down, sizeof(fixture_down));
    check(link.value == -9999, 9);
    link.value = 0;
    feed(&link, bad_length, sizeof(bad_length));
    feed(&link, fixture_up, sizeof(fixture_up));
    check(link.value == 1, 10);
    feed(&link, fixture_valid, 8);
    now_ms += 100;
    feed(&link, fixture_up, sizeof(fixture_up));
    check(link.value == 2 && link.used == 0, 11);

    link_test_init(&link);
    now_ms = 0xFFFFFFE0u;
    check(feed(&link, fixture_up, sizeof(fixture_up)) == 0u, 15);
    feed(&link, fixture_lost, sizeof(fixture_lost));
    check(link.packets == 2 && link.observations == 1, 12);
    check(feed(&link, fixture_jog_back, sizeof(fixture_jog_back)) == LINK_JOG_BACK, 16);
    link_test_jog(&link, false);
    check(link.steps == -1 && pulse_count == 1 && !pulse_direction, 17);
    for (unsigned i = 0; i < 2; ++i) {
        check(feed(&link, fixture_jog_forward, sizeof(fixture_jog_forward)) == LINK_JOG_FORWARD, 18);
        link_test_jog(&link, true);
    }
    check(link.steps == 1 && pulse_count == 3 && pulse_direction, 19);

    link_test_menu(&link, now_ms + 100u, 0, (uint8_t *)0x2001E100u);
    link_test_menu(&link, now_ms + 1600u, 0, (uint8_t *)0x2001E180u);
    check(strstr((const char *)0x2001E105u, "RX:OK") != 0, 13);
    check(strstr((const char *)0x2001E185u, "RX:LOST") != 0, 14);

    memcpy(corrupt, fixture_jog_forward, sizeof(corrupt));
    corrupt[6] ^= 1;
    check(feed(&link, corrupt, sizeof(corrupt)) == 0u && pulse_count == 3, 20);
    check(feed(&link, fixture_back, sizeof(fixture_back)) == 0u && link.steps == 1
          && link.value == 0 && pulse_count == 3, 21);
    pulse_fails = true;
    link_test_jog(&link, false);
    link_test_jog(&link, true);
    check(link.steps == 1 && pulse_count == 4 && link.step_fault, 22);
    link_test_menu(&link, now_ms, 0, (uint8_t *)0x2001E280u);
    check(strcmp((const char *)(0x2001E280u + 71u), "STEP:+1 ERR") == 0, 23);
    test_startup();
    test_pulse_cycle();
    test_center_startup();
    test_three_point_mode();
    test_center_balance();
    test_center_stall_integral();
    test_center_bias_continuity();
    test_center_saturation_recovery();
    test_center_feedback_pause();
    test_center_braking();
    test_adaptive_plant();
    test_three_point_plant();
    test_vofa_tuning();
    result[2] = 0xF4072300u;
    __asm volatile ("bkpt #0");
    for (;;) { }
}
