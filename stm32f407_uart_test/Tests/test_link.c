#include "link_test.h"
#include "board.h"
#include "center_balance.h"
#include "fixtures.h"
#include "pulse_cycle.h"
#include "startup_level.h"
#include "balance_tuning.h"
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

static void finish_center_startup(LinkTest *link)
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
static void test_center_startup(void)
{
    LinkTest link;
    finish_center_startup(&link);
    center_balance_menu(&link, now_ms);
    check(link.steps == 310 && pulse_count == 990 && up_pulses == 650, 24);
    check(strcmp(link.startup_rows[0], "BAL: S1 WAIT BALL") == 0, 24);
    check(strstr(link.startup_rows[1], "DIR") == 0, 24);
    const unsigned done = pulse_count;
    for (unsigned i = 0; i < 250; ++i) { center_balance_tick(&link, ++now_ms); }
    check(pulse_count == done, 24);
    now_ms = 0;
    link_test_init(&link);
    center_balance_init();
    center_balance_tick(&link, 20000);
    center_balance_menu(&link, 20000);
    check(strcmp(link.startup_rows[0], "BAL: S5 FAULT") == 0, 24);
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
    check(link.steps > 310 && pulse_count > 0 && pulse_direction, 24);
    feed_center(&link, fixture_center_near_left, 850);
    const int frozen = link.steps;
    const unsigned pulses = pulse_count;
    for (uint32_t t = 883; t < 1400; t += 33) {
        feed_center(&link, fixture_center_near_right, t);
        for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
    }
    center_balance_menu(&link, now_ms);
    check(frozen != 310 && link.steps == frozen && pulse_count == pulses, 24);
    check(strcmp(link.startup_rows[0], "BAL: S3 HOLD ZERO") == 0, 24);
    feed_center(&link, fixture_center_fast, 1450);
    for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
    check(link.steps > frozen && pulse_direction && link.center_velocity_mm_s == 40, 24);
    feed_center(&link, fixture_center_lost, 1500);
    const unsigned lost_pulses = pulse_count;
    for (unsigned i = 0; i < 250; ++i) { center_balance_tick(&link, ++now_ms); }
    center_balance_menu(&link, now_ms);
    check(pulse_count == lost_pulses && strcmp(link.startup_rows[0], "BAL: S1 WAIT BALL") == 0, 24);
    feed_center(&link, fixture_center_left, 1600);
    for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
    check(!pulse_direction && pulse_count > lost_pulses, 24);
    const uint32_t rx = link.center_rx_ms;
    for (unsigned i = 0; i < 15; ++i) { feed_center(&link, fixture_center_left, 1600); }
    check(link.center_rx_ms == rx, 24);
    center_balance_menu(&link, now_ms);
    check(strcmp(link.startup_rows[0], "BAL: S1 WAIT BALL") == 0, 24);
    center_balance_key(&link, LINK_KEY_BACK, now_ms);
    const unsigned stopped = pulse_count;
    feed_center(&link, fixture_center_right, 1800);
    center_balance_menu(&link, now_ms);
    check(pulse_count == stopped && strcmp(link.startup_rows[0], "BAL: S4 STOPPED") == 0, 24);
    center_balance_key(&link, LINK_KEY_OK, now_ms);
    feed_center(&link, fixture_center_right, 1900);
    center_balance_menu(&link, now_ms);
    check(strcmp(link.startup_rows[0], "BAL: S2 ZERO") == 0, 24);
    feed_center(&link, fixture_center_edge, 1950);
    const unsigned edge = pulse_count;
    feed_center(&link, fixture_center_near_left, 2000);
    center_balance_menu(&link, now_ms);
    check(pulse_count == edge && strcmp(link.startup_rows[0], "BAL: S5 FAULT") == 0, 24);
    finish_center_startup(&link);
    pulse_fails = true;
    feed_center(&link, fixture_center_right, 3000);
    for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
    center_balance_menu(&link, now_ms);
    check(link.step_fault && strcmp(link.startup_rows[0], "BAL: S5 FAULT") == 0, 24);
    pulse_fails = false;
    for (unsigned end = 0; end < 2; ++end) {
        now_ms = 0;
        link_test_init(&link);
        center_balance_init();
        link.steps = end ? 640 : 10;
        center_balance_key(&link, LINK_KEY_BACK, now_ms);
        center_balance_key(&link, LINK_KEY_OK, now_ms);
        pulse_count = 0;
        for (uint32_t t = 4000; t < 4800; t += 33) {
            feed_center(&link, end ? fixture_center_right : fixture_center_left, t);
            for (unsigned i = 0; i < 20; ++i) { center_balance_tick(&link, ++now_ms); }
            check(link.steps >= 0 && link.steps <= 650, 24);
        }
        check(link.steps == (end ? 650 : 0) && pulse_count == 10, 24);
    }
    check(g_balance_tuning.magic == 0x42414C31u && g_balance_tuning.version == 1, 24);
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
    test_center_balance();
    result[2] = 0xF4072300u;
    __asm volatile ("bkpt #0");
    for (;;) { }
}
