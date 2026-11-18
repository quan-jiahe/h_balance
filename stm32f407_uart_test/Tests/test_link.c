#include "link_test.h"
#include "board.h"
#include "fixtures.h"
#include <string.h>

static volatile uint32_t *const result = (volatile uint32_t *)0x2001E000u;
static uint32_t now_ms;
static unsigned pulse_count;
static bool pulse_direction, pulse_fails;

void SystemInit(void) { }

bool board_step_pulse(bool forward)
{
    ++pulse_count;
    pulse_direction = forward;
    return !pulse_fails;
}

static void check(int condition, unsigned bit)
{
    if (!condition) { result[0] |= 1u << bit; }
    ++result[1];
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

int main(void)
{
    LinkTest link;
    uint8_t corrupt[sizeof(fixture_jog_forward)];
    const uint8_t noise[] = {0, 0xA5, 0xA5, 0x5A, 2, 255};
    const uint8_t bad_length[] = {0xA5, 0x5A, 1, 255};
    uint8_t unknown[7];
    result[0] = result[1] = result[2] = 0;
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
    result[2] = 0xF4072300u;
    __asm volatile ("bkpt #0");
    for (;;) { }
}
