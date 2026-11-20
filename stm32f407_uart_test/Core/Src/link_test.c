#include "link_test.h"
#include "board.h"
#include <stdio.h>
#include <string.h>

uint16_t link_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000u) ? 0x1021u : 0u));
        }
    }
    return crc;
}

void link_test_init(LinkTest *link)
{
    memset(link, 0, sizeof(*link));
    link->stream_enabled = 1;
    link->menu_due = 1;
}

static void discard_byte(LinkTest *link)
{
    --link->used;
    memmove(link->buffer, link->buffer + 1, link->used);
}

uint8_t link_test_feed(LinkTest *link, uint8_t byte, uint32_t now_ms)
{
    uint8_t reply = 0;
    if ((uint32_t)(now_ms - link->last_byte_ms) > 50u) {
        link->used = 0;
    }
    link->last_byte_ms = now_ms;
    link->buffer[link->used++] = byte;
    while (link->used >= 4u) {
        uint8_t *data = link->buffer;
        const bool legacy_frame = data[2] == 1u && (data[3] == 1u || data[3] == 12u);
        const bool center_frame = data[2] == 2u && data[3] == 12u;
        if (data[0] != 0xA5u || data[1] != 0x5Au || (!legacy_frame && !center_frame)) {
            discard_byte(link);
            continue;
        }
        const uint8_t size = (uint8_t)(data[3] + 6u);
        if (link->used < size) {
            break;
        }
        const uint16_t received = (uint16_t)(data[size - 2u]
                                  | ((uint16_t)data[size - 1u] << 8));
        if (received != link_crc16(data + 2, size - 4u)) {
            ++link->crc_errors;
            discard_byte(link);
            continue;
        }
        if (!center_frame && size == 18u) {
            const int16_t position = (int16_t)((uint16_t)data[10] | ((uint16_t)data[11] << 8));
            const int16_t velocity = (int16_t)((uint16_t)data[12] | ((uint16_t)data[13] << 8));
            const uint32_t sample_ms = (uint32_t)data[6] | ((uint32_t)data[7] << 8)
                                     | ((uint32_t)data[8] << 16) | ((uint32_t)data[9] << 24);
            const bool valid = data[15] == 1u && data[14] >= 50u && data[14] <= 100u
                               && position >= -1000 && position <= 1000
                               && velocity >= -2000 && velocity <= 2000;
            const uint32_t elapsed = sample_ms - link->center_sample_ms;
            const bool fresh = link->center_observations == 0
                               || (elapsed > 0 && elapsed < 0x80000000u)
                               || (sample_ms != link->center_sample_ms
                                   && now_ms - link->center_rx_ms > 1000u);
            if (fresh) {
                link->center_error_mm = position;
                link->center_velocity_mm_s = velocity;
                link->center_sample_ms = sample_ms;
                link->center_rx_ms = now_ms;
                link->center_valid = valid;
                ++link->center_observations;
            } else if (!valid) {
                link->center_valid = false;
            }
            ++link->observations;
            link->last_status = data[15];
        } else if (center_frame) {
            /* Pixel v2 must never be interpreted as millimetres. */
            link->center_valid = false;
        } else if (data[4] >= 0x40u && data[4] <= LINK_JOG_BACK) {
            const uint8_t key = data[4];
            link->key_event = key;
            if (key < 0x44u) {
                link->last_key = key;
            }
            switch (key) {
            case 0x40:
                if (link->value < 9999) { ++link->value; }
                break;
            case 0x41:
                if (link->value > -9999) { --link->value; }
                break;
            case 0x42:
                link->stream_enabled ^= 1u;
                reply = link->stream_enabled ? LINK_START : LINK_STOP;
                break;
            case 0x43:
                link->value = 0;
                break;
            case LINK_JOG_FORWARD:
            case LINK_JOG_BACK:
                reply = key;
                break;
            default:
                break;
            }
            link->menu_due = 1;
        } else {
            memmove(data, data + size, link->used - size);
            link->used = (uint8_t)(link->used - size);
            continue;
        }
        ++link->packets;
        link->last_rx_ms = now_ms;
        link->seen_rx = 1;
        memmove(data, data + size, link->used - size);
        link->used = (uint8_t)(link->used - size);
    }
    return reply;
}

void link_test_jog(LinkTest *link, bool forward)
{
    if (link->step_fault) { return; }
    if (forward ? link->steps == INT32_MAX : link->steps == INT32_MIN) { return; }
    if (board_step_pulse(forward)) {
        link->steps += forward ? 1 : -1;
    } else {
        link->step_fault = 1;
    }
}

void link_test_command(uint8_t command, uint8_t packet[7])
{
    packet[0] = 0xA5;
    packet[1] = 0x5A;
    packet[2] = 1;
    packet[3] = 1;
    packet[4] = command;
    const uint16_t crc = link_crc16(packet + 2, 3);
    packet[5] = (uint8_t)crc;
    packet[6] = (uint8_t)(crc >> 8);
}

void link_test_menu(const LinkTest *link, uint32_t now_ms,
                    uint32_t uart_errors, uint8_t packet[LINK_MENU_SIZE])
{
    static const char *const keys[] = {"UP", "DOWN", "OK", "BACK"};
    const char *key = link->last_key >= 0x40u && link->last_key <= 0x43u
                      ? keys[link->last_key - 0x40u] : "NONE";
    const char *rx = !link->seen_rx ? "WAIT"
                   : ((uint32_t)(now_ms - link->last_rx_ms) < 1500u ? "OK" : "LOST");
    const char *observation = link->observations == 0 ? "WAIT"
                   : ((link->last_status & 0x20u) ? "LOST"
                   : ((link->last_status & 3u) ? "VALID" : "INVALID"));
    memset(packet, 0, LINK_MENU_SIZE);
    packet[0] = 0xA5;
    packet[1] = 0x5A;
    packet[2] = 1;
    packet[3] = 89;
    packet[4] = 0x50;
    snprintf((char *)packet + 5, 22, "F407 RX:%s N:%lu", rx,
             (unsigned long)(link->packets % 100000u));
    if (uart_errors || link->crc_errors) {
        snprintf((char *)packet + 5, 22, "RX:%s E:%lu C:%lu", rx,
                 (unsigned long)(uart_errors % 1000u),
                 (unsigned long)(link->crc_errors % 1000u));
    }
    snprintf((char *)packet + 27, 22, "OBS:%lu %s",
             (unsigned long)(link->observations % 100000u), observation);
    snprintf((char *)packet + 49, 22, "KEY:%s V:%ld", key, (long)link->value);
    snprintf((char *)packet + 71, 22, "STEP:%+ld%s",
             (long)link->steps, link->step_fault ? " ERR" : "");
    if (link->startup_rows[0][0]) {
        memcpy(packet + 27, link->startup_rows, sizeof(link->startup_rows));
    }
    const uint16_t crc = link_crc16(packet + 2, LINK_MENU_SIZE - 4u);
    packet[93] = (uint8_t)crc;
    packet[94] = (uint8_t)(crc >> 8);
}
