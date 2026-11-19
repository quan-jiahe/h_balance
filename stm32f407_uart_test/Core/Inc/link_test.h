#ifndef LINK_TEST_H
#define LINK_TEST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINK_MENU_SIZE 95u
#define LINK_START 0x30u
#define LINK_STOP 0x31u
#define LINK_JOG_FORWARD 0x45u
#define LINK_JOG_BACK 0x46u

typedef struct {
    uint8_t buffer[18];
    uint8_t used;
    uint8_t last_key;
    uint8_t last_status;
    uint8_t stream_enabled;
    uint8_t menu_due;
    uint8_t seen_rx;
    uint8_t step_fault;
    uint8_t key_event;
    uint32_t last_byte_ms;
    uint32_t last_rx_ms;
    uint32_t packets;
    uint32_t observations;
    uint32_t crc_errors;
    int32_t value;
    int32_t steps;
    char startup_rows[3][22];
} LinkTest;

void link_test_init(LinkTest *link);
/* Returns START/STOP for OK, or a jog command after validating its frame. */
uint8_t link_test_feed(LinkTest *link, uint8_t byte, uint32_t now_ms);
void link_test_jog(LinkTest *link, bool forward);
void link_test_menu(const LinkTest *link, uint32_t now_ms,
                    uint32_t uart_errors, uint8_t packet[LINK_MENU_SIZE]);
void link_test_command(uint8_t command, uint8_t packet[7]);
uint16_t link_crc16(const uint8_t *data, size_t length);

#endif
