#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    BOARD_SW1,
    BOARD_SW2,
    BOARD_SW3,
    BOARD_SW4,
    BOARD_SW5,
    BOARD_BUTTON_COUNT
} BoardButton;

typedef enum {
    BOARD_LED3,
    BOARD_LED4,
    BOARD_LED5,
    BOARD_LED_COUNT
} BoardLed;

void board_init(void);
uint32_t board_millis(void);
/* Poll from the main loop and pass the current value of board_millis(). */
void board_buttons_poll(uint32_t now_ms);
bool board_button_is_pressed(BoardButton button);
/* Consume a pending press. At most one press is stored per button. */
bool board_button_take_press(BoardButton button);
void board_led_set(BoardLed led, bool on);
bool board_uart_read(uint8_t *byte);
void board_uart_write(const uint8_t *data, size_t length);
uint32_t board_uart_errors(void);
bool board_step_pulse(bool forward);
bool board_imu_init(void);
/* Returns 1 for fresh data, 0 while waiting, and -1 on a bus/identity error. */
int board_imu_read(int16_t xyz[3]);

#endif
