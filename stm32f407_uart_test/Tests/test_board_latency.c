#include "board.h"
#include "stm32f4xx.h"
void USART3_IRQHandler(void);

volatile uint32_t test_timer_stuck;
volatile uint32_t test_sleep_ms;
static volatile uint32_t *const result = (volatile uint32_t *)0x2001E000u;
static void check(bool ok)
{
    ++result[1];
    if (!ok) { ++result[0]; if (!result[3]) { result[3] = result[1]; } }
}
static void wait_cycles(uint32_t cycles)
{
    const uint32_t began = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - began) < cycles) { }
}
static void drain(const uint8_t *expected, unsigned size)
{
    USART3->SR = USART_SR_TXE;
    for (unsigned i = 0; i < size; ++i) {
        USART3_IRQHandler();
        check((uint8_t)USART3->DR == expected[i]);
    }
    USART3_IRQHandler();
    check(!(USART3->CR1 & USART_CR1_TXEIE));
}
int main(void)
{
    board_init();
    check((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0);
    volatile uint32_t *const button_inputs = (volatile uint32_t *)(&GPIOD->IDR);
    const uint32_t all_released = 31u << 3;
    uint32_t pressed_ms = 0;
    *button_inputs = all_released;
    board_buttons_poll(100);
    *button_inputs = all_released & ~(1u << 6);
    board_buttons_poll(104);
    board_buttons_poll(123);
    check(!board_button_take_press_at(BOARD_SW4, &pressed_ms));
    board_buttons_poll(124);
    *button_inputs = all_released;
    board_buttons_poll(130);
    check(board_button_take_press_at(BOARD_SW4, &pressed_ms) && pressed_ms == 104);
    check(!board_button_take_press_at(BOARD_SW4, &pressed_ms));
    uint32_t began = DWT->CYCCNT;
    check(board_step_pulse(true));
    check((uint32_t)(DWT->CYCCNT - began) <= 16u * 250u);
    wait_cycles(16u * 1800u); /* About 2 ms from the first pulse start. */
    began = DWT->CYCCNT;
    check(board_step_pulse(true));
    check((uint32_t)(DWT->CYCCNT - began) <= 16u * 250u);
    check(board_step_pulse(false)); /* Immediate reversal must retain DIR hold. */
    wait_cycles(16u * 4000u);
    began = DWT->CYCCNT;
    check(board_step_pulse(true));
    check((uint32_t)(DWT->CYCCNT - began) <= 16u * 250u);
    test_sleep_ms = 4; /* Wall time advances while the virtual CPU counter is stopped. */
    began = DWT->CYCCNT;
    check(board_step_pulse(false));
    check((uint32_t)(DWT->CYCCNT - began) <= 16u * 250u);
    wait_cycles(16u * 4000u);
    DWT->CYCCNT = 0xFFFFFF00u;
    check(board_step_pulse(false));
    check(board_step_pulse(true));
    wait_cycles(16u * 4000u);
    test_timer_stuck = 1;
    began = DWT->CYCCNT;
    check(!board_step_pulse(false));
    check(!(TIM1->CR1 & TIM_CR1_CEN));
    check((uint32_t)(DWT->CYCCNT - began) < 16u * 4000u);

    uint8_t packet[200];
    for (unsigned i = 0; i < sizeof(packet); ++i) { packet[i] = (uint8_t)(i * 7u); }
    USART3->SR = 0; /* Enqueue must return even while TXE is clear. */
    board_uart_write(packet, 95);
    check((USART3->CR1 & USART_CR1_TXEIE) != 0);
    drain(packet, 95);
    __disable_irq();
    board_uart_write(packet, 200);
    check(__get_PRIMASK() == 1);
    __enable_irq();
    drain(packet, 200); /* Ring wrap. */
    const uint32_t errors = board_uart_errors();
    board_uart_write(packet, 200);
    board_uart_write(packet, 100); /* Must drop the whole second frame. */
    check(board_uart_errors() == errors + 1);
    drain(packet, 200);
    board_uart_write(packet, 2);
    USART3->SR = USART_SR_RXNE | USART_SR_TXE;
    USART3->DR = 0x42;
    USART3_IRQHandler();
    uint8_t received = 0;
    check(board_uart_read(&received) && received == 0x42);
    check((uint8_t)USART3->DR == packet[0]);
    USART3->SR = USART_SR_TXE;
    USART3_IRQHandler();
    check((uint8_t)USART3->DR == packet[1]);
    USART3_IRQHandler();
    check(!(USART3->CR1 & USART_CR1_TXEIE));
    check(!board_uart_read(&received));
    result[2] = 0xF407B04Du;
    __asm volatile ("bkpt #0");
    for (;;) { }
}
