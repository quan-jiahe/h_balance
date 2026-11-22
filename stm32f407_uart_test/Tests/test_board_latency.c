#include "board.h"
#include "board_mcu.h"
#if defined(STM32F103xB)
#include "vofa_uart.h"
#define LINK_USART USART2
#define LINK_IRQ USART2_IRQHandler
void USART1_IRQHandler(void);
#else
#define LINK_USART USART3
#define LINK_IRQ USART3_IRQHandler
#endif
void LINK_IRQ(void);

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
    LINK_USART->SR = USART_SR_TXE;
    for (unsigned i = 0; i < size; ++i) {
        LINK_IRQ();
        check((uint8_t)LINK_USART->DR == expected[i]);
    }
    LINK_IRQ();
    check(!(LINK_USART->CR1 & USART_CR1_TXEIE));
}
int main(void)
{
    board_init();
    check((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0);
    const uint32_t cycles_per_us = SystemCoreClock / 1000000u;
    check(TIM1->PSC == cycles_per_us - 1u && TIM1->ARR == 199u && TIM1->CCR1 == 100u);
#if defined(STM32F103xB)
    check(SystemCoreClock == 64000000u && SysTick->LOAD == 63999u && USART2->BRR == 278u);
    check((AFIO->MAPR & AFIO_MAPR_SWJ_CFG) == AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
    check((GPIOA->CRL & 0xFFF0u) == 0x8A20u);
    check((GPIOA->CRH & 15u) == 0xAu);
    static GPIO_TypeDef *const key_ports[] = {GPIOA, GPIOA, GPIOB, GPIOB, GPIOB};
    static const uint8_t key_pins[] = {11, 15, 3, 4, 5};
    *(volatile uint32_t *)&GPIOA->IDR = 0xFFFFu;
    *(volatile uint32_t *)&GPIOB->IDR = 0xFFFFu;
    for (unsigned i = 0; i < BOARD_BUTTON_COUNT; ++i) {
        volatile uint32_t *const input = (volatile uint32_t *)&key_ports[i]->IDR;
        const uint32_t now = 100u + i * 100u;
        uint32_t pressed_ms = 0;
        *input &= ~(1u << key_pins[i]);
        board_buttons_poll(now);
        board_buttons_poll(now + 19u);
        check(!board_button_take_press_at((BoardButton)i, &pressed_ms));
        board_buttons_poll(now + 20u);
        check(board_button_take_press_at((BoardButton)i, &pressed_ms) && pressed_ms == now);
        check(!board_button_take_press_at((BoardButton)i, &pressed_ms));
        *input = 0xFFFFu;
    }
    vofa_uart_init();
    check(USART1->BRR == 556u && (GPIOA->CRH & 0xFF0u) == 0x8A0u);
    USART1->DR = 0x21;
    USART1->SR = USART_SR_RXNE;
    USART1_IRQHandler();
    uint8_t vofa_byte = 0;
    check(vofa_uart_read(&vofa_byte) && vofa_byte == 0x21);
    check(vofa_uart_write(&vofa_byte, 1));
    USART1->SR = USART_SR_TXE;
    USART1_IRQHandler();
    check((uint8_t)USART1->DR == 0x21);
#else
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
#endif
    uint32_t began = DWT->CYCCNT;
    check(board_step_pulse(true));
    check((uint32_t)(DWT->CYCCNT - began) <= cycles_per_us * 250u);
    wait_cycles(cycles_per_us * 1800u); /* About 2 ms from the first pulse start. */
    began = DWT->CYCCNT;
    check(board_step_pulse(true));
    check((uint32_t)(DWT->CYCCNT - began) <= cycles_per_us * 250u);
    check(board_step_pulse(false)); /* Immediate reversal must retain DIR hold. */
    wait_cycles(cycles_per_us * 4000u);
    began = DWT->CYCCNT;
    check(board_step_pulse(true));
    check((uint32_t)(DWT->CYCCNT - began) <= cycles_per_us * 250u);
    test_sleep_ms = 4; /* Wall time advances while the virtual CPU counter is stopped. */
    began = DWT->CYCCNT;
    check(board_step_pulse(false));
    check((uint32_t)(DWT->CYCCNT - began) <= cycles_per_us * 250u);
    wait_cycles(cycles_per_us * 4000u);
    DWT->CYCCNT = 0xFFFFFF00u;
    check(board_step_pulse(false));
    check(board_step_pulse(true));
    wait_cycles(cycles_per_us * 4000u);
    test_timer_stuck = 1;
    began = DWT->CYCCNT;
    check(!board_step_pulse(false));
    check(!(TIM1->CR1 & TIM_CR1_CEN));
    check((uint32_t)(DWT->CYCCNT - began) < cycles_per_us * 4000u);

    uint8_t packet[200];
    for (unsigned i = 0; i < sizeof(packet); ++i) { packet[i] = (uint8_t)(i * 7u); }
    LINK_USART->SR = 0; /* Enqueue must return even while TXE is clear. */
    board_uart_write(packet, 95);
    check((LINK_USART->CR1 & USART_CR1_TXEIE) != 0);
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
    LINK_USART->SR = USART_SR_RXNE | USART_SR_TXE;
    LINK_USART->DR = 0x42;
    LINK_IRQ();
    uint8_t received = 0;
    check(board_uart_read(&received) && received == 0x42);
    check((uint8_t)LINK_USART->DR == packet[0]);
    LINK_USART->SR = USART_SR_TXE;
    LINK_IRQ();
    check((uint8_t)LINK_USART->DR == packet[1]);
    LINK_IRQ();
    check(!(LINK_USART->CR1 & USART_CR1_TXEIE));
    check(!board_uart_read(&received));
    result[2] = 0xF407B04Du;
    __asm volatile ("bkpt #0");
    for (;;) { }
}
