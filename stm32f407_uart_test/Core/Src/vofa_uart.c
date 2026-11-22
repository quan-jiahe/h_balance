#include "vofa_uart.h"
#include "board_mcu.h"

static uint8_t rx[128], tx[256];
static volatile uint16_t rx_head, rx_tail, tx_head, tx_tail;
static volatile uint32_t errors;

void vofa_uart_init(void)
{
#if defined(STM32F103xB)
    const uint32_t clock_hz = F103_HCLK_HZ;
#else
    const uint32_t clock_hz = 16000000u;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
#endif
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;
    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
#if defined(STM32F103xB)
    f103_gpio_config(GPIOA, 9u, F103_GPIO_ALTERNATE);
    GPIOA->BSRR = 1u << 10;
    f103_gpio_config(GPIOA, 10u, F103_GPIO_PULL);
#else
    GPIOA->MODER = (GPIOA->MODER & ~((3u << 18) | (3u << 20))) | (2u << 18) | (2u << 20);
    GPIOA->OTYPER &= ~((1u << 9) | (1u << 10));
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~((3u << 18) | (3u << 20))) | (2u << 18) | (2u << 20);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~((3u << 18) | (3u << 20))) | (1u << 18) | (1u << 20);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFFu << 4)) | (0x77u << 4);
#endif
    rx_head = rx_tail = tx_head = tx_tail = 0;
    errors = 0;
    USART1->BRR = (clock_hz + 115200u / 2u) / 115200u;
    USART1->CR2 = USART1->CR3 = 0;
    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
}

bool vofa_uart_read(uint8_t *byte)
{
    if (rx_tail == rx_head) { return false; }
    __DMB();
    *byte = rx[rx_tail];
    __DMB();
    rx_tail = (uint16_t)((rx_tail + 1u) & 127u);
    return true;
}

bool vofa_uart_write(const uint8_t *data, size_t size)
{
    uint16_t head = tx_head;
    const unsigned free = (tx_tail - head - 1u) & 255u;
    if (size > free) { return false; } /* Drop whole frames, never block the motor loop. */
    for (size_t i = 0; i < size; ++i) {
        tx[head] = data[i];
        head = (uint16_t)((head + 1u) & 255u);
    }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    tx_head = head;
    USART1->CR1 |= USART_CR1_TXEIE;
    if (!mask) { __enable_irq(); }
    return true;
}

uint32_t vofa_uart_errors(void) { return errors; }

void USART1_IRQHandler(void)
{
    const uint32_t status = USART1->SR;
    const uint32_t bad = USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE;
    if (status & (USART_SR_RXNE | bad)) {
        const uint8_t byte = (uint8_t)USART1->DR;
        const uint16_t next = (uint16_t)((rx_head + 1u) & 127u);
        if ((status & bad) || next == rx_tail) {
            ++errors;
        } else {
            rx[rx_head] = byte;
            __DMB();
            rx_head = next;
        }
    }
    if ((status & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE)) {
        if (tx_tail == tx_head) {
            USART1->CR1 &= ~USART_CR1_TXEIE;
        } else {
            USART1->DR = tx[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1u) & 255u);
        }
    }
}
