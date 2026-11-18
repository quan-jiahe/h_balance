#include "board.h"
#include "stm32f4xx.h"

uint32_t SystemCoreClock = 16000000u;
static volatile uint32_t ticks_ms;
static volatile uint32_t uart_errors;
static uint8_t rx_buffer[256];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

/* Common-anode STEP input: idle high, pulse low. Clear CC1P for active high. */
#define STEP_POLARITY TIM_CCER_CC1P
#define STEP_DIR_PIN (1u << 7)

void SystemInit(void)
{
    SCB->CPACR |= (0xFu << 20);
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }
    RCC->CFGR &= ~RCC_CFGR_SW;
    while ((RCC->CFGR & RCC_CFGR_SWS) != 0u) { }
    RCC->CFGR = 0;
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON | RCC_CR_CSSON);
    FLASH->ACR = 0;
    SCB->VTOR = FLASH_BASE;
    /* SystemInit runs before C data initialization; SystemCoreClock is initialized later. */
}

void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 16000000u;
}

static void step_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;

    /* PWM2 holds STEP idle until CCR1, then drives it until the one-pulse update. */
    TIM1->CR1 = TIM_CR1_OPM;
    TIM1->PSC = 15u;  /* 16 MHz TIM1 clock, 1 us per count. */
    TIM1->ARR = 199u;
    TIM1->CCR1 = 100u;
    TIM1->CCMR1 = TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CCER = TIM_CCER_CC1E | STEP_POLARITY;
    TIM1->BDTR = TIM_BDTR_MOE;

    /* Attach PE9 to the idle timer output and PE7 to the direction latch. */
    GPIOE->BSRR = STEP_DIR_PIN;
    GPIOE->OTYPER &= ~(STEP_DIR_PIN | (1u << 9));
    GPIOE->AFR[1] = (GPIOE->AFR[1] & ~(15u << 4)) | (1u << 4);
    GPIOE->MODER = (GPIOE->MODER & ~((3u << 14) | (3u << 18)))
                   | (1u << 14) | (2u << 18);
}

void board_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    (void)RCC->APB1ENR;
    RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;

    /* PD8=USART3_TX, PD9=USART3_RX, alternate function AF7. */
    GPIOD->MODER = (GPIOD->MODER & ~((3u << 16) | (3u << 18)))
                   | (2u << 16) | (2u << 18);
    GPIOD->OTYPER &= ~((1u << 8) | (1u << 9));
    GPIOD->OSPEEDR = (GPIOD->OSPEEDR & ~((3u << 16) | (3u << 18)))
                     | (2u << 16) | (2u << 18);
    GPIOD->PUPDR = (GPIOD->PUPDR & ~((3u << 16) | (3u << 18)))
                   | (1u << 16) | (1u << 18);
    GPIOD->AFR[1] = (GPIOD->AFR[1] & ~0xFFu) | 0x77u;

    USART3->BRR = (16000000u + 115200u / 2u) / 115200u;
    USART3->CR2 = 0;
    USART3->CR3 = 0;
    NVIC_SetPriority(USART3_IRQn, 1);
    NVIC_EnableIRQ(USART3_IRQn);
    USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    (void)SysTick_Config(SystemCoreClock / 1000u);
    step_init();
}

bool board_step_pulse(bool forward)
{
    GPIOE->BSRR = forward ? STEP_DIR_PIN : STEP_DIR_PIN << 16;
    TIM1->SR = 0;
    TIM1->CR1 = TIM_CR1_OPM | TIM_CR1_CEN;
    const uint32_t start = board_millis();
    while (TIM1->CR1 & TIM_CR1_CEN) {
        if ((uint32_t)(board_millis() - start) >= 3u) {
            TIM1->CR1 = TIM_CR1_OPM;
            TIM1->CNT = 0;
            return false;
        }
    }
    /* Keep DIR stable after the final STEP edge even for queued clicks. */
    const uint32_t end = board_millis();
    while ((uint32_t)(board_millis() - end) < 2u) { }
    return (TIM1->SR & TIM_SR_UIF) != 0u;
}

void SysTick_Handler(void)
{
    ++ticks_ms;
}

void USART3_IRQHandler(void)
{
    const uint32_t status = USART3->SR;
    const uint32_t errors = USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE;
    if (status & (USART_SR_RXNE | errors)) {
        const uint8_t byte = (uint8_t)USART3->DR;
        if (status & errors) {
            ++uart_errors;
        } else {
            const uint16_t next = (uint16_t)((rx_head + 1u) & 255u);
            if (next == rx_tail) {
                ++uart_errors;
            } else {
                rx_buffer[rx_head] = byte;
                __DMB();
                rx_head = next;
            }
        }
    }
}

uint32_t board_millis(void)
{
    return ticks_ms;
}

uint32_t board_uart_errors(void)
{
    return uart_errors;
}

bool board_uart_read(uint8_t *byte)
{
    if (rx_tail == rx_head) { return false; }
    __DMB();
    *byte = rx_buffer[rx_tail];
    __DMB();
    rx_tail = (uint16_t)((rx_tail + 1u) & 255u);
    return true;
}

void board_uart_write(const uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; ++i) {
        while (!(USART3->SR & USART_SR_TXE)) { }
        USART3->DR = data[i];
    }
}
