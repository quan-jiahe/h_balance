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

enum { FIRST_BUTTON_PIN = 3, BUTTON_DEBOUNCE_MS = 20 };
#define BUTTON_MODE_MASK (((1u << (2u * BOARD_BUTTON_COUNT)) - 1u) << (2u * FIRST_BUTTON_PIN))
#define LED3_PIN (1u << 10)
#define LED4_PIN (1u << 11)
#define LED5_PIN (1u << 8)

typedef struct {
    uint32_t raw_changed_ms;
    bool raw_pressed;
    bool stable_pressed;
    bool press_pending;
} ButtonState;

static ButtonState buttons[BOARD_BUTTON_COUNT];

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

static void panel_io_init(void)
{
    const uint32_t pd_led_modes = (3u << 20) | (3u << 22);
    const uint32_t pa_led_mode = 3u << 16;

    /* SW1-SW5 already have external pull-ups and pull low when pressed. */
    GPIOD->PUPDR &= ~(BUTTON_MODE_MASK | pd_led_modes);
    GPIOA->PUPDR &= ~pa_led_mode;

    /* Preload the active-high LEDs low before GPIO output is enabled. */
    GPIOD->BSRR = (LED3_PIN | LED4_PIN) << 16;
    GPIOA->BSRR = LED5_PIN << 16;
    GPIOD->OTYPER &= ~(LED3_PIN | LED4_PIN);
    GPIOA->OTYPER &= ~LED5_PIN;
    GPIOD->OSPEEDR &= ~pd_led_modes;
    GPIOA->OSPEEDR &= ~pa_led_mode;
    GPIOD->MODER = (GPIOD->MODER & ~(BUTTON_MODE_MASK | pd_led_modes)) |
                   (1u << 20) | (1u << 22);
    GPIOA->MODER = (GPIOA->MODER & ~pa_led_mode) | (1u << 16);
}

void board_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN;
    (void)RCC->AHB1ENR;
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
    panel_io_init();
    step_init();
}

void board_buttons_poll(uint32_t now_ms)
{
    const uint32_t input = GPIOD->IDR;
    for (unsigned i = 0; i < BOARD_BUTTON_COUNT; ++i) {
        ButtonState *const state = &buttons[i];
        const bool pressed = (input & (1u << (FIRST_BUTTON_PIN + i))) == 0u;
        if (pressed != state->raw_pressed) {
            state->raw_pressed = pressed;
            state->raw_changed_ms = now_ms;
        } else if (pressed != state->stable_pressed &&
                   now_ms - state->raw_changed_ms >= BUTTON_DEBOUNCE_MS) {
            state->stable_pressed = pressed;
            if (pressed) { state->press_pending = true; }
        }
    }
}

bool board_button_is_pressed(BoardButton button)
{
    return (unsigned)button < BOARD_BUTTON_COUNT && buttons[button].stable_pressed;
}

bool board_button_take_press(BoardButton button)
{
    if ((unsigned)button >= BOARD_BUTTON_COUNT) { return false; }
    const bool pending = buttons[button].press_pending;
    buttons[button].press_pending = false;
    return pending;
}

void board_led_set(BoardLed led, bool on)
{
    static const struct {
        GPIO_TypeDef *port;
        uint32_t pin;
    } led_gpio[BOARD_LED_COUNT] = {
        [BOARD_LED3] = {GPIOD, LED3_PIN},
        [BOARD_LED4] = {GPIOD, LED4_PIN},
        [BOARD_LED5] = {GPIOA, LED5_PIN}
    };
    if ((unsigned)led >= BOARD_LED_COUNT) { return; }
    const uint32_t pin = led_gpio[led].pin;
    led_gpio[led].port->BSRR = on ? pin : pin << 16;
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

#define IMU_CS (1u << 0)
#define IMU_WHO_AM_I 0x72u
#define IMU_ID 0xE9u
#define IMU_INT1_STATUS 0x19u
#define IMU_DATA_READY (1u << 2)
static bool imu_big_endian;

static bool spi_wait(uint32_t mask, bool set)
{
    const uint32_t started = board_millis();
    while (((SPI2->SR & mask) != 0u) != set) {
        if ((uint32_t)(board_millis() - started) >= 2u) { return false; }
    }
    return true;
}

static bool spi_byte(uint8_t out, uint8_t *in)
{
    volatile uint8_t *const data = (volatile uint8_t *)&SPI2->DR;
    if (!spi_wait(SPI_SR_TXE, true)) { return false; }
    *data = out;
    if (!spi_wait(SPI_SR_RXNE, true)) { return false; }
    *in = *data;
    return true;
}

static bool imu_transfer(uint8_t address, uint8_t *data, size_t count, bool read)
{
    uint8_t discarded;
    GPIOB->BSRR = IMU_CS << 16;
    bool ok = spi_byte(read ? (uint8_t)(address | 0x80u) : address, &discarded);
    for (size_t i = 0; ok && i < count; ++i) {
        ok = read ? spi_byte(0, data + i) : spi_byte(data[i], &discarded);
    }
    const bool idle = spi_wait(SPI_SR_BSY, false);
    GPIOB->BSRR = IMU_CS;
    return ok && idle;
}

static bool imu_write(uint8_t address, uint8_t value)
{
    return imu_transfer(address, &value, 1, false);
}

static void delay_ms(uint32_t duration)
{
    const uint32_t started = board_millis();
    while ((uint32_t)(board_millis() - started) <= duration) { __WFI(); }
}

bool board_imu_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    (void)RCC->APB1ENR;
    GPIOB->BSRR = IMU_CS;
    GPIOB->OTYPER &= ~(IMU_CS | (7u << 13));
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFFFu << 20)) | (0x555u << 20);
    GPIOB->MODER = (GPIOB->MODER & ~((0x3Fu << 26) | 3u)) | (0x2Au << 26) | 1u;
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1 | SPI_CR1_BR_0;
    SPI2->CR1 |= SPI_CR1_SPE;  /* Mode 0, 1 MHz, four wires. */

    delay_ms(10);
    uint8_t value;
    if (!imu_transfer(IMU_WHO_AM_I, &value, 1, true) || value != IMU_ID
            || !imu_write(0x7Fu, 0x02u)) { return false; }
    delay_ms(2);
    if (!imu_transfer(IMU_INT1_STATUS, &value, 1, true) || !(value & 0x80u)) {
        return false;
    }
    /* Read indirect SREG_CTRL (0xA267) to honor the chip's data byte order. */
    uint8_t indirect_address[] = {0xA2u, 0x67u};
    if (!imu_transfer(0x7Cu, indirect_address, sizeof(indirect_address), false)) {
        return false;
    }
    delay_ms(1);
    if (!imu_transfer(0x7Eu, &value, 1, true)) { return false; }
    imu_big_endian = (value & 2u) != 0u;
    return imu_write(0x16u, IMU_DATA_READY)
        && imu_write(0x1Bu, 0x49u)  /* ±2 g, 100 Hz. */
        && imu_write(0x10u, 0x03u); /* Low-noise accelerometer. */
}

int board_imu_read(int16_t xyz[3])
{
    uint8_t value;
    if (!imu_transfer(IMU_WHO_AM_I, &value, 1, true) || value != IMU_ID
            || !imu_transfer(IMU_INT1_STATUS, &value, 1, true)) { return -1; }
    if (!(value & IMU_DATA_READY)) { return 0; }
    uint8_t data[6];
    if (!imu_transfer(0x00u, data, sizeof(data), true)) { return -1; }
    const unsigned upper = imu_big_endian ? 0u : 1u;
    for (unsigned i = 0; i < 3u; ++i) {
        xyz[i] = (int16_t)((uint16_t)data[2u * i + upper] << 8
                           | data[2u * i + (1u - upper)]);
    }
    return 1;
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
