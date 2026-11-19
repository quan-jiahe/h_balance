#include "center_oled.h"
#ifdef F407_CENTER_BALANCE
#include "board.h"
#include "center_balance.h"
#include "stm32f4xx.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define SCL (1u << 8)
#define SDA (1u << 9)
enum { WIDTH = 128, PAGES = 8, CHUNK = 8, ANGLE_PAGE = 3,
       SETTLE_MS = 250, IMU_PERIOD_MS = 25, ZERO_SAMPLES = 8 };

static uint8_t address;
static bool imu_ready, zeroed, at_level;
static unsigned samples;
static float sum, zero_angle, angle;
static uint32_t level_since, last_sample, last_good, next_chunk, frame_start;
static unsigned page, column;
static bool clearing;
static uint8_t angle_line[WIDTH];

static void delay_half_bit(void)
{
    const uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < SystemCoreClock / 200000u) { }
}

static bool clock_high(void)
{
    const uint32_t start = DWT->CYCCNT;
    GPIOB->BSRR = SCL;
    while (!(GPIOB->IDR & SCL)) {
        if ((uint32_t)(DWT->CYCCNT - start) >= SystemCoreClock / 1000u) { return false; }
    }
    delay_half_bit();
    return true;
}

static bool write_byte(uint8_t byte)
{
    for (unsigned bit = 0x80u; bit != 0u; bit >>= 1) {
        GPIOB->BSRR = (byte & bit) ? SDA : SDA << 16;
        delay_half_bit();
        if (!clock_high()) { return false; }
        GPIOB->BSRR = SCL << 16;
    }
    GPIOB->BSRR = SDA;
    delay_half_bit();
    if (!clock_high()) { return false; }
    const bool ack = (GPIOB->IDR & SDA) == 0u;
    GPIOB->BSRR = SCL << 16;
    return ack;
}

static bool oled_write(uint8_t control, const uint8_t *data, unsigned count)
{
    GPIOB->BSRR = SDA | SCL;
    if (!clock_high() || !(GPIOB->IDR & SDA)) { return false; }
    GPIOB->BSRR = SDA << 16;
    delay_half_bit();
    GPIOB->BSRR = SCL << 16;
    bool ok = write_byte((uint8_t)(address << 1)) && write_byte(control);
    for (unsigned i = 0; ok && i < count; ++i) { ok = write_byte(data[i]); }
    GPIOB->BSRR = (SCL | SDA) << 16;
    delay_half_bit();
    const bool stopped = clock_high();
    GPIOB->BSRR = SDA | SCL;
    delay_half_bit();
    return ok && stopped;
}

void center_oled_init(void)
{
    static const uint8_t setup[] = {
        0xFD, 0x12, 0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x7F, 0xD9, 0x22,
        0xDB, 0x20, 0x2E, 0xA4, 0xA6, 0xAF
    };
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;
    GPIOB->OTYPER |= SCL | SDA;
    GPIOB->BSRR = SCL | SDA;
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(15u << 16)) | (5u << 16);
    GPIOB->MODER = (GPIOB->MODER & ~(15u << 16)) | (5u << 16);
    address = 0x3Cu;
    if (!oled_write(0, setup, sizeof(setup))) {
        address = 0x3Du;
        if (!oled_write(0, setup, sizeof(setup))) { address = 0; }
    }
    if (address) {
        imu_ready = board_imu_init();
        clearing = true;
    }
}

static void update_angle(const LinkTest *link, uint32_t now)
{
    const int32_t level = center_balance_level_steps();
    const bool parked = level >= 0 && link->steps == level && !link->step_fault;
    if (!zeroed && (!parked || !at_level)) {
        level_since = now;
        samples = 0;
        sum = 0.0f;
    }
    at_level = parked;
    if (!imu_ready || now - last_sample < IMU_PERIOD_MS) { return; }
    last_sample = now;
    int16_t raw[3];
    if (board_imu_read(raw) != 1) { return; }
    const float x = raw[0], y = raw[1], z = raw[2];
    const float horizontal_sq = x * x + y * y;
    const float gravity_sq = horizontal_sq + z * z;
    if (gravity_sq < 13000.0f * 13000.0f || gravity_sq > 19800.0f * 19800.0f) { return; }
    const float reading = atan2f(z, sqrtf(horizontal_sq)) * 57.29578f;
    if (zeroed) {
        angle += (reading - zero_angle - angle) * 0.25f;
    } else if (parked && now - level_since >= SETTLE_MS) {
        sum += reading;
        if (++samples == ZERO_SAMPLES) {
            zero_angle = sum / ZERO_SAMPLES;
            angle = 0.0f;
            zeroed = true;
        }
    }
    last_good = now;
}

static const uint8_t glyphs[][5] = {
    {0x3e,0x51,0x49,0x45,0x3e}, {0,0x42,0x7f,0x40,0},
    {0x62,0x51,0x49,0x49,0x46}, {0x22,0x49,0x49,0x49,0x36},
    {0x18,0x14,0x12,0x7f,0x10}, {0x2f,0x49,0x49,0x49,0x31},
    {0x3e,0x49,0x49,0x49,0x32}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x26,0x49,0x49,0x49,0x3e},
    {0x7e,0x11,0x11,0x11,0x7e}, {0x7f,0x49,0x49,0x49,0x36},
    {0x3e,0x41,0x41,0x41,0x22}, {0x7f,0x41,0x41,0x22,0x1c},
    {0x7f,0x49,0x49,0x49,0x41}, {0x7f,0x09,0x09,0x09,0x01},
    {0x3e,0x41,0x49,0x49,0x7a}, {0,0x41,0x7f,0x41,0},
    {0x7f,0x40,0x40,0x40,0x40}, {0x7f,0x02,0x0c,0x02,0x7f},
    {0x7f,0x02,0x04,0x08,0x7f}, {0x3e,0x41,0x41,0x41,0x3e},
    {0x7f,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7f,0x01,0x01}, {0x3f,0x40,0x38,0x40,0x3f},
    {0x61,0x51,0x49,0x45,0x43}, {0x08,0x08,0x3e,0x08,0x08},
    {0x08,0x08,0x08,0x08,0x08}, {0,0x60,0x60,0,0},
};
static const char glyph_keys[] = "0123456789ABCDEFGILMNORSTWZ+-.";

static void draw_line(uint8_t *line, const char *text)
{
    memset(line, 0, WIDTH);
    for (unsigned x = 4; *text && x + 5u <= WIDTH; x += 6u, ++text) {
        const char *glyph = strchr(glyph_keys, *text);
        if (glyph) { memcpy(line + x, glyphs[glyph - glyph_keys], 5); }
    }
}

static void make_frame(uint32_t now)
{
    char line[22];
    if (!imu_ready || now - last_good > 200u) {
        draw_line(angle_line, "ANGLE IMU ERR");
    } else if (!zeroed) {
        draw_line(angle_line, "ANGLE WAIT ZERO");
    } else {
        const float scaled = angle * 100.0f;
        const int hundredths = (int)(scaled + (scaled < 0.0f ? -0.5f : 0.5f));
        const unsigned magnitude = (unsigned)(hundredths < 0 ? -hundredths : hundredths);
        snprintf(line, sizeof(line), "ANGLE %c%u.%02u DEG",
                 hundredths < 0 ? '-' : ' ', magnitude / 100u, magnitude % 100u);
        draw_line(angle_line, line);
    }
}

void center_oled_tick(const LinkTest *link, uint32_t now)
{
    static const uint8_t blank[CHUNK] = {0};
    if (!address) { return; }
    update_angle(link, now);
    if ((int32_t)(now - next_chunk) < 0) { return; }
    if (!clearing && column == 0) {
        if (now - frame_start < 250u) { return; }
        frame_start = now;
        make_frame(now);
    }
    const uint8_t command[] = {
        (uint8_t)(0xB0u | page), (uint8_t)(column & 15u), (uint8_t)(0x10u | (column >> 4))
    };
    const uint8_t *data = clearing ? blank : angle_line + column;
    next_chunk = now + 8u;
    if (!oled_write(0, command, sizeof(command)) || !oled_write(0x40, data, CHUNK)) {
        next_chunk = now + 250u;
        return;
    }
    column += CHUNK;
    if (column < WIDTH) { return; }
    column = 0;
    if (clearing && ++page < PAGES) { return; }
    page = ANGLE_PAGE;
    clearing = false;
}
#endif
