#ifndef BOARD_MCU_H
#define BOARD_MCU_H

#if defined(STM32F103xB)
#include "stm32f1xx.h"

/* HSI/2 x16, with APB1 divided by two and APB2 undivided. */
enum { F103_HCLK_HZ = 64000000, F103_PCLK1_HZ = F103_HCLK_HZ / 2 };

/* The STM32F1 GPIO control nibble is CNF[1:0]:MODE[1:0]. */
enum {
    F103_GPIO_FLOATING = 0x4u,
    F103_GPIO_PULL = 0x8u,
    F103_GPIO_OUTPUT = 0x2u,
    F103_GPIO_OPEN_DRAIN = 0x6u,
    F103_GPIO_ALTERNATE = 0xAu
};

static inline void f103_gpio_config(GPIO_TypeDef *port, unsigned pin, uint32_t mode)
{
    volatile uint32_t *control = pin < 8u ? &port->CRL : &port->CRH;
    const unsigned shift = (pin & 7u) * 4u;
    *control = (*control & ~(15u << shift)) | (mode << shift);
}
#else
#include "stm32f4xx.h"
#endif

#endif
