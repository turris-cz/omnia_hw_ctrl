#ifndef GPIO_COMMON_STM32_GD32_H
#define GPIO_COMMON_STM32_GD32_H

#include "bits.h"

typedef enum {
	PORT_A = 0,
	PORT_B,
	PORT_C,
	PORT_D,
	PORT_F,
} port_t;

#define NPORTS 5

#define PIN_PORT_MASK	GENMASK8(7, 4)
#define PIN_NR_MASK	GENMASK8(3, 0)

#define PIN_BIT_TYPE_T	uint16_t

#include "gpio_common.h"

#if !defined(__STM32F0XX_H) && !defined(GD32F1X0_H)
# error "stm32f0xx.h or gd32f1x0.h must be included before including gpio_common_stm32_gd32.h"
#endif

static __force_inline uint8_t pin_irqn(gpio_t pin)
{
	switch (pin_nr(pin)) {
	case 0 ... 1: return EXTI0_1_IRQn;
	case 2 ... 3: return EXTI2_3_IRQn;
	case 4 ... 15: return EXTI4_15_IRQn;
	default: unreachable();
	}
}

#endif /* GPIO_COMMON_STM32_GD32_H */
