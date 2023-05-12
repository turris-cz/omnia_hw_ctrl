#ifndef GPIO_H
#define GPIO_H

#include "stm32f0xx_gpio.h"
#include "gpio_common.h"

typedef GPIOOType_TypeDef pin_out_t;
static const pin_out_t pin_pushpull = GPIO_OType_PP;
static const pin_out_t pin_opendrain = GPIO_OType_OD;

typedef GPIOPuPd_TypeDef pin_pull_t;
static const pin_pull_t pin_nopull = GPIO_PuPd_NOPULL;
static const pin_pull_t pin_pullup = GPIO_PuPd_UP;
static const pin_pull_t pin_pulldown = GPIO_PuPd_DOWN;

typedef GPIOSpeed_TypeDef pin_spd_t;
static const pin_spd_t pin_spd_1 = GPIO_Speed_Level_1;
static const pin_spd_t pin_spd_2 = GPIO_Speed_Level_2;
static const pin_spd_t pin_spd_3 = GPIO_Speed_Level_3;

static __force_inline GPIOMode_TypeDef pin_mode_to_plat(pin_mode_t mode)
{
	switch (mode) {
	case pin_mode_in: return GPIO_Mode_IN;
	case pin_mode_out: return GPIO_Mode_OUT;
	case pin_mode_alt_0 ... pin_mode_alt_7: return GPIO_Mode_AF;
	default: unreachable();
	}
}

static __force_inline uint32_t port_clk_bit(port_t port)
{
	switch (port) {
	case PORT_A: return RCC_AHBENR_GPIOAEN;
	case PORT_B: return RCC_AHBENR_GPIOBEN;
	case PORT_C: return RCC_AHBENR_GPIOCEN;
	case PORT_D: return RCC_AHBENR_GPIODEN;
	case PORT_F: return RCC_AHBENR_GPIOFEN;
	default: unreachable();
	}
}

static __force_inline GPIO_TypeDef *port_to_plat(port_t port)
{
	switch (port) {
	case PORT_A: return GPIOA;
	case PORT_B: return GPIOB;
	case PORT_C: return GPIOC;
	case PORT_D: return GPIOD;
	case PORT_F: return GPIOF;
	default: unreachable();
	}
}

static __force_inline GPIO_TypeDef *pin_port_to_plat(gpio_t pin)
{
	return port_to_plat(pin_port(pin));
}

static __force_inline bool gpio_read(gpio_t pin)
{
	if (pin == PIN_INVALID)
		return 0;

	return !!(pin_port_to_plat(pin)->IDR & pin_bit(pin));
}

static __force_inline uint16_t gpio_read_port(port_t port)
{
	return port_to_plat(port)->IDR;
}

static __force_inline bool gpio_read_output(gpio_t pin)
{
	if (pin == PIN_INVALID)
		return 0;

	return !!(pin_port_to_plat(pin)->ODR & pin_bit(pin));
}

static __force_inline void gpio_write(gpio_t pin, bool value)
{
	if (pin == PIN_INVALID)
		return;

	if (value)
		pin_port_to_plat(pin)->BSRR = pin_bit(pin);
	else
		pin_port_to_plat(pin)->BRR = pin_bit(pin);
}

static __force_inline void _pin_set_alt_fn(GPIO_TypeDef *plat, uint8_t nr,
					   pin_mode_t mode)
{
	uint8_t reg, pos;

	reg = nr >> 3;
	pos = (nr & 7) * 4;

	plat->AFR[reg] = (plat->AFR[reg] & ~(0xf << pos)) |
			 ((pin_mode_to_alt_fn(mode) & 0xf) << pos);
}

static __force_inline void gpio_set_mode(gpio_t pin, pin_mode_t mode)
{
	GPIO_TypeDef *plat;
	uint8_t nr;

	if (pin == PIN_INVALID)
		return;

	plat = pin_port_to_plat(pin);
	nr = pin_nr(pin);

	if (pin_mode_is_alt(mode))
		_pin_set_alt_fn(plat, nr, mode);

	plat->MODER = (plat->MODER & ~(0x3 << (nr * 2))) |
		      (pin_mode_to_plat(mode) << (nr * 2));
}

static inline void gpio_write_multi_list(bool value, unsigned int len,
					 const gpio_t *pins)
{
	uint16_t bits[NPORTS] = {};

	for (unsigned int i = 0; i < len; ++i)
		bits[pin_port(pins[i])] |= pin_bit(pins[i]);

	for (unsigned int i = 0; i < NPORTS; ++i) {
		if (!bits[i])
			continue;

		if (value)
			port_to_plat(i)->BSRR = bits[i];
		else
			port_to_plat(i)->BRR = bits[i];
	}
}

static inline void gpio_init_port_clks(unsigned int len, const gpio_t *pins)
{
	uint32_t clk_bits = 0;

	for (unsigned int i = 0; i < len; ++i)
		if (pins[i] != PIN_INVALID)
			clk_bits |= port_clk_bit(pin_port(pins[i]));

	RCC->AHBENR |= clk_bits;
}

static inline void gpio_init_list(pin_mode_t mode, pin_out_t otype,
				  pin_spd_t spd, pin_pull_t pull,
				  bool init_val, unsigned int len,
				  const gpio_t *pins)
{
	/* initialize port clock */
	gpio_init_port_clks(len, pins);

	/* set initial output values if output mode */
	if (mode == pin_mode_out)
		gpio_write_multi_list(init_val, len, pins);

	/* initialize individual pins */
	for (unsigned int i = 0; i < len; ++i) {
		if (pins[i] != PIN_INVALID) {
			GPIO_TypeDef *plat = pin_port_to_plat(pins[i]);
			uint8_t nr = pin_nr(pins[i]);

			/* set alt func if alt mode */
			if (pin_mode_is_alt(mode))
				_pin_set_alt_fn(plat, nr, mode);

			plat->OSPEEDR = (plat->OSPEEDR & ~(0x3 << (nr * 2))) |
					(spd << (nr * 2));
			plat->OTYPER = (plat->OTYPER & ~(0x1 << nr)) |
					(otype << nr);
			plat->MODER = (plat->MODER & ~(0x3 << (nr * 2))) |
				      (pin_mode_to_plat(mode) << (nr * 2));
			plat->PUPDR = (plat->PUPDR & ~(0x3 << (nr * 2))) |
				      (pull << (nr * 2));
		}
	}
}

#endif /* GPIO_H */
