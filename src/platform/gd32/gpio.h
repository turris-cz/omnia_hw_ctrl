#ifndef GPIO_H
#define GPIO_H

#include "gpio_common.h"
#include "gd32f1x0_gpio.h"
#include "gd32f1x0_rcu.h"

typedef enum {
	pin_pushpull = GPIO_OTYPE_PP,
	pin_opendrain = GPIO_OTYPE_OD,
} pin_out_t;

typedef enum {
	pin_nopull = GPIO_PUPD_NONE,
	pin_pullup = GPIO_PUPD_PULLUP,
	pin_pulldown = GPIO_PUPD_PULLDOWN,
} pin_pull_t;

typedef enum {
	pin_spd_1 = GPIO_OSPEED_2MHZ,
	pin_spd_2 = GPIO_OSPEED_10MHZ,
	pin_spd_3 = GPIO_OSPEED_50MHZ,
} pin_spd_t;

static __force_inline uint8_t pin_mode_to_plat(pin_mode_t mode)
{
	switch (mode) {
	case pin_mode_in: return GPIO_MODE_INPUT;
	case pin_mode_out: return GPIO_MODE_OUTPUT;
	case pin_mode_alt_0 ... pin_mode_alt_7: return GPIO_MODE_AF;
	default: unreachable();
	}
}

static __force_inline uint32_t port_clk_bit(port_t port)
{
	switch (port) {
	case PORT_A: return RCU_AHBEN_PAEN;
	case PORT_B: return RCU_AHBEN_PBEN;
	case PORT_C: return RCU_AHBEN_PCEN;
	case PORT_D: return RCU_AHBEN_PDEN;
	case PORT_F: return RCU_AHBEN_PFEN;
	default: unreachable();
	}
}

static __force_inline uint32_t port_to_plat(port_t port)
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

static __force_inline uint32_t pin_port_to_plat(gpio_t pin)
{
	return port_to_plat(pin_port(pin));
}

static __force_inline bool gpio_read(gpio_t pin)
{
	if (pin == PIN_INVALID)
		return 0;

	return !!(GPIO_ISTAT(pin_port_to_plat(pin)) & pin_bit(pin));
}

static __force_inline uint16_t gpio_read_port(port_t port)
{
	return GPIO_ISTAT(port_to_plat(port));
}

static __force_inline bool gpio_read_output(gpio_t pin)
{
	if (pin == PIN_INVALID)
		return false;

	return !!(GPIO_OCTL(pin_port_to_plat(pin)) & pin_bit(pin));
}

static __force_inline void gpio_write(gpio_t pin, bool value)
{
	if (pin == PIN_INVALID)
		return;

	if (value)
		GPIO_BOP(pin_port_to_plat(pin)) = pin_bit(pin);
	else
		GPIO_BC(pin_port_to_plat(pin)) = pin_bit(pin);
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
			GPIO_BOP(port_to_plat(i)) = bits[i];
		else
			GPIO_BC(port_to_plat(i)) = bits[i];
	}
}

static inline void gpio_init_port_clks(unsigned int len, const gpio_t *pins)
{
	uint32_t clk_bits = 0;

	for (unsigned int i = 0; i < len; ++i)
		if (pins[i] != PIN_INVALID)
			clk_bits |= port_clk_bit(pin_port(pins[i]));

	RCU_AHBEN |= clk_bits;
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
			uint32_t plat = pin_port_to_plat(pins[i]);
			uint8_t nr = pin_nr(pins[i]);

			/* set alt func if alt mode */
			if (pin_mode_is_alt(mode)) {
				if (nr < 8)
					GPIO_AFSEL0(plat) = (GPIO_AFSEL0(plat) & ~GPIO_AFR_MASK(nr & 7)) |
							    GPIO_AFR_SET(nr & 7, pin_mode_to_alt_fn(mode));
				else
					GPIO_AFSEL1(plat) = (GPIO_AFSEL1(plat) & ~GPIO_AFR_MASK(nr & 7)) |
							    GPIO_AFR_SET(nr & 7, pin_mode_to_alt_fn(mode));
			}

			GPIO_CTL(plat) = (GPIO_CTL(plat) & ~GPIO_MODE_MASK(nr)) |
					 GPIO_MODE_SET(nr, pin_mode_to_plat(mode));
			GPIO_PUD(plat) = (GPIO_PUD(plat) & ~GPIO_PUPD_MASK(nr)) |
					 GPIO_PUPD_SET(nr, pull);
			GPIO_OMODE(plat) = (GPIO_OMODE(plat) & ~BIT(nr)) |
					   (otype << nr);
			GPIO_OSPD(plat) = (GPIO_OSPD(plat) & ~GPIO_OSPEED_MASK(nr)) |
					  GPIO_OSPEED_SET(nr, spd);
		}
	}
}

#endif /* GPIO_H */
