#ifndef GPIO_H
#define GPIO_H

#include <stdarg.h>
#include "gd32f1x0_gpio.h"
#include "gd32f1x0_rcu.h"
#include "compiler.h"
#include "bits.h"

typedef enum {
	PORT_A = 0,
	PORT_B,
	PORT_C,
	PORT_D,
	PORT_F,
} port_t;

#define NPORTS 5

typedef enum {
	pin_mode_in,
	pin_mode_out,
	pin_mode_alt_0,
	pin_mode_alt_1,
	pin_mode_alt_2,
	pin_mode_alt_3,
	pin_mode_alt_4,
	pin_mode_alt_5,
	pin_mode_alt_6,
	pin_mode_alt_7,
} pin_mode_t;

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

typedef uint8_t gpio_t;

static const uint8_t PIN_INVALID = 0xff;

#define _PIN(_port, _pin)		((gpio_t)((PORT_ ## _port) << 4) | ((_pin) & 0xf))
#define _PIN_3(_port, _pin, _cond)	((_cond) ? _PIN(_port, _pin) : PIN_INVALID)
#define _PIN_2(_port, _pin)		_PIN(_port, _pin)
#define PIN(...)			VARIADIC(_PIN_, __VA_ARGS__)

static __force_inline port_t pin_port(gpio_t pin)
{
	return pin >> 4;
}

static __force_inline uint8_t pin_nr(gpio_t pin)
{
	return pin & 0xf;
}

static __force_inline uint16_t pin_bit(gpio_t pin)
{
	if (pin == PIN_INVALID)
		return 0;
	else
		return BIT(pin_nr(pin));
}

static __force_inline pin_mode_t pin_mode_alt(uint8_t alt_fn)
{
	switch (alt_fn) {
	case 0: return pin_mode_alt_0;
	case 1: return pin_mode_alt_1;
	case 2: return pin_mode_alt_2;
	case 3: return pin_mode_alt_3;
	case 4: return pin_mode_alt_4;
	case 5: return pin_mode_alt_5;
	case 6: return pin_mode_alt_6;
	case 7: return pin_mode_alt_7;
	default: unreachable();
	}
}

static __force_inline uint8_t pin_mode_to_plat(pin_mode_t mode)
{
	switch (mode) {
	case pin_mode_in: return GPIO_MODE_INPUT;
	case pin_mode_out: return GPIO_MODE_OUTPUT;
	case pin_mode_alt_0 ... pin_mode_alt_7: return GPIO_MODE_AF;
	default: unreachable();
	}
}

static __force_inline bool pin_mode_is_alt(pin_mode_t mode)
{
	return mode != pin_mode_in && mode != pin_mode_out;
}

static __force_inline uint8_t pin_mode_to_alt_fn(pin_mode_t mode)
{
	switch (mode) {
	case pin_mode_alt_0: return 0;
	case pin_mode_alt_1: return 1;
	case pin_mode_alt_2: return 2;
	case pin_mode_alt_3: return 3;
	case pin_mode_alt_4: return 4;
	case pin_mode_alt_5: return 5;
	case pin_mode_alt_6: return 6;
	case pin_mode_alt_7: return 7;
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

#define gpio_write_multi(_value, ...)					\
	({								\
		gpio_t ___pins[] = { __VA_ARGS__ };			\
		gpio_write_multi_list((_value), ARRAY_SIZE(___pins),	\
				      ___pins);				\
	})

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

#define gpio_init_inputs(_pull, ...)				\
	({							\
		gpio_t ___pins[] = { __VA_ARGS__ };		\
		gpio_init_list(pin_mode_in, 0, 0, (_pull), 0,	\
			       ARRAY_SIZE(___pins), ___pins);	\
	})

#define gpio_init_outputs(_otype, _spd, _init_val, ...)		\
	({							\
		gpio_t ___pins[] = { __VA_ARGS__ };		\
		gpio_init_list(pin_mode_out, (_otype), (_spd),	\
			       pin_pullup, (_init_val),		\
			       ARRAY_SIZE(___pins), ___pins);	\
	})

#define gpio_init_alts(_alt_fn, _otype, _spd, _pull, ...)		\
	({								\
		gpio_t ___pins[] = { __VA_ARGS__ };			\
		gpio_init_list(pin_mode_alt(_alt_fn), (_otype),		\
			       (_spd), (_pull), 0, ARRAY_SIZE(___pins),	\
			       ___pins);				\
	})

#endif /* GPIO_H */
