#ifndef GPIO_H
#define GPIO_H

#include <stdarg.h>
#include "stm32f0xx_gpio.h"
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

static __force_inline GPIOMode_TypeDef pin_mode_to_plat(pin_mode_t mode)
{
	switch (mode) {
	case pin_mode_in: return GPIO_Mode_IN;
	case pin_mode_out: return GPIO_Mode_OUT;
	case pin_mode_alt_0 ... pin_mode_alt_7: return GPIO_Mode_AF;
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
			if (pin_mode_is_alt(mode)) {
				uint8_t r = nr >> 3, pos = (nr & 7) * 4;

				plat->AFR[r] = (plat->AFR[r] & ~(0xf << pos)) |
					       ((pin_mode_to_alt_fn(mode) & 0xf) << pos);
			}

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
