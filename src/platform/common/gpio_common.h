#ifndef GPIO_COMMON_H
#define GPIO_COMMON_H

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

typedef uint8_t gpio_t;

static const uint8_t PIN_INVALID = 0xff;

#define PIN_PORT_MASK	GENMASK8(7, 4)
#define PIN_NR_MASK	GENMASK8(3, 0)

#define _PIN(_port, _pin)		((gpio_t)(FIELD_PREP(PIN_PORT_MASK, PORT_ ## _port) | \
						  FIELD_PREP(PIN_NR_MASK, _pin)))
#define _PIN_3(_port, _pin, _cond)	((_cond) ? _PIN(_port, _pin) : PIN_INVALID)
#define _PIN_2(_port, _pin)		_PIN(_port, _pin)
#define PIN(...)			VARIADIC(_PIN_, __VA_ARGS__)

static __force_inline port_t pin_port(gpio_t pin)
{
	return FIELD_GET(PIN_PORT_MASK, pin);
}

static __force_inline uint8_t pin_nr(gpio_t pin)
{
	return FIELD_GET(PIN_NR_MASK, pin);
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

#define gpio_write_multi(_value, ...)					\
	({								\
		gpio_t ___pins[] = { __VA_ARGS__ };			\
		gpio_write_multi_list((_value), ARRAY_SIZE(___pins),	\
				      ___pins);				\
	})

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

#endif /* GPIO_COMMON_H */
